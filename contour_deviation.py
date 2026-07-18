"""
contour_deviation.py

Reference implementation for measuring the geometric deviation between a
robot-drawn line trace and its reference outline, both extracted from the
same photograph.

Pipeline overview
------------------
1. Scale calibration  - derive a px -> mm ratio from a checkerboard target
                         of known physical size, visible in the photo.
2. Color segmentation  - separate the two overlaid curves (reference vs.
                         robot trace) by HSV color thresholding.
3. Skeletonization     - thin each curve to a 1-pixel-wide centerline.
4. Spur pruning        - remove short false branches produced by ink pooling.
5. Nearest-neighbor distance (core algorithm) - for every point on the
                         reference curve, find its closest point on the
                         robot-trace curve using a KD-tree, giving a
                         per-point deviation profile in millimeters.
6. Peak selection (NMS) - pick the top-K worst deviation points for
                         callout annotation, suppressing near-duplicate
                         peaks that sit on the same local error region.

Steps 5 and 6 are the actual measurement algorithm; steps 1-4 are
preprocessing needed to get two clean point sets to compare.
"""

import cv2
import numpy as np
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize


# ---------------------------------------------------------------------------
# Step 1: scale calibration
# ---------------------------------------------------------------------------

def calibrate_scale_from_checkerboard(large_block_px, small_block_px,
                                       large_block_mm=10.0, small_block_mm=5.0):
    """Derive px-per-mm from a checkerboard target with two known block
    sizes, and cross-check one against the other.

    large_block_px / small_block_px: mean measured pixel width of one
    checkerboard square in each size class (measured from black/white
    transition positions along a scan line through the target).
    """
    px_per_mm_from_large = large_block_px / large_block_mm
    px_per_mm_from_small = small_block_px / small_block_mm
    return (px_per_mm_from_large + px_per_mm_from_small) / 2.0


# ---------------------------------------------------------------------------
# Steps 2-4: extract a clean 1px-wide centerline for a curve of a given color
# ---------------------------------------------------------------------------

def extract_curve_mask(bgr_image, hsv_range, exclude_boxes=None):
    """Threshold the image in HSV space to isolate one curve's ink color.

    hsv_range: dict with keys 'h', 's', 'v', each a (min, max) tuple, OR
    for hues that wrap around 0/180 (e.g. red), 'h' may be a list of
    (min, max) tuples that are OR-ed together.
    """
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    h_ranges = hsv_range['h'] if isinstance(hsv_range['h'], list) else [hsv_range['h']]
    h_mask = np.zeros(h.shape, dtype=bool)
    for lo, hi in h_ranges:
        h_mask |= (h >= lo) & (h <= hi)

    s_lo, s_hi = hsv_range.get('s', (0, 255))
    v_lo, v_hi = hsv_range.get('v', (0, 255))
    mask = h_mask & (s >= s_lo) & (s <= s_hi) & (v >= v_lo) & (v <= v_hi)

    if exclude_boxes:
        for (x0, y0, x1, y1) in exclude_boxes:
            mask[y0:y1, x0:x1] = False

    return mask


def clean_and_skeletonize(mask, min_component_area=15):
    """Morphological cleanup + connected-component filtering, then thin the
    curve to a 1-pixel-wide skeleton."""
    m = (mask.astype(np.uint8)) * 255
    kernel = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=1)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    clean = np.zeros_like(m)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_component_area:
            clean[labels == i] = 255

    return skeletonize(clean > 0)


def prune_skeleton_spurs(skeleton, min_branch_len=6, max_passes=20):
    """Remove short spur branches from a boolean skeleton image.

    Marker ink pooling/blobbing can create short false branches off the
    main centerline after skeletonization. This walks outward from every
    skeleton endpoint; if the walk terminates within `min_branch_len`
    pixels at a branch point (a pixel with >= 3 neighbors), the short
    branch is deleted. Repeated until no more spurs are found.
    """
    skeleton = skeleton.copy()

    def neighbors_of(p, coords):
        x, y = p
        return [(x + dx, y + dy)
                for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                if (dx, dy) != (0, 0) and (x + dx, y + dy) in coords]

    for _ in range(max_passes):
        ys, xs = np.nonzero(skeleton)
        if len(xs) == 0:
            break
        coords = set(zip(xs.tolist(), ys.tolist()))
        endpoints = [p for p in coords if len(neighbors_of(p, coords)) == 1]
        if not endpoints:
            break

        to_remove = set()
        for ep in endpoints:
            path, visited, cur = [ep], {ep}, ep
            for _ in range(min_branch_len):
                nbrs = [n for n in neighbors_of(cur, coords) if n not in visited]
                if len(nbrs) != 1:
                    break
                cur = nbrs[0]
                path.append(cur)
                visited.add(cur)
            else:
                continue  # walked the full length without hitting a branch point: not a spur
            if len(neighbors_of(cur, coords)) >= 3:
                to_remove.update(path[:-1])

        if not to_remove:
            break
        for (x, y) in to_remove:
            skeleton[y, x] = False

    return skeleton


def skeleton_to_points(skeleton):
    """Convert a boolean skeleton image to an (N, 2) array of (x, y) points."""
    ys, xs = np.nonzero(skeleton)
    return np.stack([xs, ys], axis=1).astype(np.float64)


# ---------------------------------------------------------------------------
# Step 5 (core algorithm): nearest-neighbor deviation
# ---------------------------------------------------------------------------

def nearest_neighbor_deviation(reference_points, measured_points, px_per_mm):
    """For every point on the reference curve, find the distance to its
    nearest point on the measured (robot-drawn) curve.

    This is a one-directional nearest-neighbor query: it answers "how far
    is the reference curve, at this point, from the nearest ink the robot
    actually drew?" It does NOT require the two curves to have matching
    point counts or matching arc-length parameterization -- a KD-tree makes
    the nearest-point lookup close to O(log N) per query regardless of how
    the points are ordered along each curve.

    Parameters
    ----------
    reference_points : (N, 2) array of (x, y) pixel coordinates
    measured_points   : (M, 2) array of (x, y) pixel coordinates
    px_per_mm         : calibrated scale factor from step 1

    Returns
    -------
    dist_mm : (N,) array, deviation in millimeters for each reference point
    """
    tree = cKDTree(measured_points)
    dist_px, _nearest_idx = tree.query(reference_points)
    return dist_px / px_per_mm


def summarize_deviation(dist_mm):
    return {
        "mean_mm": float(dist_mm.mean()),
        "rmse_mm": float(np.sqrt((dist_mm ** 2).mean())),
        "p95_mm": float(np.percentile(dist_mm, 95)),
        "max_mm": float(dist_mm.max()),
    }


# ---------------------------------------------------------------------------
# Step 6 (core algorithm): peak selection via non-max suppression
# ---------------------------------------------------------------------------

def select_deviation_peaks(points, dist_mm, min_spacing_mm, px_per_mm,
                            exclude_box=None, top_k=2):
    """Pick the top-K worst-deviation points for callout annotation, while
    enforcing a minimum spacing between picks (greedy non-max suppression).

    Without NMS, the top-K points by raw distance tend to cluster on the
    same local error region (many adjacent skeleton pixels all have a
    similarly large distance), producing redundant callouts. This greedily
    accepts the globally worst point first, then only accepts subsequent
    candidates (in descending distance order) that are at least
    `min_spacing_mm` away from every point already accepted.

    Parameters
    ----------
    points          : (N, 2) reference-curve points, same order as dist_mm
    dist_mm         : (N,) per-point deviation from nearest_neighbor_deviation
    min_spacing_mm  : minimum allowed distance between two selected peaks
    px_per_mm       : calibrated scale factor (to convert min_spacing_mm to px)
    exclude_box     : optional (x0, y0, x1, y1) region to skip (e.g. to avoid
                       a known-noisy area from being reported)
    top_k           : number of peaks to return

    Returns
    -------
    list of indices into `points` / `dist_mm`, ranked worst-first
    """
    min_spacing_px = min_spacing_mm * px_per_mm
    order = np.argsort(-dist_mm)  # descending: worst deviation first
    picked = []

    for i in order:
        p = points[i]
        if exclude_box is not None:
            x0, y0, x1, y1 = exclude_box
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                continue
        if all(np.linalg.norm(p - points[j]) >= min_spacing_px for j in picked):
            picked.append(i)
        if len(picked) >= top_k:
            break

    return picked


# ---------------------------------------------------------------------------
# End-to-end example
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    IMAGE_PATH = "noder.png"

    # --- Step 1: scale calibration ---
    # Measured directly from the checkerboard target in the photo: mean
    # pixel width of a 10mm block and of a 5mm block (see README for how
    # these were measured).
    PX_PER_MM = calibrate_scale_from_checkerboard(
        large_block_px=38.25, small_block_px=19.33,
        large_block_mm=10.0, small_block_mm=5.0,
    )

    bgr = cv2.imread(IMAGE_PATH)

    # region occupied by the checkerboard target itself -- must be excluded
    # from curve extraction so its black/white squares aren't mistaken for
    # drawn ink
    CALIBRATION_TARGET_BOX = (515, 935, 700, 1015)

    # --- Steps 2-4: extract both curves ---
    # NOTE: thresholds match step6_noder.py exactly (strict "<"/">" at the
    # low/high value-saturation boundary, not "<="/">="), so this script
    # reproduces the report figure's numbers precisely. Max deviation is a
    # single-point statistic, so even a handful of boundary pixels differing
    # can shift it by ~0.3-0.5mm -- see README for why mean/RMSE are the
    # more robust summary numbers.
    reference_hsv_range = {"h": (0, 179), "s": (0, 59), "v": (0, 129)}   # dark/gray traced line
    robot_hsv_range = {"h": [(0, 12), (165, 179)], "s": (41, 255), "v": (61, 255)}  # red/pink traced line

    reference_mask = extract_curve_mask(bgr, reference_hsv_range, [CALIBRATION_TARGET_BOX])
    robot_mask = extract_curve_mask(bgr, robot_hsv_range, [CALIBRATION_TARGET_BOX])

    reference_skeleton = prune_skeleton_spurs(clean_and_skeletonize(reference_mask))
    robot_skeleton = prune_skeleton_spurs(clean_and_skeletonize(robot_mask))

    reference_points = skeleton_to_points(reference_skeleton)
    robot_points = skeleton_to_points(robot_skeleton)

    # --- Step 5: nearest-neighbor deviation (core algorithm) ---
    dist_mm = nearest_neighbor_deviation(reference_points, robot_points, PX_PER_MM)
    stats = summarize_deviation(dist_mm)
    print(f"Mean deviation: {stats['mean_mm']:.2f} mm")
    print(f"Max deviation:  {stats['max_mm']:.2f} mm")

    # --- Step 6: peak selection for callouts (core algorithm) ---
    peak_indices = select_deviation_peaks(
        reference_points, dist_mm,
        min_spacing_mm=15.0, px_per_mm=PX_PER_MM,
        exclude_box=(300, 460, 430, 560),  # example: skip a known-noisy area
        top_k=2,
    )
    print("Worst-deviation points:")
    for i in peak_indices:
        x, y = reference_points[i]
        print(f"  ({x:.0f}, {y:.0f}) -> {dist_mm[i]:.1f} mm")
