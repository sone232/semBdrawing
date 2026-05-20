"""
image.py — Convert image to drawing path for V-Plotter robot
Generates path.txt compatible with newnoder.py

Usage:
    python image.py <image_file> [fraction]
    python image.py starbucks.jpg
    python image.py starbucks.jpg 0.7
"""

import cv2
import numpy as np
from skimage.morphology import thin
import matplotlib.pyplot as plt
import sys
import os

# ============================================================
# Physical Constants
# ============================================================
BASE_WIDTH    = 0.59        # Distance between motors (meters)
H_BOARD       = 0.6         # Whiteboard height (meters)
H_ROBOT       = 0.19        # Robot body height (meters)
X_LIM         = [0.06, 0.53]   # Safe X range (meters)
Y_LIM         = [0.13, 0.41]   # Safe Y range (meters)
Y_MIN_DRAW    = 0.10        # Don't draw above 10cm from top
FRACTION      = 0.9         # Use 50% of safe zone  // was 0.7
MERGE_GAP_PIX = 3           # Max pixel gap to merge segments
MIN_SEGMENT_LEN = 3         # Discard segments shorter than this
RESAMPLE_DIST    = 0.005     # היה 0.003 → 2mm
SIMPLIFY_EPSILON = 0.0002    # היה 0.0004 → פי 2 יותר מדויק
IMAGE_MAX_DIM    = 600       # היה 500

def simplify_segment(segment, epsilon=SIMPLIFY_EPSILON):
    """
    Douglas-Peucker simplification — remove unnecessary points
    while keeping the shape. Like zooming out on details.
    """
    if len(segment) < 3:
        return segment
    
    # cv2.approxPolyDP needs specific format
    curve = segment.reshape(-1, 1, 2).astype(np.float32)
    simplified = cv2.approxPolyDP(curve, epsilon, closed=False)
    return simplified.reshape(-1, 2).astype(np.float64)

# ============================================================
# Step 1: Image Processing → Skeleton
# ============================================================
def process_image(image_path):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Cannot load image: {image_path}")

    # --- Resize if too large ---
    h, w = img.shape[:2]
    if max(h, w) > IMAGE_MAX_DIM:
        scale = IMAGE_MAX_DIM / max(h, w)
        img = cv2.resize(img, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
        print(f"      → Resized: {w}x{h} → {img.shape[1]}x{img.shape[0]}")

    # ... שאר הפונקציה נשארת אותו דבר ...
    # --- Grayscale ---
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # --- Adaptive threshold (dark foreground → white) ---
    #     Matches MATLAB: imbinarize(img,'adaptive','ForegroundPolarity','dark')
    block_size = max(15, (min(gray.shape) // 20) | 1)  # odd number
    binary = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        block_size, 10
    )

    # --- Clean: remove isolated pixels (no neighbors) ---
    #     Matches MATLAB: bwmorph(img,'clean')
    kernel = np.ones((3, 3), np.uint8)
    kernel[1, 1] = 0
    neighbor_count = cv2.filter2D(
        (binary > 0).astype(np.uint8), -1, kernel
    )
    cleaned = binary.copy()
    cleaned[(binary > 0) & (neighbor_count == 0)] = 0

    # --- Skeleton: thin to 1px lines ---
    #     Matches MATLAB: bwmorph(img,'thin',inf)
    skeleton = thin(cleaned > 0)

    return skeleton, img


# ============================================================
# Step 2: Extract Ordered Segments from Skeleton
# ============================================================
def extract_segments(skeleton):
    """
    Walk the skeleton image and extract ordered path segments.
    Each segment is an Nx2 array of (row, col) pixel coordinates.
    At junctions, paths split into separate segments.
    """
    h, w = skeleton.shape
    remaining = skeleton.copy().astype(bool)
    segments = []

    def count_neighbors(r, c):
        count = 0
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and remaining[nr, nc]:
                    count += 1
        return count

    def find_start():
        """Find a starting pixel — prefer endpoints (1 neighbor)."""
        ys, xs = np.where(remaining)
        if len(ys) == 0:
            return None
        # Look for endpoints first
        for i in range(len(ys)):
            if count_neighbors(ys[i], xs[i]) == 1:
                return (ys[i], xs[i])
        # No endpoint → closed loop, take any pixel
        return (ys[0], xs[0])

    def walk(start):
        """Walk along connected pixels, preferring straight direction."""
        path = [start]
        remaining[start[0], start[1]] = False
        current = start
        prev = None

        while True:
            r, c = current
            best = None
            best_score = -1

            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and remaining[nr, nc]:
                        # Prefer continuing in same direction
                        score = 0
                        if prev is not None:
                            pr, pc = prev
                            if dr == (r - pr) and dc == (c - pc):
                                score = 2   # same direction
                            elif dr == (r - pr) or dc == (c - pc):
                                score = 1   # partially same
                        if best is None or score > best_score:
                            best = (nr, nc)
                            best_score = score

            if best is None:
                break

            path.append(best)
            remaining[best[0], best[1]] = False
            prev = current
            current = best

        return path

    # --- Extract all segments ---
    while True:
        start = find_start()
        if start is None:
            break
        path = walk(start)
        if len(path) >= MIN_SEGMENT_LEN:
            segments.append(np.array(path, dtype=float))

    return segments


# ============================================================
# Step 3: Merge Segments with Nearby Endpoints
# ============================================================
def connect_segments(segments, max_gap=MERGE_GAP_PIX):
    """
    Merge segments whose endpoints are within max_gap pixels.
    Matches MATLAB: connectSegments()
    """
    if len(segments) < 2:
        return segments

    merged = True
    while merged:
        merged = False
        i = 0
        while i < len(segments):
            best_j = None
            best_dist = max_gap + 1
            best_flip_i = False
            best_flip_j = False

            j = i + 1
            while j < len(segments):
                si, sj = segments[i], segments[j]

                # Check all 4 endpoint combinations
                combos = [
                    (np.linalg.norm(si[-1] - sj[0]),  False, False),  # end→start
                    (np.linalg.norm(si[-1] - sj[-1]), False, True),   # end→end
                    (np.linalg.norm(si[0]  - sj[0]),  True,  False),  # start→start
                    (np.linalg.norm(si[0]  - sj[-1]), True,  True),   # start→end
                ]

                for dist, fi, fj in combos:
                    if dist < best_dist:
                        best_dist = dist
                        best_j = j
                        best_flip_i = fi
                        best_flip_j = fj
                j += 1

            if best_j is not None and best_dist <= max_gap:
                si = segments[i][::-1] if best_flip_i else segments[i]
                sj = segments[best_j][::-1] if best_flip_j else segments[best_j]
                segments[i] = np.vstack([si, sj])
                segments.pop(best_j)
                merged = True
                # Don't increment i — check again with merged segment
            else:
                i += 1

    return segments


# ============================================================
# Step 4: Pixel Coordinates → Meters
# ============================================================
def get_pixel_limits(segments):
    """Get bounding box of all segments in pixel coordinates."""
    all_pts = np.vstack(segments)
    x_lim_pix = [float(all_pts[:, 1].min()), float(all_pts[:, 1].max())]
    y_lim_pix = [float(all_pts[:, 0].min()), float(all_pts[:, 0].max())]
    return x_lim_pix, y_lim_pix


def pixels_to_meters(segments, x_lim_pix, y_lim_pix, fraction=FRACTION):
    """
    Map pixel coordinates to meters within safe drawing zone.
    Preserves aspect ratio. Centers drawing in the safe zone.
    Matches MATLAB: transformPixelsToMeters()
    
    Output: list of Nx2 arrays with (x_meters, y_meters)
    """
    # Effective safe zone (enforce Y_MIN_DRAW)
    y_min = max(Y_LIM[0], Y_MIN_DRAW)
    y_max = Y_LIM[1]
    x_min, x_max = X_LIM

    # Available drawing area (scaled by fraction)
    x_range = (x_max - x_min) * fraction
    y_range = (y_max - y_min) * fraction
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2

    # Pixel dimensions
    pix_w = x_lim_pix[1] - x_lim_pix[0]
    pix_h = y_lim_pix[1] - y_lim_pix[0]
    if pix_w == 0 or pix_h == 0:
        raise ValueError("Image has zero width or height!")

    # Uniform scale — preserve aspect ratio
    scale = min(x_range / pix_w, y_range / pix_h)
    actual_w = pix_w * scale
    actual_h = pix_h * scale

    segments_m = []
    for seg in segments:
        rows = seg[:, 0]   # pixel Y
        cols = seg[:, 1]   # pixel X

        # Normalize to 0..1
        nx = (cols - x_lim_pix[0]) / pix_w
        ny = (rows - y_lim_pix[0]) / pix_h

        # Map to meters (centered)
        mx = x_center - actual_w / 2 + nx * actual_w
        my = y_center - actual_h / 2 + ny * actual_h

        # Clip to safe zone (safety net)
        mx = np.clip(mx, x_min, x_max)
        my = np.clip(my, y_min, y_max)

        segments_m.append(np.column_stack([mx, my]))

    return segments_m


# ============================================================
# Step 5: Resample Segments
# ============================================================
def resample_segment(segment, max_dist=RESAMPLE_DIST):
    """
    Resample so no two consecutive points are further than max_dist apart.
    Matches MATLAB: reduceSegment()
    """
    if len(segment) < 2:
        return segment

    resampled = [segment[0]]

    for i in range(1, len(segment)):
        p0 = resampled[-1]
        p1 = segment[i]
        dist = np.linalg.norm(p1 - p0)

        if dist > max_dist:
            n_steps = int(np.ceil(dist / max_dist))
            for j in range(1, n_steps + 1):
                t = j / n_steps
                resampled.append(p0 + t * (p1 - p0))
        else:
            resampled.append(p1)

    return np.array(resampled)


# ============================================================
# Step 6: Route Optimization
# ============================================================
def optimize_route(segments, start_xy=None):
    """
    Reorder segments using nearest-neighbor heuristic.
    Also flips segments if the end is closer than the start.
    Matches MATLAB: routeOptimization()
    """
    if len(segments) <= 1:
        return segments

    if start_xy is None:
        current = segments[0][0].copy()
    else:
        current = np.array(start_xy, dtype=float)

    remaining = list(range(len(segments)))
    ordered = []

    while remaining:
        best_idx = None
        best_dist = float('inf')
        best_flip = False

        for idx in remaining:
            seg = segments[idx]
            d_start = np.linalg.norm(current - seg[0])
            d_end   = np.linalg.norm(current - seg[-1])

            if d_start < best_dist:
                best_dist = d_start
                best_idx = idx
                best_flip = False
            if d_end < best_dist:
                best_dist = d_end
                best_idx = idx
                best_flip = True

        seg = segments[best_idx]
        if best_flip:
            seg = seg[::-1].copy()

        ordered.append(seg)
        current = seg[-1].copy()
        remaining.remove(best_idx)

    return ordered


# ============================================================
# Step 7: Visualization
# ============================================================
def plot_pixels(segments, title="Pixel Segments"):
    """Plot segments in pixel coordinates, each in a different color."""
    plt.figure(figsize=(10, 8))
    for seg in segments:
        plt.plot(seg[:, 1], seg[:, 0], linewidth=1)
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title(f"{title}  ({len(segments)} segments)")
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.grid(True, alpha=0.2)
    plt.tight_layout()
    plt.show()


def plot_meters(segments, title="Drawing Preview"):
    """Plot segments in meters with safe zone rectangle."""
    y_min = max(Y_LIM[0], Y_MIN_DRAW)

    plt.figure(figsize=(10, 8))

    # Safe zone rectangle
    rect = plt.Rectangle(
        (X_LIM[0], y_min),
        X_LIM[1] - X_LIM[0],
        Y_LIM[1] - y_min,
        fill=False, edgecolor='red', linewidth=2,
        linestyle='--', label='Safe Zone'
    )
    plt.gca().add_patch(rect)

    # Motor positions
    plt.plot([0, BASE_WIDTH], [0, 0], 'ks', markersize=10, label='Motors')

    # Drawing segments
    total_pts = 0
    for seg in segments:
        plt.plot(seg[:, 0], seg[:, 1], linewidth=1)
        total_pts += len(seg)

    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.xlim(-0.02, BASE_WIDTH + 0.02)
    plt.ylim(H_BOARD + 0.02, -0.02)
    plt.title(f"{title}\n{len(segments)} segments, {total_pts} points")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.legend(loc='lower right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


# ============================================================
# Step 8: Write path.txt
# ============================================================
def write_path(segments, filename="path.txt"):
    """
    Write drawing commands to path.txt.
    Format compatible with newnoder.py:
      PEN_UP / PEN_DOWN / G:x,y
    """
    total_moves = 0

    with open(filename, 'w') as f:
        f.write(f"# Generated by image.py\n")
        f.write(f"# {len(segments)} segments\n")
        f.write(f"# Safe zone X: {X_LIM}  Y: {Y_LIM}\n\n")

        for i, seg in enumerate(segments):
            # --- Move to start (pen up) ---
            f.write("PEN_UP\n")
            f.write(f"G:{seg[0][0]:.5f},{seg[0][1]:.5f}\n")
            total_moves += 1

            # --- Draw segment (pen down) ---
            f.write("PEN_DOWN\n")
            for pt in seg[1:]:
                f.write(f"G:{pt[0]:.5f},{pt[1]:.5f}\n")
                total_moves += 1

        # --- Finish with pen up ---
        f.write("PEN_UP\n")

    return total_moves


# ============================================================
# Main
# ============================================================
def main():
    # --- Parse arguments ---
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = input("Image file path: ").strip()

    if not os.path.exists(image_path):
        print(f"Error: '{image_path}' not found!")
        sys.exit(1)

    fraction = FRACTION
    if len(sys.argv) > 2:
        try:
            fraction = float(sys.argv[2])
            print(f"  Using fraction = {fraction}")
        except ValueError:
            pass

    print(f"\n{'='*50}")
    print(f"  image.py — Image to Drawing Path")
    print(f"{'='*50}")
    print(f"  Image:     {image_path}")
    print(f"  Fraction:  {fraction}")
    print(f"  Safe X:    {X_LIM[0]:.2f} — {X_LIM[1]:.2f} m")
    print(f"  Safe Y:    {max(Y_LIM[0], Y_MIN_DRAW):.2f} — {Y_LIM[1]:.2f} m")
    print(f"{'='*50}\n")

    # --- Step 1: Process image ---
    print("[1/7] Processing image...")
    skeleton, original = process_image(image_path)
    n_pixels = int(np.sum(skeleton))
    print(f"      → Skeleton: {n_pixels} pixels")

    # --- Step 2: Extract segments ---
    print("[2/7] Extracting segments...")
    segments_pix = extract_segments(skeleton)
    print(f"      → Found {len(segments_pix)} segments")

    if not segments_pix:
        print("\nError: No segments found! Try a different image.")
        sys.exit(1)

    # --- Step 3: Merge nearby endpoints ---
    print("[3/7] Merging nearby endpoints...")
    n_before = len(segments_pix)
    segments_pix = connect_segments(segments_pix, MERGE_GAP_PIX)
    print(f"      → {n_before} → {len(segments_pix)} segments")

    # Show pixel preview
    print("\n      📊 Pixel preview (close window to continue)...")
    plot_pixels(segments_pix, f"After Merge — {image_path}")

    # --- Step 4: Convert to meters ---
    print("[4/7] Converting pixels → meters...")
    x_lim_pix, y_lim_pix = get_pixel_limits(segments_pix)
    print(f"      → Pixel X range: {x_lim_pix[0]:.0f} — {x_lim_pix[1]:.0f}")
    print(f"      → Pixel Y range: {y_lim_pix[0]:.0f} — {y_lim_pix[1]:.0f}")
    segments_m = pixels_to_meters(
        segments_pix, x_lim_pix, y_lim_pix, fraction
    )

    # --- Step 5: Resample ---
    print(f"[5/7] Resampling (max {RESAMPLE_DIST*1000:.1f}mm)...")
    segments_m = [resample_segment(seg, RESAMPLE_DIST) for seg in segments_m]
    total_pts = sum(len(s) for s in segments_m)
    print(f"      → {total_pts} total points")
   # --- Step 5.5: Simplify ---
    print("[5.5] Simplifying paths...")
    segments_m = [simplify_segment(seg, SIMPLIFY_EPSILON) for seg in segments_m]
    segments_m = [seg for seg in segments_m if len(seg) >= 2]  # remove tiny
    total_pts = sum(len(s) for s in segments_m)
    print(f"      → {total_pts} total points after simplification")
    # --- Step 6: Optimize route ---
    print("[6/7] Optimizing route...")
    start_xy = [(X_LIM[0] + X_LIM[1]) / 2, max(Y_LIM[0], Y_MIN_DRAW)]
    segments_m = optimize_route(segments_m, start_xy)
    print(f"      → Reordered {len(segments_m)} segments")

    # --- Step 7: Write path.txt FIRST (before previews!) ---
    output = "path.txt"
    print(f"[7/7] Writing {output}...")
    n_moves = write_path(segments_m, output)
    print(f"      → {len(segments_m)} segments, {n_moves} moves")
    print(f"      → Saved to {output}")

    print(f"\n{'='*50}")
    print(f"  ✅ Done! Run newnoder.py to draw.")
    print(f"{'='*50}\n")

    # --- Show previews AFTER saving ---
    print("      📊 Pixel preview...")
    plot_pixels(segments_pix, f"After Merge — {image_path}")

    print("      📊 Drawing preview...")
    plot_meters(segments_m, f"Drawing Preview — {image_path}")

if __name__ == "__main__":
    main()