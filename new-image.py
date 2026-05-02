"""
final_plotter_ultimate.py — הגרסה הסופית לרובוט ציור
כולל מצב מיוחד (5) לציורים מוכנים מ-AI.
"""

import cv2
import numpy as np
from skimage.morphology import thin
import matplotlib.pyplot as plt
import sys
import os

# ==================== הגדרות פיזיות ====================
IMAGE_MAX_DIM = 800  
BASE_WIDTH    = 0.59        
H_BOARD       = 0.6         
H_ROBOT       = 0.19        
X_LIM         = [0.06, 0.53]   
Y_LIM         = [0.13, 0.41]   
Y_MIN_DRAW    = 0.10        

# ==================== הגדרות פרופילים משודרגות ====================
PRESETS = {
    ord('1'): {"mode": "thresh", "blur": 3,  "block": 11, "min_area": 5, "desc": "1. Sharp (Logos/Text)"},
    ord('2'): {"mode": "thresh", "blur": 7,  "block": 21, "min_area": 10, "desc": "2. Balanced (Simple Photos)"},
    ord('3'): {"mode": "dog",    "blur1": 1.0, "blur2": 3.0, "thresh": 0.98, "desc": "3. PRO Portrait (Only for Real Photos!)"}, 
    ord('4'): {"mode": "thresh", "blur": 21, "block": 91, "min_area": 80, "desc": "4. Abstract Sketch"},
    ord('5'): {"mode": "lineart", "min_area": 10, "desc": "5. AI / Line Art (PERFECT for AI images)"} 
}

# ==================== פונקציות עיבוד תמונה ====================

def process_image_lineart(img_orig, settings):
    """
    אלגוריתם חדש במיוחד לציורים מוכנים מ-AI.
    פשוט, חד, בלי ניחושים.
    """
    if len(img_orig.shape) == 3:
        gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_orig.copy()

    # שיפור ניגודיות קל לוודא שהשחור הוא שחור והלבן הוא לבן
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)

    # ניקוי רעשים מינימלי (נקודות בודדות שה-AI אולי השאיר)
    kernel = np.ones((2,2), np.uint8)
    # סגירת רווחים קטנטנים בקו
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    return binary

def process_image_dog(img_orig, settings):
    """Difference of Gaussians - לפורטרטים מצולמים בלבד"""
    if len(img_orig.shape) == 3:
        gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_orig.copy()

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    img_float = gray.astype(np.float32) / 255.0

    g1 = cv2.GaussianBlur(img_float, (0,0), settings["blur1"])
    g2 = cv2.GaussianBlur(img_float, (0,0), settings["blur2"])
    dog = g1 - g2
    dog_norm = (dog * 255).astype(np.uint8)
    thresh_val = 255 * 0.05 
    _, binary = cv2.threshold(dog_norm, thresh_val, 255, cv2.THRESH_BINARY)
    kernel = np.ones((2,2), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    return binary

def process_image_standard(img_orig, settings):
    """Adaptive Threshold - ללוגואים ותמונות רגילות"""
    if len(img_orig.shape) == 3:
        gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_orig.copy()
    
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)

    b = settings["blur"]
    gray = cv2.bilateralFilter(gray, 9, b*10, b*10)

    blk = settings["block"]
    if blk % 2 == 0: blk += 1 
    binary = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blk, 
        10 
    )
    return binary

def process_image_router(img_orig, settings):
    # נתב חכם
    if settings["mode"] == "dog":
        binary = process_image_dog(img_orig, settings)
    elif settings["mode"] == "lineart":
        binary = process_image_lineart(img_orig, settings)
    else:
        binary = process_image_standard(img_orig, settings)

    # שלב סופי משותף: ניקוי והפיכה לקו דק
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    cleaned = np.zeros_like(binary)
    min_a = settings.get("min_area", 20) 
    
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= min_a:
            cleaned[labels == i] = 255

    skeleton = thin(cleaned > 0)
    return skeleton

# ==================== תצוגה מקדימה ====================

def create_preview_grid(img_orig):
    h, w = img_orig.shape[:2]
    preview_dim = 400
    scale = preview_dim / max(h, w)
    img_small = cv2.resize(img_orig, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    
    previews = []
    # מציג את 4 האפשרויות הרלוונטיות ביותר (דילגתי על 4 כדי להכניס את 5)
    keys_to_show = [ord('1'), ord('2'), ord('3'), ord('5')]
    
    for key in keys_to_show:
        settings = PRESETS[key]
        skeleton = process_image_router(img_small, settings)
        vis = (skeleton * 255).astype(np.uint8)
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        
        color = (0, 255, 0)
        if key == ord('5'): color = (0, 165, 255) # כתום להדגשה לחדש
            
        cv2.putText(vis, settings["desc"], (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.rectangle(vis, (0,0), (vis.shape[1], vis.shape[0]), (50,50,50), 2)
        previews.append(vis)

    top_row = np.hstack((previews[0], previews[1]))
    bot_row = np.hstack((previews[2], previews[3]))
    grid = np.vstack((top_row, bot_row))
    return grid

# ==================== פונקציות מסלול ====================

def simplify_segment(segment, epsilon=0.0005):
    if len(segment) < 3: return segment
    curve = segment.reshape(-1, 1, 2).astype(np.float32)
    simplified = cv2.approxPolyDP(curve, epsilon, closed=False)
    return simplified.reshape(-1, 2).astype(np.float64)

def extract_segments(skeleton, min_length=15):
    h, w = skeleton.shape
    remaining = skeleton.copy()
    segments_pix = []
    
    def get_neighbors(r, c):
        nbs = []
        for dr in [-1,0,1]:
            for dc in [-1,0,1]:
                if dr==0 and dc==0: continue
                nr, nc = r+dr, c+dc
                if 0<=nr<h and 0<=nc<w and remaining[nr,nc]:
                    nbs.append((nr,nc))
        return nbs

    while np.any(remaining):
        ys, xs = np.where(remaining)
        start = (ys[0], xs[0])
        for i in range(len(ys)):
            if len(get_neighbors(ys[i], xs[i])) == 1:
                start = (ys[i], xs[i])
                break
        
        path = [start]
        remaining[start] = False
        curr = start
        
        while True:
            nbs = get_neighbors(curr[0], curr[1])
            if not nbs: break
            best = nbs[0]
            path.append(best)
            remaining[best] = False
            curr = best
            
        if len(path) > min_length: 
            segments_pix.append(np.array(path, dtype=float))
            
    return segments_pix

def optimize_route(segments, start_xy):
    if len(segments) <= 1: return segments
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
                best_dist = d_start; best_idx = idx; best_flip = False
            if d_end < best_dist:
                best_dist = d_end; best_idx = idx; best_flip = True
        
        seg = segments[best_idx]
        if best_flip: seg = seg[::-1].copy()
        ordered.append(seg)
        current = seg[-1]
        remaining.remove(best_idx)
    return ordered

def plot_preview(segments_pix, segments_m):
    plt.figure(figsize=(14, 7))
    
    plt.subplot(1, 2, 1)
    for seg in segments_pix:
        plt.plot(seg[:, 1], seg[:, 0], linewidth=1)
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title(f"Detected Paths (Pixels) - {len(segments_pix)} Segments")
    plt.grid(True, alpha=0.3)
    
    plt.subplot(1, 2, 2)
    y_min_safe = max(Y_LIM[0], Y_MIN_DRAW)
    rect = plt.Rectangle((X_LIM[0], y_min_safe), X_LIM[1]-X_LIM[0], Y_LIM[1]-y_min_safe, 
                         fill=False, edgecolor='r', linestyle='--', linewidth=2, label='Safe Zone')
    plt.gca().add_patch(rect)
    plt.plot([0, BASE_WIDTH], [0, 0], 'ks', markersize=10, label='Motors')
    
    for seg in segments_m:
        plt.plot(seg[:, 0], seg[:, 1], linewidth=1)
        
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.xlim(-0.05, BASE_WIDTH+0.05)
    plt.ylim(H_BOARD+0.05, -0.05)
    plt.title("Robot Path (Meters)")
    plt.legend(loc='lower right')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print("📊 Opening preview window...")
    plt.show()

# ==================== Main ====================

def main():
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = input("Enter image filename: ").strip()
    
    if not os.path.exists(image_path):
        print("Error: File not found!")
        return

    print("Loading image...")
    img_orig = cv2.imread(image_path)
    
    h, w = img_orig.shape[:2]
    if max(h, w) > IMAGE_MAX_DIM:
        scale = IMAGE_MAX_DIM / max(h, w)
        img_orig = cv2.resize(img_orig, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    # --- שלב 1: בחירת סגנון ---
    grid = create_preview_grid(img_orig)
    window_name = "1. Select Style (Press 1, 2, 3 or 5)"
    cv2.namedWindow(window_name)
    cv2.imshow(window_name, grid)
    
    print("\n--- STEP 1: SELECT STYLE ---")
    print("  TIP: Use '5' for AI Generated Images!")
    
    selected_settings = None
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in PRESETS:
            selected_settings = PRESETS[key]
            print(f" -> Selected: {selected_settings['desc']}")
            break
        elif key == ord('q'): return

    # --- שלב 2: בחירת גודל ---
    size_screen = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.putText(size_screen, "STEP 2: CHOOSE SIZE", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(size_screen, "[S] Small  (0.5)", (50, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 255, 100), 2)
    cv2.putText(size_screen, "[M] Medium (0.8)", (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 255, 255), 2)
    cv2.putText(size_screen, "[L] Large  (1.0)", (50, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 255), 2)
    
    cv2.destroyWindow(window_name)
    window_size = "2. Select Size (S/M/L)"
    cv2.imshow(window_size, size_screen)
    
    print("\n--- STEP 2: SELECT SIZE ---")
    fraction = 0.8
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('s') or key == ord('S'): fraction = 0.5; break
        elif key == ord('m') or key == ord('M'): fraction = 0.8; break
        elif key == ord('l') or key == ord('L'): fraction = 1.0; break
        elif key == ord('q'): return

    cv2.destroyAllWindows()

    # --- שלב 3: עיבוד סופי ---
    print("\nProcessing full resolution path...")
    skeleton_final = process_image_router(img_orig, selected_settings)
    
    # חילוץ וסינון
    min_len = selected_settings.get("min_area", 15)
    segments_pix = extract_segments(skeleton_final, min_length=min_len)
    
    if not segments_pix:
        print("Error: No paths found!")
        return

    # המרה למטרים
    all_pts = np.vstack(segments_pix)
    x_lim_pix = [all_pts[:,1].min(), all_pts[:,1].max()]
    y_lim_pix = [all_pts[:,0].min(), all_pts[:,0].max()]
    
    pix_w = x_lim_pix[1] - x_lim_pix[0]
    pix_h = y_lim_pix[1] - y_lim_pix[0]
    x_range = (X_LIM[1] - X_LIM[0]) * fraction
    y_range = (Y_LIM[1] - max(Y_LIM[0], Y_MIN_DRAW)) * fraction
    scale = min(x_range/pix_w, y_range/pix_h)
    
    x_center = (X_LIM[0] + X_LIM[1]) / 2
    y_center = (max(Y_LIM[0], Y_MIN_DRAW) + Y_LIM[1]) / 2
    actual_w = pix_w * scale
    actual_h = pix_h * scale
    
    segments_m = []
    for seg in segments_pix:
        rows, cols = seg[:,0], seg[:,1]
        nx = (cols - x_lim_pix[0]) / pix_w
        ny = (rows - y_lim_pix[0]) / pix_h
        mx = x_center - actual_w/2 + nx*actual_w
        my = y_center - actual_h/2 + ny*actual_h
        mx = np.clip(mx, X_LIM[0], X_LIM[1])
        my = np.clip(my, max(Y_LIM[0], Y_MIN_DRAW), Y_LIM[1])
        
        seg_m = np.column_stack([mx, my])
        seg_m = simplify_segment(seg_m)
        segments_m.append(seg_m)

    start_xy = [(X_LIM[0] + X_LIM[1]) / 2, max(Y_LIM[0], Y_MIN_DRAW)]
    segments_m = optimize_route(segments_m, start_xy)

    # שמירה
    filename = "path.txt"
    with open(filename, 'w') as f:
        f.write(f"# Style: {selected_settings['desc']}, Size: {fraction}\n\n")
        for seg in segments_m:
            f.write("PEN_UP\n")
            f.write(f"G:{seg[0][0]:.5f},{seg[0][1]:.5f}\n")
            f.write("PEN_DOWN\n")
            for pt in seg[1:]:
                f.write(f"G:{pt[0]:.5f},{pt[1]:.5f}\n")
            f.write("PEN_UP\n")
            
    print(f"✅ DONE! Saved to '{filename}'.")
    plot_preview(segments_pix, segments_m)

if __name__ == "__main__":
    main()