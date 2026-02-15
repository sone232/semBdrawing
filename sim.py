import matplotlib.pyplot as plt
import numpy as np
import image_processing as ip # מייבא את הלוגיקה שלך
import os

def run_enhanced_simulation(image_path, speed_factor=2.0):
    if not os.path.exists(image_path):
        print(f"Error: {image_path} not found!")
        return

    print("--- מכין את התכנון לסימולציה ---")
    
    # 1. עיבוד התמונה (בדיוק כמו ברובוט האמיתי)
    skeleton, _ = ip.process_image(image_path)
    segments_pix = ip.extract_segments(skeleton)
    segments_pix = ip.connect_segments(segments_pix, ip.MERGE_GAP_PIX)
    x_lim_pix, y_lim_pix = ip.get_pixel_limits(segments_pix)
    segments_m = ip.pixels_to_meters(segments_pix, x_lim_pix, y_lim_pix)
    
    # פישוט ואופטימיזציה
    segments_m = [ip.resample_segment(seg, ip.RESAMPLE_DIST) for seg in segments_m]
    segments_m = [ip.simplify_segment(seg, ip.SIMPLIFY_EPSILON) for seg in segments_m]
    
    start_xy = [(ip.X_LIM[0] + ip.X_LIM[1]) / 2, max(ip.Y_LIM[0], ip.Y_MIN_DRAW)]
    segments_m = ip.optimize_route(segments_m, start_xy)

    # 2. הכנת הגרף
    fig, ax = plt.subplots(figsize=(10, 8))
    y_min_safe = max(ip.Y_LIM[0], ip.Y_MIN_DRAW)
    
    # ציור גבולות הציור הבטוחים
    safe_rect = plt.Rectangle((ip.X_LIM[0], y_min_safe), 
                               ip.X_LIM[1] - ip.X_LIM[0], 
                               ip.Y_LIM[1] - y_min_safe, 
                               fill=False, edgecolor='red', linestyle='--', label='Safe Zone')
    ax.add_patch(safe_rect)
    ax.plot([0, ip.BASE_WIDTH], [0, 0], 'ks', markersize=10, label='Motors')

    # --- הוספת שלב ה"תכנון" ---
    # מצייר את כל הציור מראש באפור בהיר מאוד
    print("מצייר את מפת התכנון...")
    for seg in segments_m:
        ax.plot(seg[:, 0], seg[:, 1], color='#eeeeee', linewidth=1, zorder=1)

    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.set_title(f"סימולציה: {image_path} (ציור מתקדם ואיטי)")
    ax.grid(True, alpha=0.2)

    # 3. אנימציה איטית ומתקדמת
    current_pos = np.array(start_xy)
    
    print("מתחיל לצבוע בשחור...")
    for i, seg in enumerate(segments_m):
        # תנועה באוויר (Pen Up) - אפור מקווקו עדין
        ax.plot([current_pos[0], seg[0, 0]], [current_pos[1], seg[0, 1]], 
                color='gray', linestyle=':', alpha=0.1, linewidth=0.5)
        
        # ציור נקודה-נקודה ליצירת אפקט התקדמות איטי
        # ככל ש-speed_factor גדול יותר, זה יהיה איטי יותר
        for j in range(1, len(seg)):
            ax.plot(seg[j-1:j+1, 0], seg[j-1:j+1, 1], color='black', linewidth=1.5, zorder=2)
            
            # הפסקה קטנה בין כל "משיכת מכחול"
            plt.pause(0.001 * speed_factor)
            
        current_pos = seg[-1]
        
        if i % 10 == 0:
            print(f"התקדמות: {i}/{len(segments_m)} סגמנטים נצבעו")

    print("הסימולציה הסתיימה!")
    plt.show()

if __name__ == "__main__":
    # הגדר speed_factor גבוה יותר בשביל עוד יותר איטי (למשל 5.0)
    run_enhanced_simulation("mona.png", speed_factor=3.0)