# lidar_test_hambot.py
# -------------------------------------------------------------
# Simple test script to read and print HamBot LIDAR distances
# from multiple directions (front, left, right, back).
# -------------------------------------------------------------

import time
from robot_systems.robot import HamBot


def safe_min(values):
    vals = [v for v in values if v > 0.0]
    return min(vals) * 0.001 if vals else float("inf")  # mm → m


def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(1.0)
    print("[TEST] Starting LIDAR range test...")
    print("Press Ctrl+C to stop.\n")

    # Define sector slices (based on 360° LIDAR layout)
    FRONT = slice(175, 185)   # ~180° forward
    LEFT  = slice(90, 100)    # ~90° left
    RIGHT = slice(260, 270)   # ~270° right
    BACK  = slice(350, 360)   # ~0°/360° back

    try:
        while True:
            scan = bot.get_range_image()
            if scan == -1 or len(scan) < 360:
                print("[WARN] Invalid scan data.")
                time.sleep(0.1)
                continue

            front_d = safe_min(scan[FRONT])
            left_d  = safe_min(scan[LEFT])
            right_d = safe_min(scan[RIGHT])
            back_d  = safe_min(scan[BACK])

            print(f"Front: {front_d:.3f} m | Left: {left_d:.3f} m | "
                  f"Right: {right_d:.3f} m | Back: {back_d:.3f} m")

            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n[TEST] Stopped by user.")
    finally:
        bot.stop_motors()


if __name__ == "__main__":
    main()
