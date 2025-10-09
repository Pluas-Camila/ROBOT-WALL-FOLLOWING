# task1_forward_stop_hambot.py
# Lab 2 – Task 1 (HamBot): PID forward wall stop using LIDAR (front sector min)
# -------------------------------------------------------------------------------
# - Uses LIDAR.get_current_scan via HamBot.get_range_image()    (360 beams, deg: 0=back, 90=left, 180=front, 270=right)
# - Pure forward controller: PID sets equal wheel speeds (no steering)
# - If < 1.0 m, robot gently backs up to 1.0 m
# - Prints measured forward distance continuously

import time, math
from robot_systems.robot import HamBot

# ===== Robot geometry (for m/s ↔ RPM) =====
R_WHEEL = 0.045     # m
AXLE_L  = 0.184     # m

# ===== Safety limits (HamBot clamps to ±75 RPM) =====
RPM_MAX = 75.0
RPM_MIN = -75.0

# ===== Controller targets & loop timing =====
TARGET = 1.00       # m (desired front distance)
DEADBAND = 0.02     # ±2 cm
DT = 0.032          # s control period (match assignment)
FRONT_SPAN = 6      # use ±6° around 180

# ===== PID gains (sweep per assignment: 0.001, 0.01, 0.5, 1.0, 5.0, 10.0) =====
KP = 1.0
KI = 0.05
KD = 0.10

def mps_to_rpm(v_mps: float) -> float:
    """Linear wheel speed [m/s] -> RPM."""
    return (v_mps / (2.0 * math.pi * R_WHEEL)) * 60.0

def sector_min(ranges, center=180, half_span=FRONT_SPAN):
    """Min range in small sector around 'center' index (wrap 0..359)."""
    n = len(ranges)
    vals = []
    for k in range(-half_span, half_span + 1):
        v = float(ranges[(center + k) % n])
        if v > 0:  # ignore invalid zeros
            vals.append(v)
    return min(vals) if vals else 6.0

def saturate(val, lo, hi):
    return max(lo, min(hi, val))

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    print(f"\n[Task1/HamBot] Forward PID → target {TARGET:.2f} m  (dt={DT:.3f}s)")
    print(f"Gains: Kp={KP}, Ki={KI}, Kd={KD} | RPM limit ±{RPM_MAX:.0f}")

    # PID state
    i_acc = 0.0
    e_prev = 0.0
    t_prev = time.time()

    try:
        while True:
            t0 = time.time()
            # 1) Measure front distance (min of sector around 180°)
            ranges = bot.get_range_image()
            if ranges == -1:
                print("[WARN] LIDAR not enabled or no data; stopping.")
                bot.stop_motors()
                time.sleep(0.5)
                continue
            y = sector_min(ranges, center=180, half_span=FRONT_SPAN)

            # 2) Error e = r - y  (positive if too far; negative if too close)
            e = TARGET - y

            # 3) PID (discrete)
            dt = max(DT, time.time() - t_prev)
            i_acc += e * dt
            d = (e - e_prev) / dt if dt > 0 else 0.0
            u = KP * e + KI * i_acc + KD * d   # body forward speed command [m/s]

            # 4) Deadband & safety
            if abs(e) <= DEADBAND:
                u = 0.0

            # 5) Convert to wheel RPM (equal wheels -> straight)
            rpm = mps_to_rpm(u)
            rpm = saturate(rpm, RPM_MIN, RPM_MAX)

            # 6) Send commands (HamBot left is inverted inside the class setter)
            bot.set_left_motor_speed(rpm)
            bot.set_right_motor_speed(rpm)

            # 7) Print diagnostics (continuous)
            print(f"[Task1] front={y:5.2f} m | e={e:+6.3f} | v={u:+.3f} m/s | rpm={rpm:+6.1f}")

            # PID state update
            e_prev = e
            t_prev = time.time()

            # Loop pacing
            elapsed = time.time() - t0
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[Task1] Stopping…")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()
