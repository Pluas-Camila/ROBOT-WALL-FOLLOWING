# wall_follow_hambot_fix_units.py
# -------------------------------------------------------------
# HamBot – Task 2 wall following w/ robust LIDAR unit normalization.
# - Normalizes each scan to meters (mm/cm/m)
# - Uses explicit sectors (0=back, 90=left, 180=front, 270=right)
# - P(+D) on side distance for steering; P on forward speed
# - Timed ~90° corner turn; wrap-search when wall is “gone”
# -------------------------------------------------------------
import math 
import time
from robot_systems.robot import HamBot

# FOLLOW_SIDE = "left"         # "left" or "right"
# DT = 0.02
# MAX_RPM = 35.0
# CRUISE_RPM = 25.0
# TARGET_SIDE = 0.30           # meters
# STOP_HARD = 0.25            # emergency stop distance (m)
# TURN_TRIGGER = 0.6          # start corner turn when front < this (m)
# WALL_GONE = 0.60             # treat as wall end if side > this (m)

# # Gains
# SIDE_KP = 1.2
# SIDE_KD = 0.25
# FWD_KP  = 1.0
# FWD_KD  = 0.05

# TURN_RPM = 30.0
# TURN_SEC = 0.6
# SEARCH_RPM = 25.0
# SEARCH_SEC = 1.20

# # Sector definitions (10°-ish windows)
# FRONT_S = slice(160, 200)   # ~180°
# LEFT_S  = slice( 90, 116)    # ~ 90°
# RIGHT_S = slice(244, 280)    # ~270°
# BACK_S  = slice(355, 360)    # ~  0°/360°

# ==================================================================
# ROTATION CONTROL (your rotate_in_place function cleaned up)
# ==================================================================

def rotate_in_place(robot, angle, Kp=1.5, tol=0.01, poll_dt=0.01):
    """
    Rotate the robot in place by the given 'angle' (radians).
    Uses proportional control with IMU heading feedback.
    Positive angle = counter-clockwise (left turn).
    """
    L = getattr(robot, 'axle length', 0.28)
    R=getattr(robot, 'wheel radius' , 0.45)
    w_max = 75.0  # wheel angular velocity limit (rad/s)

    # --- Helper functions ---
    def sat(u):
        """Clamp control command to saturation limits."""
        return max(-w_max, min(w_max, u))

    def wrap_to_pi(x):
        """Wrap any angle to (-pi, pi]."""
        return (x + math.pi) % (2 * math.pi) - math.pi

    # --- Track cumulative rotation from IMU ---
    left_cur_heading_prev, right_cur_heading_prev = robot.get_encoder_readings() # initial heading in [0, 2π)
    cum_rot = 0.0  # signed rotation since start

    while True:
        left_cur_heading, right_cur_heading = robot.get_encoder_readings()
        d_left_heading = left_cur_heading - left_cur_heading_prev
        d_right_heading = right_cur_heading - right_cur_heading_prev

        left_cur_heading_prev, right_cur_heading_prev = left_cur_heading, right_cur_heading

        d_heading = (R/L) * (d_right_heading - d_left_heading)
        cum_rot += d_heading

        # Proportional control on angle error
        e_theta = angle - cum_rot
        if abs(e_theta) <= tol:
            break

        # Wheel angular speed (saturated)
        w_cmd = sat(Kp * e_theta)

        # In-place turn: CCW if angle > 0
        robot.set_left_motor_speed(-w_cmd)
        robot.set_right_motor_speed(+w_cmd)
        time.sleep(poll_dt)

    # Stop both wheels
    robot.set_wheel_speeds(0.0, 0.0)


# ==================================================================
# PID CONTROLLER CLASS
# ==================================================================

class PID:
    """A standard PID controller."""

    def __init__(self, Kp, Ki, Kd, windup=None, out_min=None, out_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windup = windup
        self.out_min = out_min
        self.out_max = out_max
        self.integrator = 0.0
        self.prev_err = None

    def reset(self):
        self.integrator = 0.0
        self.prev_err = None

    def update(self, err, dt):
        """Compute the PID output for a given error and timestep."""
        if dt <= 0:
            return 0.0

        # Proportional term
        P = self.Kp * err

        # Integral term
        self.integrator += err * dt
        if self.windup is not None:
            self.integrator = max(-self.windup, min(self.windup, self.integrator))
        I = self.Ki * self.integrator

        # Derivative term
        D = 0.0
        if self.prev_err is not None:
            D = self.Kd * ((err - self.prev_err) / dt)
        self.prev_err = err

        # PID sum
        u = P + I + D

        # Clamp output
        if self.out_min is not None and self.out_max is not None:
            u = max(self.out_min, min(self.out_max, u))

        return u


# ==================================================================
# LIDAR HELPERS
# ==================================================================

def wrap_to_pi(x):
    """Wrap an angle to (-pi, pi]."""
    return (x + math.pi) % (2 * math.pi) - math.pi


def lidar_angle_distance(angles, distances, target_angle, window=5 * math.pi / 180):
    """Return median distance around a target LIDAR angle."""
    samples = [
        d for a, d in zip(angles, distances)
        if abs(wrap_to_pi(a - target_angle)) < window
        and d is not None and not math.isinf(d)
    ]
    if not samples:
        return None
    samples.sort()
    return samples[len(samples) // 2]


def lidar_front_min(angles, distances, window=30 * math.pi / 180):
    """Return the minimum distance detected in front of the robot."""
    valid = [
        d for a, d in zip(angles, distances)
        if abs(wrap_to_pi(a)) < window and d is not None and not math.isinf(d)
    ]
    return min(valid) if valid else None


# ==================================================================
# MOTION HELPER
# ==================================================================

def wheel_speeds_from_v_omega(v, omega, wheel_radius, axle_length):
    """Convert (v, ω) linear/angular velocity into left/right wheel angular speeds."""
    w_l = (2 * v - omega * axle_length) / (2 * wheel_radius)
    w_r = (2 * v + omega * axle_length) / (2 * wheel_radius)
    return w_l, w_r


# ==================================================================
# WALL FOLLOWER CLASS
# ==================================================================

class WallFollower:
    """PID-based wall-following controller using LIDAR feedback."""

    def __init__(self, robot, side='left', r_target=0.4):
        self.robot = robot
        self.side = side
        self.r_target = r_target

        # PID for distance control
        self.pid = PID(Kp=2.0, Ki=0.0, Kd=0.2,
                       windup=1.0, out_min=-1.5, out_max=1.5)

        # Robot geometry and parameters
        self.wheel_radius = getattr(robot, 'wheel_radius', 0.03)
        self.axle_length = getattr(robot, 'axle_length', 0.12)
        self.v_cruise = 0.18
        self.front_thresh = 0.25
        self.side_lost_thresh = 1.0
        self.side_angle = math.pi / 2 if side == 'left' else -math.pi / 2

    def step(self, dt):
        """Perform one step of wall-following control."""
        distance = self.robot.get_range_image()

        if distance[0] <= self.front_thresh:
            self.robot.set_left_motor_speed(0.0)
            self.robot.set_right_motor_speed(0.0)
            angle = math.pi /2 
            if self.side == 'left':
                rotate_in_place(self.robot, angle)
            else:
                -(math.pi) /2
        else:
            self.robot.set_left_motor_speed(self.v_cruise)
            self.robot.set_right_motor_speed(self.v_cruise)


        # d_front = lidar_front_min(angles, distances)
        # d_side = lidar_angle_distance(angles, distances, self.side_angle)

        # # --- Decision logic ---
        # if d_front is not None and d_front < self.front_thresh:
        #     # Obstacle detected in front → rotate 90° and continue
        #     self.robot.set_left_motor_speed(0.0) 
        #     self.robot.set_right_motor_speed(0.0)
        #     angle = math.pi / 2 if self.side == 'left' else -math.pi / 2
        #     rotate_in_place(self.robot, angle)
        #     self.pid.reset()
        #     return

        # # Lost wall → sweep back toward side
        # if d_side is None or d_side > self.side_lost_thresh:
        #     omega = 0.4 if self.side == 'left' else -0.4
        #     v = 0.1
        # else:
        #     # Maintain lateral offset via PID
        #     error = self.r_target - d_side
        #     omega = self.pid.update(error, dt)
        #     v = self.v_cruise

        # # Apply wheel speeds
        # w_l, w_r = wheel_speeds_from_v_omega(v, omega, self.wheel_radius, self.axle_length)
        # self.robot.set_left_motor_speed(w_l )
        # self.robot.set_right_motor_speed(w_r)


# ==================================================================
# MAIN PROGRAM
# ==================================================================

def run_wall_follow(robot, side='left', duration=60.0):
    """Run wall following for a fixed duration."""
    follower = WallFollower(robot, side)
    dt = 0.05
    start_time = time.time()

    while time.time() - start_time < duration:
        follower.step(dt)
        time.sleep(dt)

    robot.set_left_motor_speed(0.0)
    robot.set_right_motor_speed(0.0)


def main():
    """
    Main entry point — runs left and right wall-following
    on maze1.xml and maze3.xml sequentially.
    """

    # Initialize robot (assumes simulator API)
    robot = HamBot(lidar_enabled=True, camera_enabled=False)

    print("→ Following LEFT wall")
    run_wall_follow(robot, side='left', duration=60.0)    
    robot.stop()
    time.sleep(2.0)

    # print("→ Following RIGHT wall")
    # run_wall_follow(robot, side='right', duration=60.0)
    # robot.stop()
    # time.sleep(2.0)

    print(" All wall-following runs complete.")


if __name__ == "__main__":
    main()

###########################################################################3333

# def clamp(x, lo, hi): return max(lo, min(hi, x))

# def safe_min(vals):
#     good = [v for v in vals if v > 0.0]
#     return min(good) if good else float("inf")

# def _median(xs):
#     ys = sorted(xs)
#     n = len(ys)
#     if n == 0: return float("inf")
#     return ys[n//2]

# def normalize_scan_to_m(scan):
#     """
#     Return a list of distances in meters.
#     Detects units per frame: if typical magnitudes look like
#       - mm (100..5000) -> scale 0.001
#       - cm (10..500)   -> scale 0.01
#       - m  (0.05..10)  -> scale 1.0
#     """
#     # Filter to a subset of non-zero samples
#     samples = [v for v in scan if v > 0.0]
#     if not samples:
#         return [float("inf")] * len(scan)

#     med = _median(samples[: min(200, len(samples))])

#     if med > 50.0:      # likely mm
#         s = 0.001
#     elif med > 5.0:     # likely cm
#         s = 0.01
#     else:               # likely meters
#         s = 1.0

#     out = []
#     for v in scan:
#         if v <= 0.0:
#             out.append(float("inf"))
#         else:
#             m = v * s
#             # clamp absurd values
#             out.append(m if m < 20.0 else float("inf"))
#     return out

# def main():
#     bot = HamBot(lidar_enabled=True, camera_enabled=False)
#     time.sleep(0.3)

#     side_e_prev = 0.0
#     fwd_e_prev  = 0.0

#     print(f"[Task2] Wall follow {FOLLOW_SIDE}, target side={TARGET_SIDE:.2f} m")

#     try:
#         # One-time: show a sanity line so you can confirm sectors & units
#         scan0 = bot.get_range_image()
#         scan0_m = normalize_scan_to_m(scan0)
#         print("[SANITY] Front/Left/Right/Back (m):",
#               round(safe_min(scan0_m[FRONT_S]),3),
#               round(safe_min(scan0_m[LEFT_S]),3),
#               round(safe_min(scan0_m[RIGHT_S]),3),
#               round(safe_min(scan0_m[BACK_S]),3))

#         while True:
#             scan = bot.get_range_image()
#             if scan == -1 or len(scan) < 360:
#                 print("[WARN] LIDAR not ready.")
#                 bot.stop_motors()
#                 time.sleep(0.1)
#                 continue

#             scan_m = normalize_scan_to_m(scan)
#             front_d = safe_min(scan_m[FRONT_S])
#             left_d  = safe_min(scan_m[LEFT_S])
#             right_d = safe_min(scan_m[RIGHT_S])
#             # back_d  = safe_min(scan_m[BACK_S])  # not needed here

#             side_d = left_d if FOLLOW_SIDE == "left" else right_d

#             # Emergency stop
            

#             # Corner turn
#             if front_d > TURN_TRIGGER:
#                 bot.stop_motors()
#                 time.sleep(0.05)
#                 print(f"[TURN] Corner: front={front_d:.2f} m")
#                 x = Stop(bot, front_d)
#                 if x == True:  break

#                 if FOLLOW_SIDE == "left":
#                     bot.set_left_motor_speed(-TURN_RPM)
#                     bot.set_right_motor_speed(+TURN_RPM)
#                 else:
#                     bot.set_left_motor_speed(+TURN_RPM)
#                     bot.set_right_motor_speed(-TURN_RPM)
#                 time.sleep(TURN_SEC)
#                 bot.stop_motors()
#                 time.sleep(0.05)
#                 side_e_prev = 0.0
#                 fwd_e_prev  = 0.0
#                 continue
            
            

#             # Wall gone (open corner) → wrap-search
#             if side_d >= WALL_GONE:
#                 print(f"[WRAP] Side too far: {side_d:.2f} m → search {FOLLOW_SIDE}")
                
                
#                 if FOLLOW_SIDE == "left":
#                     bot.set_left_motor_speed(-SEARCH_RPM)
#                     bot.set_right_motor_speed(+SEARCH_RPM)
#                 else:
#                     bot.set_left_motor_speed(+SEARCH_RPM)
#                     bot.set_right_motor_speed(-SEARCH_RPM)
#                 x = Stop(bot, side_d)
#                 if x == True:  break

#                 time.sleep(SEARCH_SEC)
#                 bot.stop_motors()
#                 time.sleep(0.02)
#                 continue

#             # Forward speed (P + small D) toward ~0.5 m front buffer
#             fwd_e = front_d - 0.50
#             fwd_de = (fwd_e - fwd_e_prev) / DT
#             fwd_e_prev = fwd_e
#             v_fwd = CRUISE_RPM + 30.0 * (FWD_KP * fwd_e + FWD_KD * fwd_de)
#             v_fwd = clamp(v_fwd, 0.0, CRUISE_RPM)

#             # Side steering (P + D): too close → steer away
#             side_e = TARGET_SIDE - side_d
#             side_de = (side_e - side_e_prev) / DT
#             side_e_prev = side_e
#             steer = SIDE_KP * side_e + SIDE_KD * side_de
#             steer = -steer if FOLLOW_SIDE == "left" else +steer

#             # Keep steering bounded relative to forward
#             steer = clamp(steer, -0.45 * v_fwd, 0.45 * v_fwd)

#             left_rpm  = clamp(v_fwd + steer, -MAX_RPM, MAX_RPM)
#             right_rpm = clamp(v_fwd - steer, -MAX_RPM, MAX_RPM)
#             bot.set_left_motor_speed(left_rpm)
#             bot.set_right_motor_speed(right_rpm)

#             print(f"[FOLLOW] side={side_d:.2f} m  front={front_d:.2f} m  "
#                   f"vf={v_fwd:.1f} rpm  steer={steer:+.1f} rpm  "
#                   f"L={left_rpm:.1f} R={right_rpm:.1f}")

#             time.sleep(DT)

#     except KeyboardInterrupt:
#         print("\n[Task2] Stopped by user.")
#     finally:
#         bot.stop_motors()

# def Stop(bot, front_d):
#     if front_d <= STOP_HARD:
#                 bot.stop_motors()
#                 print(f"[STOP] front={front_d:.2f} m (emergency)")
#                 time.sleep(0.1)
#                 return True
#     else:
#         return False
    
# if __name__ == "__main__":
#     main()