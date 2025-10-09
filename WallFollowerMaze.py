 
"""
WallFollowerMaze

Implements:
 - left / right wall following using side PID + forward-speed PID
 - on frontal obstacle: rotate 90 degrees and resume following same wall
 - on wall-end (sharp corner): "wrap around" search to reacquire wall and continue
 - emergency stop to avoid bumps

HOW TO ADAPT:
 - Override the robot API methods (get_lidar_range_image, get_heading_deg, set_left_motor_velocity,
   set_right_motor_velocity, step) to call your robot/simulator.
 - Adjust lidar index slices (front_sector, left_sector, right_sector) to match your lidar layout.
 - Tune PID gains & thresholds for your robot dynamics.
"""

import math
import time
from typing import Sequence, Optional


class WallFollowerMaze:
    # --- STATES for state-machine ---
    STATE_FOLLOW = 0
    STATE_ROTATE_90 = 1
    STATE_WRAP_SEARCH = 2

    def __init__(
        self,
        timestep_ms: int = 64,
        max_motor_velocity: float = 10.0,
        follow_side: str = "left",                 # "left" or "right"
        front_sector: slice = slice(175, 186),
        left_sector: slice = slice(90, 116),
        right_sector: slice = slice(244, 280),
        emergency_stop_distance: float = 0.12,     # m
        desired_side_distance: float = 0.2,        # target lateral offset from wall
        desired_forward_distance: float = 0.45,    # target forward distance before braking/rotate
    ):
        self.timestep = timestep_ms
        self.max_motor_velocity = float(max_motor_velocity)
        self.follow_side = follow_side.lower()
        if self.follow_side not in ("left", "right"):
            raise ValueError("follow_side must be 'left' or 'right'")

        # lidar sectors
        self.front_sector = front_sector
        self.left_sector = left_sector
        self.right_sector = right_sector

        # safety & targets
        self.emergency_stop_distance = emergency_stop_distance
        self.desired_side = desired_side_distance
        self.desired_forward = desired_forward_distance

        # PID parameters (tune)
        # Side PID (primary for angular correction)
        self.side_Kp = 2.2
        self.side_Ki = 0.0
        self.side_Kd = 0.4

        # Forward PID (reduce forward speed as obstacle approaches)
        self.forward_Kp = 1.2
        self.forward_Ki = 0.0
        self.forward_Kd = 0.08
        self.base_forward_speed = 1.0

        # Rotation PID for precise rotations using IMU (keep Kp < 1 per your note)
        self.rot_Kp = 0.7
        self.rot_Ki = 0.0
        self.rot_Kd = 0.03

        # internal PID state
        self.side_integral = 0.0
        self.side_prev_err = 0.0
        self.forward_integral = 0.0
        self.forward_prev_err = 0.0
        self.rot_integral = 0.0
        self.rot_prev_err = 0.0

        self.integral_limit = 5.0

        # wrap/search behavior parameters
        self.wrap_side_threshold = 0.6   # if side distance exceeds this, treat as wall-end
        self.search_turn_speed = 0.6     # wheel difference when slowly searching for wall
        self.max_search_rotation_deg = 360.0

        # correction scaling to convert side PID output to wheel differential (tune)
        self.correction_scale = 0.5

        # runtime state
        self.state = self.STATE_FOLLOW
        self.rotate_target_heading = None
        self.search_start_heading = None

    # ----------------------------
    # --- Robot API placeholders ---
    # ----------------------------
    # Override these methods for your robot/simulator

    def get_lidar_range_image(self) -> Sequence[float]:
        """Return sequence of lidar ranges (meters)."""
        raise NotImplementedError("Override get_lidar_range_image()")

    def get_heading_deg(self) -> float:
        """Return yaw/heading in degrees [0, 360)."""
        raise NotImplementedError("Override get_heading_deg()")

    def set_left_motor_velocity(self, v: float):
        raise NotImplementedError("Override set_left_motor_velocity()")

    def set_right_motor_velocity(self, v: float):
        raise NotImplementedError("Override set_right_motor_velocity()")

    def stop(self):
        """Stop motors immediately."""
        self.set_left_motor_velocity(0.0)
        self.set_right_motor_velocity(0.0)

    def step(self) -> int:
        """Advance simulator one timestep. Return -1 to stop. Override in subclass."""
        raise NotImplementedError("Override step()")

    # ----------------------------
    # --- Helpers & utilities ---
    # ----------------------------
    def sat(self, v: float) -> float:
        if v > self.max_motor_velocity:
            return self.max_motor_velocity
        if v < -self.max_motor_velocity:
            return -self.max_motor_velocity
        return v

    @staticmethod
    def _safe_min(values: Sequence[float], fallback=float("inf")) -> float:
        cleaned = [v for v in values if v is not None and not math.isinf(v) and not math.isnan(v) and v > 0.0]
        return min(cleaned) if cleaned else fallback

    def check_emergency(self) -> bool:
        lidar = self.get_lidar_range_image()
        front_min = self._safe_min(lidar[self.front_sector], fallback=float("inf"))
        return front_min <= self.emergency_stop_distance

    # ----------------------------
    # --- PID computation ---
    # ----------------------------
    def _pid_step(self, error, prev_err, integral, Kp, Ki, Kd, dt):
        derivative = (error - prev_err) / dt if dt > 0 else 0.0
        integral = integral + error * dt
        integral = max(-self.integral_limit, min(self.integral_limit, integral))
        out = Kp * error + Ki * integral + Kd * derivative
        return out, integral, error

    def compute_forward_velocity(self, dt):
        lidar = self.get_lidar_range_image()
        front = self._safe_min(lidar[self.front_sector], fallback=float("inf"))
        # error = actual - desired -> positive when farther than desired
        error = front - self.desired_forward
        out, self.forward_integral, self.forward_prev_err = self._pid_step(
            error, self.forward_prev_err, self.forward_integral, self.forward_Kp, self.forward_Ki, self.forward_Kd, dt
        )
        forward_v = self.base_forward_speed + out
        return self.sat(forward_v), front

    def compute_side_correction(self, dt):
        lidar = self.get_lidar_range_image()
        if self.follow_side == "left":
            side_actual = self._safe_min(lidar[self.left_sector], fallback=float("inf"))
        else:
            side_actual = self._safe_min(lidar[self.right_sector], fallback=float("inf"))

        # error = desired - actual (positive if robot is too close)
        error = self.desired_side - side_actual
        out, self.side_integral, self.side_prev_err = self._pid_step(
            error, self.side_prev_err, self.side_integral, self.side_Kp, self.side_Ki, self.side_Kd, dt
        )
        return out, side_actual

    def rotation_pid_output(self, target_heading, dt):
        curr = self.get_heading_deg() % 360.0
        delta = (target_heading - curr + 180.0) % 360.0 - 180.0  # shortest angle -180..180
        out, self.rot_integral, self.rot_prev_err = self._pid_step(
            delta, self.rot_prev_err, self.rot_integral, self.rot_Kp, self.rot_Ki, self.rot_Kd, dt
        )
        return out, delta

    # ----------------------------
    # --- Rotation helpers ---
    # ----------------------------
    def start_rotate_90(self):
        """Schedule a 90-degree rotation while preserving wall-side."""
        curr = self.get_heading_deg() % 360.0
        if self.follow_side == "left":
            # turning away from front obstacle so left-wall stays left after rotation:
            # if following left and obstacle ahead, rotate +90 (turn right) keeps wall on left
            target = (curr + 90.0) % 360.0
        else:
            # following right -> rotate -90 (turn left)
            target = (curr - 90.0) % 360.0
        self.rotate_target_heading = target
        self.state = self.STATE_ROTATE_90

    def do_rotate_step(self, dt):
        """Perform rotation toward rotate_target_heading using rotation PID.
           Returns True when rotation is complete, else False.
        """
        target = self.rotate_target_heading
        if target is None:
            return True
        out, angle_err = self.rotation_pid_output(target, dt)
        # convert out (deg error-based) into wheel differential: scale appropriately
        # sign convention: positive out -> need positive angular velocity (turn right)
        # map to wheel velocity difference:
        wheel_diff = 0.02 * out  # scale factor, tune this
        # apply opposite signs to wheels to rotate in place
        left_v = self.sat(-wheel_diff)
        right_v = self.sat(wheel_diff)
        self.set_left_motor_velocity(left_v)
        self.set_right_motor_velocity(right_v)

        # consider rotated if angle error small
        if abs(angle_err) < 2.0:  # degrees tolerance
            self.rotate_target_heading = None
            return True
        return False

    # ----------------------------
    # --- Wrap-around search ---
    # ----------------------------
    def start_wrap_search(self):
        """Called when the side wall 'ends' (sudden large side distance). We'll rotate slowly toward the side
           where the wall should continue and search until side sensor finds a wall or we've rotated a lot.
        """
        self.state = self.STATE_WRAP_SEARCH
        self.search_start_heading = self.get_heading_deg() % 360.0
        # stop integrators so search starts fresh
        self.side_integral = 0.0
        self.side_prev_err = 0.0

    def do_wrap_search_step(self, dt):
        """Rotate slowly toward the side we expect the wall to be and check side distance."""
        lidar = self.get_lidar_range_image()
        if self.follow_side == "left":
            side_actual = self._safe_min(lidar[self.left_sector], fallback=float("inf"))
            # rotate counter-clockwise slowly to 'wrap' around obstacle (left-follow -> rotate left to find wall)
            left_v = -self.search_turn_speed
            right_v = self.search_turn_speed
        else:
            side_actual = self._safe_min(lidar[self.right_sector], fallback=float("inf"))
            # right-follow -> rotate clockwise slowly to find wall
            left_v = self.search_turn_speed
            right_v = -self.search_turn_speed

        self.set_left_motor_velocity(self.sat(left_v))
        self.set_right_motor_velocity(self.sat(right_v))

        # If we find a wall within a reasonable side distance, resume follow
        if side_actual < self.wrap_side_threshold:
            self.state = self.STATE_FOLLOW
            return True

        # If we've searched more than allowed rotation, give up and go forward a bit then resume follow check
        curr_heading = self.get_heading_deg() % 360.0
        turned = abs((curr_heading - self.search_start_heading + 180.0) % 360.0 - 180.0)
        if turned >= self.max_search_rotation_deg:
            # stop rotation, attempt brief forward move to try wrapping around
            self.set_left_motor_velocity(0.0)
            self.set_right_motor_velocity(0.0)
            self.state = self.STATE_FOLLOW
            return True

        return False

    # ----------------------------
    # --- Main compute step ---
    # ----------------------------
    def compute_and_act(self, dt):
        """
        Called every control cycle. Decides what to do based on state machine.
        """

        # emergency stop immediate
        if self.check_emergency():
            self.stop()
            print("EMERGENCY STOP: obstacle too close")
            return

        if self.state == self.STATE_FOLLOW:
            # normal wall-following behavior
            forward_v, front_distance = self.compute_forward_velocity(dt)
            side_corr, side_distance = self.compute_side_correction(dt)

            # If frontal obstacle closer than rotation trigger, rotate 90 deg and resume
            # trigger threshold slightly larger than emergency_stop_distance
            rotate_trigger_dist = max(self.emergency_stop_distance + 0.02, self.desired_forward * 0.6)
            if front_distance <= rotate_trigger_dist:
                # begin rotation preserving wall side
                self.start_rotate_90()
                return

            # If the side distance becomes very large we consider the wall ended — start wrap search
            if side_distance >= self.wrap_side_threshold:
                # begin wrap around search for the same wall
                self.start_wrap_search()
                return

            # Map side correction to wheel velocities: produce differential
            wheel_diff = self.correction_scale * side_corr
            if self.follow_side == "left":
                left_v = self.sat(forward_v - wheel_diff)
                right_v = self.sat(forward_v + wheel_diff)
            else:
                left_v = self.sat(forward_v + wheel_diff)
                right_v = self.sat(forward_v - wheel_diff)

            # command motors
            self.set_left_motor_velocity(left_v)
            self.set_right_motor_velocity(right_v)

        elif self.state == self.STATE_ROTATE_90:
            finished = self.do_rotate_step(dt)
            if finished:
                # after rotation, resume following (integrators reset optional)
                self.state = self.STATE_FOLLOW
                # small pause to settle
                self.set_left_motor_velocity(0.0)
                self.set_right_motor_velocity(0.0)
                # allow a single small forward burst so we don't get re-triggered by the same obstacle
                time.sleep(0.04)  # small real-time pause; fine-tune or remove if your step() blocks
            return

        elif self.state == self.STATE_WRAP_SEARCH:
            finished = self.do_wrap_search_step(dt)
            if finished:
                # resumed to follow state inside do_wrap_search_step
                pass
            return

    # ----------------------------
    # --- High-level runner ---
    # ----------------------------
    def run(self, desired_forward: Optional[float] = None, desired_side: Optional[float] = None, max_runtime_s: Optional[float] = None):
        """Top-level loop you can call once robot API overrides are in place."""
        if desired_forward is not None:
            self.desired_forward = desired_forward
        if desired_side is not None:
            self.desired_side = desired_side

        start_time = time.time()
        prev = start_time

        while True:
            status = self.step()
            if status == -1:
                self.stop()
                break

            now = time.time()
            dt = now - prev if now > prev else self.timestep / 1000.0
            prev = now

            # Run controller
            self.compute_and_act(dt)

            if max_runtime_s is not None and (now - start_time) >= max_runtime_s:
                self.stop()
                break

            # Respect loop rate (if step doesn't block)
            time.sleep(self.timestep / 1000.0)

        # final stop
        self.stop()
