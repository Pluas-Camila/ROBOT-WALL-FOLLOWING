from robot_systems.robot import HamBot
import math
import time

class WallFollowerPID(HamBot):
    def __init__(self):
        super().__init__()
        self.timestep = 32
        self.wheel_radius = 0.02
        self.max_motor_velocity = 6.28
        
        # PID control variables
        self.prev_error = 0
        self.error_sum = 0
        
        # PID constants (tune these)
        self.Kp = 3.0
        self.Ki = 0.0
        self.Kd = 0.5

    # -------------------------
    # SENSOR HELPERS
    # -------------------------
    def get_left_side_distance(self):
        return min(self.get_lidar_range_image()[90:175])

    def get_right_side_distance(self):
        return min(self.get_lidar_range_image()[185:270])

    def get_front_distance(self):
        return min(self.get_lidar_range_image()[175:185])

    # -------------------------
    # MOTOR + CONTROL UTILITIES
    # -------------------------
    def sat(self, v):
        """Limit velocity to robot capabilities."""
        return max(-self.max_motor_velocity, min(self.max_motor_velocity, v))

    def stop(self):
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

    # -------------------------
    # PID-BASED ROTATION
    # -------------------------
    def rotate_to(self, end_bearing, margin_error=0.5):
        """Rotate until compass reading matches desired bearing."""
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.rotation_PID(end_bearing)

            heading = self.get_compass_reading()
            if end_bearing - margin_error <= heading <= end_bearing + margin_error:
                self.stop()
                break

    def rotation_PID(self, target_bearing):
        current = self.get_compass_reading()
        error = (target_bearing - current + 540) % 360 - 180  # shortest turn direction
        phi = self.sat(self.Kp * error * 0.01)
        
        if error > 0:
            self.set_left_motor_velocity(-phi)
            self.set_right_motor_velocity(phi)
        else:
            self.set_left_motor_velocity(phi)
            self.set_right_motor_velocity(-phi)

    # -------------------------
    # WALL FOLLOWING PID
    # -------------------------
    def wall_follow_PID(self, side='left', desired_dist=0.2):
        """Maintain distance to wall using PID on lidar."""
        if side == 'left':
            actual_dist = self.get_left_side_distance()
        else:
            actual_dist = self.get_right_side_distance()

        # Error: actual - desired
        error = desired_dist - actual_dist

        # PID terms
        self.error_sum += error
        delta_error = error - self.prev_error
        self.prev_error = error

        angular_correction = self.Kp * error + self.Ki * self.error_sum + self.Kd * delta_error
        return self.sat(angular_correction)

    # -------------------------
    # GO FORWARD + AVOID COLLISION
    # -------------------------
    def go_forward(self, linear_speed=3.0):
        """Move forward safely, applying wall PID correction."""
        front_dist = self.get_front_distance()
        if front_dist < 0.25:  # too close to wall
            self.stop()
            self.rotate_90()
            return

        # Determine steering correction
        correction = self.wall_follow_PID(side='left')
        left_v = self.sat(linear_speed - correction)
        right_v = self.sat(linear_speed + correction)
        self.set_left_motor_velocity(left_v)
        self.set_right_motor_velocity(right_v)

    # -------------------------
    # 90° ROTATION LOGIC
    # -------------------------
    def rotate_90(self, angle=90):
        """Rotate 90 degrees (turn away from obstacle)."""
        start_heading = self.get_compass_reading()
        end_bearing = (start_heading + angle) % 360
        if end_bearing >= 360:
            end_bearing -= 360
        self.rotate_to(end_bearing)

    # -------------------------
    # MAIN CONTROL LOOP
    # -------------------------
    def run(self):
        """Main loop: wall follow indefinitely."""
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.go_forward()
