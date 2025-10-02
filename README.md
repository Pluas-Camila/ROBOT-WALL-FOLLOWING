# ROBOT-WALL-FOLLOWING
Control of Mobile Robots (Wall following)
Objective
Apply a PID controller to navigate parallel to a wall and stop at a desired distance from an end wall. Learn how to use LIDAR measurements for wall distance estimation and control.

Robot Sensors Utilized: LIDAR
In FAIRIS-Lite, the LIDAR is exposed on the robot via the MyRobot class and accessed through robot.get_lidar_range_image(). This returns a list of 360 distances over a full 360° scan.

References

Webots LIDAR reference: https://www.cyberbotics.com/doc/reference/lidarLinks to an external site.
FAIRIS-Lite docs (LIDAR usage): FAIRIS-Lite READMELinks to an external site.
PID Controller
Use the LIDAR to drive a full PID controller that sets motor velocities proportional to wall distance error.

Definitions
r(t): desired distance to wall
y(t): measured distance to wall
e(t) = r(t) − y(t): distance error
Kp, Ki, Kd: PID gains
u(t): control (pre-saturation)
ur(t) = fSat(u(t)): saturated control

Task 1 — PID Forward Wall Stop
Use forward PID gains to control motion toward an end wall. The robot must approach and stop 1.0 m from the wall, without influencing lateral motion and without hitting walls. Start at 6.5 m from the end wall (see Figure 3).

World: maze2.xml   (controller line: maze_file = '../../worlds/Fall25/maze2.xml')
Timestep: dt = 0.032
Explore gains (apply separately to each of Kp, Ki, Kd): 0.001, 0.01, 0.5, 1.0, 5.0, 10.0
Robot slows as it nears the wall and rests at 1.0 m
If placed < 1.0 m from the wall, it should back up to 1.0 m
Continuously print the measured forward distance during testing


Task 2 — Wall Following
Implement left- or right-wall following on two mazes (maze1.xml, maze3.xml).

Maintain consistent contact with the chosen wall (left or right) throughout.
On a frontal obstacle, rotate 90° and resume following the same wall.
If the wall ends (sharp corner), wrap around and continue following the same wall.
Run both left and right wall following on each maze.
Hint. Consider an additional PID using side distance for angular velocity to maintain a target lateral offset and to smoothly round corners.
