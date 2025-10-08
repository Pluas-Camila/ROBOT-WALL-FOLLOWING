# 🧭 ROBOT-WALL-FOLLOWING

## **Control of Mobile Robots: Wall Following**

### **Objective**
Apply a **PID controller** to navigate parallel to a wall and stop at a desired distance from an end wall.  
Learn how to use **LIDAR measurements** for wall distance estimation and control.

---

### **Robot Sensors Utilized**
**Sensor:** LIDAR  
In **FAIRIS-Lite**, the LIDAR is exposed on the robot via the `MyRobot` class and accessed through:

```python
robot.get_lidar_range_image()
```

This returns a list of **360 distance values** representing a **full 360° scan** around the robot.

---

### **References**
- [Webots LIDAR Reference](https://www.cyberbotics.com/doc/reference/lidar)
- FAIRIS-Lite Documentation (LIDAR usage): *FAIRIS-Lite README*

---

## **PID Controller**

A **PID controller** adjusts control inputs based on the error between the desired and measured distances to the wall.

| Symbol | Definition |
|:-------|:------------|
| `r(t)` | Desired distance to wall |
| `y(t)` | Measured distance to wall |
| `e(t)` | Error = r(t) − y(t) |
| `Kp, Ki, Kd` | PID gains (Proportional, Integral, Derivative) |
| `u(t)` | Control signal (before saturation) |
| `ur(t)` | Saturated control signal, `ur(t) = fSat(u(t))` |

---

## **Task 1 — PID Forward Wall Stop**

### **Goal**
Use forward PID control to **approach and stop** at a target distance of **1.0 m** from a wall.  
The robot starts **6.5 m** away from the wall.

### **World Configuration**
- **World file:** `maze2.xml`
- **Controller line:**  
  ```python
  maze_file = '../../worlds/Fall25/maze2.xml'
  ```
- **Timestep:** `dt = 0.032 s`

---

### **PID Tuning**
Explore the following gain values individually for **Kp, Ki, Kd**:
```
0.001, 0.01, 0.5, 1.0, 5.0, 10.0
```

---

### **Performance Requirements**
✅ The robot **slows down** smoothly as it approaches the wall.  
✅ Stops **exactly at 1.0 m** without collision.  
✅ If initially closer than 1.0 m, it **backs up** to maintain that distance.  
✅ **Continuously print** the measured forward distance during testing.

---

## **Task 2 — Wall Following**

### **Goal**
Implement **left-wall** or **right-wall following** behavior on the mazes:
- `maze1.xml`
- `maze3.xml`

---

### **Requirements**
- Maintain consistent lateral distance to the selected wall (left or right).
- On detecting a **frontal obstacle**, rotate **90°** and continue following the same wall.
- If the wall **ends (sharp corner)**, smoothly **wrap around** and continue following.

Run **both left and right wall following** on each maze.

---

### **Implementation Hint**
Use an **additional PID controller** for **angular velocity**, based on **side distance error**, to:
- Maintain target **lateral offset**
- **Smoothly turn corners**
- Avoid oscillations and wall collisions

---

### **Deliverables**
- PID-controlled forward motion toward the wall (Task 1)
- Left- and right-wall following behaviors (Task 2)
- Console output showing real-time distance measurements

---

### **Suggested Debug Output Example**
```
Forward Distance: 6.45 m
Forward Distance: 5.98 m
Forward Distance: 1.05 m
Target Reached: Stop
```

---

### **Notes**
- Start with **low Kp** and gradually increase until stable.
- Tune **Ki** for long-term accuracy (eliminate steady-state error).
- Use **Kd** to reduce oscillations.
- Ensure **no interference** between forward and angular control loops.
