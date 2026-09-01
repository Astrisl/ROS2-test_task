# ROS 2 Safety Collision-Avoidance Node (`safe_node`)

An automated safety node written in C++ for ROS 2 Jazzy that prevents a differential drive robot from colliding with obstacles using LIDAR feedback.

## Overview
The `safe_node` acts as a safety layer sitting between the raw velocity commands (`/cmd_vel_raw`) and the controller (`/diff_drive_controller/cmd_vel`). It processes incoming `/scan` LIDAR data and dynamically intercepts/blocks forward motion if an obstacle enters the predefined safety corridor ahead of the robot.

---

## Key Features
* **Strict Safety Threshold:** Stops the robot reliably before reaching a **0.50m** distance from obstacles.
* **Unrestricted Maneuvering:** Allows backward movement and in-place rotation even when forward motion is blocked.
* **Corridor-Based Detection:** Uses Cartesian projection ($x, y$) relative to the robot chassis instead of naive angle filtering, avoiding side collisions.
* **Robust Data Filtering:** Safely ignores `NaN`, `Inf`, and near-field self-reflections (chassis/wheels).

---

## Design Choices & Assumptions

### 1. Safety Corridor Architecture (Cartesian Projection)
Instead of filtering scan points by raw angle range, points are converted into Cartesian coordinates relative to the robot frame:
$$x = r \cdot \cos(\theta), \quad y = r \cdot \sin(\theta)$$

* **Rectangular Corridor:** Checks $x \in (0.20\text{m}, 0.50\text{m}]$ and $|y| \le (w_{\text{robot}} / 2) + d_{\text{clearance}}$.
* **Why this approach?** It precisely tracks obstacles directly in the robot's driving path while allowing the robot to pass close to walls or navigate narrow doorways without false triggers.

### 2. Signal Interception & Velocity Control
* **Subscriptions:** `/cmd_vel_raw` (`geometry_msgs/msg/TwistStamped`), `/scan` (`sensor_msgs/msg/LaserScan`).
* **Publisher:** `/diff_drive_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`, QoS: Best Effort).
* **Logic:** If $x \le 0.50\text{m}$ inside the corridor, `twist.linear.x` is overridden to `0.0` for forward commands (`linear.x > 0.0`). Angular commands (`angular.z`) and rearward motion (`linear.x < 0.0`) remain unaffected.

### 3. Assumptions
* LIDAR frame $0^\circ$ points directly forward along the robot's +X axis.
* Chassis self-reflections occur within a $0.20\text{m}$ radius from the sensor origin and are filtered out.
* Simulation time (`use_sim_time`) is enabled to maintain clock synchronization with Gazebo.

---

## How to Run Locally with Docker Container

### 1. Navigate to Workspace Source Folder & Clone Repository
Inside the running Docker container, navigate to the source directory of the workspace, remove any old package version, and pull the updated package:

```bash
# 1. Navigate to the ROS 2 workspace source directory
cd /root/ros2_ws/src

# 2. If an old version of the package exists, remove it
rm -rf my_diff_robot

# 3. Clone repository and set up package directory
git clone [https://github.com/Astrisl/ROS2-test_task.git](https://github.com/Astrisl/ROS2-test_task.git) temp && mv temp/my_diff_robot . && rm -rf temp

# 3. Clone YOUR repository (replace with your actual GitHub URL):
git clone https://github.com/Astrisl/ROS2-test_task.git temp_repo
cp -r temp_repo/my_diff_robot ./
rm -rf temp_repo
```


### 2. Build the Package
Inside your ROS 2 workspace container (`/root/ros2_ws`):

```
cd ~/ros2_ws
colcon build --packages-select my_diff_robot
source install/setup.bash
```

### 3. Launch the Simulation Environment
Start Gazebo, `ros2_control`, and RViz:

```
ros2 launch my_diff_robot robot.launch.py
```

### 4. Run the Safety Node
In a separate terminal inside the container:

```
source ~/ros2_ws/install/setup.bash
ros2 run my_diff_robot safe_node --ros-args -p use_sim_time:=true
```
### 5. Teleoperate the Robot
Send velocity commands to /cmd_vel_raw in a separate terminal:

```
python3 simple_teleop.py
```
Or via CLI:
```
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}}, twist: {linear: {x: 0.4, y: 0.0, z: 0.0}}}"
```