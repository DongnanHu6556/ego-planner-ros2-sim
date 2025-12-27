# ego-planner-ros2-sim
Description of ego planner ros2 simulation in gazebo

## 0. Precondition:

- Ubuntu22.04

- ROS2 Humble

- PX4 Firmware (>= 1.14.0)

- QGroundControl

- Gazebo Harmonic (Tips: use `sudo apt-get install ros-humble-ros-gzharmonic`. The command `sudo apt-get install gz-harmonic` may lead to problems of gz_bridge at following steps)

- Micro XRCE Agent

- Mavros2

## 1. PX4 Gazebo Simulation Setup

### 1.1 PX4 Simulation Launch File Location
- copy `px4_sitl_ros2.launch.py` to your `PX4-Autopilot/launch`

### 1.2 Gazebo Related Script Location
```
mkdir -p ~/ros_proj/gazebo_start
cp simulation-gazebo depth_gz_bridge.py ~/ros_proj/gazebo_start
```
