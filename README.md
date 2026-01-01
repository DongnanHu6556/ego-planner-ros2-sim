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
```
git clone https://github.com/DongnanHu6556/ego-planner-ros2-sim.git
cd ./ego-planner-ros2-sim
cp px4_sitl_ros2.launch.py <your_px4_path>/PX4-Autopilot/launch
```

### 1.2 Gazebo Related Script Location
```
mkdir -p ~/ros_proj/gazebo_start
cp simulation-gazebo depth_gz_bridge.py ~/ros_proj/gazebo_start
# This command is used to download worlds and models in .simulation-gazebo
cd ~/ros_proj/gazebo_start && python3 simulation-gazebo
# Then you can put your world and models in .simulation-gazebo. Here I provide my world file ego.sdf
cd ~/ego-planner-ros2-sim
cp ego.sdf ~/.simulation-gazebo/worlds
```
Then change `simulation-gazebo` file:

In line 18 `parser.add_argument('--world', help='World to run in Gazebo', required=False, default="default")`, change the world name form `"default"` to `"ego"`

### 1.3 Launch PX4 Gazebo Simulation
```
# Please notice your launch path
ros2 launch ./PX4-Autopilot/launch/px4_sitl_ros2.launch.py
```
Then you can see the interface:
<img width="1827" height="1039" alt="ego-sim-screenshot" src="https://github.com/user-attachments/assets/7c34ff3f-3833-4d82-8ea5-47a7cb982e16" />

## 2. PX4 Control Fsm
This package is designed to send the Ego Planner’s trajectory to the PX4 low-level controller, while also providing state management. The topics are transmitted via the Micro XRCE Agent.
Please follow this link to install the package: https://github.com/DongnanHu6556/px4_ego/tree/main
```
ros2 run px4_ego_py offboard_control_test
cd px4_ego
python3 mode_key.py 
```
You can input 't' in keyboard terminal to make the drone takeoff. When hovering at the desire height, inputting 'o' to switch the drone to Offboard mode. Then, the drone will start following the trajectory once it is published
 (see Part 3).

<img width="1830" height="1042" alt="image" src="https://github.com/user-attachments/assets/84eef922-1eb4-4218-aa6b-85777ffe9e77" />

