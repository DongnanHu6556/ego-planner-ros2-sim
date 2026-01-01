from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    # mavros start up
    mavros_start_cmd = ExecuteProcess(
        cmd = ["ros2 launch",
        "mavros",
        "px4.launch",
        "fcu_url:=udp://:14540@127.0.0.1:14555"],
        output = "both",
        shell = True
    )

    # Micro XRCE Agent（PX4 <-> ROS2）
    micro_xrce_start_cmd = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        shell=False,
        output='screen'
    )

    # acquire PX4 path
    PX4_DIR = os.path.expanduser('~/PX4-Autopilot')
    PX4_BIN = os.path.join(PX4_DIR, 'build/px4_sitl_default/bin/px4')

    # 1st PX4 instance
    px4_instance0 = ExecuteProcess(
        cmd=[
            PX4_BIN,
            '-i', '0'
        ],
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_GZ_MODEL_POSE': '-6,0',
            'PX4_SIM_MODEL': 'gz_x500_depth'
        },
        output='screen'
    )

    # 2nd PX4 instance
    px4_instance1 = ExecuteProcess(
        cmd=[PX4_BIN, '-i', '1'],
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_GZ_MODEL_POSE': '0,2',
            'PX4_SIM_MODEL': 'gz_x500'
        },
        output='screen'
    )

    # launch gazebo
    gazebo_start_cmd = ExecuteProcess(
            cmd=[
                'gz', 'sim',
                # LaunchConfiguration('world'),
                # '--verbose'
            ],
            shell=False,
            output='screen'
    )

    # home path
    home_dir = os.path.expanduser('~')

    # gazebo simulation startup script
    gazebo_simulation_cmd = ExecuteProcess(
        cmd=[
            'python3',
            home_dir+'/ros_proj/gazebo_start/simulation-gazebo',
        ],
        output='screen'
    )

    # launch ros_gz_bridge parameter_bridge
    depth_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )

    lidar_point_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_points_bridge',
        arguments=[
            '/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points',
            '/lidar_points')
        ],
        output='screen'
    )

    # best effort depth image publish script
    best_effort_depth_img_pub = ExecuteProcess(
        cmd=[
            'python3',
            home_dir+'/ros_proj/gazebo_start/depth_gz_bridge.py',
        ],
        output='screen'
    )

    # best effort lidar points publish script
    best_effort_lidar_points_pub = ExecuteProcess(
        cmd=[
            'python3',
            home_dir+'/ros_proj/gazebo_start/lidar_gz_bridge.py',
        ],
        output='screen'
    )

    return LaunchDescription([
        mavros_start_cmd,
        gazebo_simulation_cmd,
        px4_instance0,
        # px4_instance1,
        micro_xrce_start_cmd,
        # lidar_point_bridge_node,
        depth_bridge_node,
        best_effort_depth_img_pub,
        # best_effort_lidar_points_pub,
    ])
