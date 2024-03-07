from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_fast_slam_gz = get_package_share_directory("fast_slam_gz_description")
    pkg_fast_slam_ros = get_package_share_directory("fast_slam_ros")

    # Launch bot description
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_slam_gz, "launch", "simple_bot_gz.launch.py")
        )
    )

    fast_slam_node = Node(
        package='fast_slam_ros', 
        executable='fast_slam_ros', 
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_fast_slam_ros, "rviz", "fast_slam_landmarks.rviz"),
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(
        [
            simulation,
            fast_slam_node,
            rviz
        ]    
    )