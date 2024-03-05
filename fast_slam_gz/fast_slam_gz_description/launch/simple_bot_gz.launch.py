import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # define package paths
    pkg_description = get_package_share_directory("fast_slam_gz_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    doc = xacro.process_file(os.path.join(pkg_description, "urdf", "simple_bot.urdf.xacro"))
    robot_desc = doc.toprettyxml(indent='  ')

    bridge_config_path = os.path.join(pkg_description, 'config', 'bridge_config.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_desc}],
        arguments=['--ros-args', '--log-level', "info"]
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": " -v4 -s -r " + os.path.join(pkg_description, 'world', "landmark.world"),
            "gz_version": "8"
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-v4 -g " + os.path.join(pkg_description, 'world', "landmark.world"),
            "gz_version": "8"
        }.items(),
    )

    spawner = Node( 
        package='ros_gz_sim', 
        executable='create', 
        arguments=[ '-name', 'simple_bot', '-world', 'landmark', '-topic', 'robot_description', '-z', '0.2'], 
        output='screen', 
    )
    
    

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": bridge_config_path},
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_description, "rviz", "landmark_world.rviz"),
        ],
    )

    return LaunchDescription(
        [
            gz_sim_server,
            gz_sim_gui,
            robot_state_publisher,
            bridge,
            spawner,
            rviz
        ]    
    )