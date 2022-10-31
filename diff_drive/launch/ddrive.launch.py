"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file."""


import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # robot_name = 'diff_drive'
    # world_file_name = 'ddrive.world'

    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='ros_ign_gazebo').find('ros_ign_gazebo')

    # pkg_gazebo_ros = get_package_share_directory('ros_ign_gazebo')

    SHARE = FindPackageShare(package='diff_drive').find('diff_drive')

    # world_path = os.path.join(get_package_share_directory(
    #     robot_name), 'worlds', world_file_name)

    world_path = os.path.join(SHARE, 'ddrive.world')

    # urdf_tutorial_path = get_package_share_path('diff_drive')
    # yaml_path = urdf_tutorial_path / 'ddrive.world'

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')


    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'gz_args': world_path
            }.items()
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(gz_sim)

    return ld

    # return LaunchDescription([

    #     gz_sim

    # ])

    # 'ign_args': '-v 4 -r ' + str(yaml_path)

    # '-v 4 -r /home/marno/Classes/Fall22/Embedde_Robotics_ROS/ros2_ws_hw3/src/homework3-Marnonel6/diff_drive/worlds/ddrive.world'