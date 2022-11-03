"""Launch Gazebo server and client with command line arguments."""
"""Spawn robot from URDF file.




CMD LINE 
ros2 launch diff_drive ddrive.launch.py


"""


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
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # robot_name = 'diff_drive'
    # world_file_name = 'ddrive.world'



    # Set the path to the Gazebo ROS package
    ## pkg_gazebo_ros = FindPackageShare(package='ros_ign_gazebo').find('ros_ign_gazebo')

    pkg_gazebo_ros = get_package_share_directory('ros_ign_gazebo')

    ##SHARE = FindPackageShare(package='diff_drive').find('diff_drive')

    # world_path = os.path.join(get_package_share_directory(
    #     robot_name), 'worlds', world_file_name)

    ##world_path = os.path.join(SHARE, 'ddrive.world')

    urdf_tutorial_path = get_package_share_path('diff_drive')
    yaml_path = urdf_tutorial_path / 'ddrive.world'

    # declare_world_cmd = DeclareLaunchArgument(
    # name='world',
    # default_value=world_path,
    # description='Full path to the world model file to load')


    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'gz_args': '-v 4 -r ' + str(yaml_path)
            }.items()
    )


    # world_path



    urdf_tutorial_path = get_package_share_path('diff_drive')
    default_model_path = urdf_tutorial_path / 'ddrive.urdf.xacro'

    # robot_desc = os.path.join(FindPackageShare(package='diff_drive').find('diff_drive'), 'ddrive.urdf.xacro')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')

    robot_desc = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)


                    # Import the model urdf (load from file, xacro ...)
    # robot_desc = \
    #     '<?xml version="1.0" ?>'\
    #     '<robot name="will_be_ignored">'\
    #     '<link name="link">'\
    #     '<visual>'\
    #     '<geometry>'\
    #     '<sphere radius="10.0"/>'\
    #     '</geometry>'\
    #     '</visual>'\
    #     '<collision>'\
    #     '<geometry>'\
    #     '<sphere radius="10.0"/>'\
    #     '</geometry>'\
    #     '</collision>'\
    #     '<inertial>'\
    #     '<mass value="1"/>'\
    #     '<inertia ixx="1" ixy="0.0" ixz="0.0" iyy="1" iyz="0.0" izz="1"/>'\
    #     '</inertial>'\
    #     '</link>'\
    #     '</robot>'


    """Use
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 launch diff_drive ddrive.launch.py

    """

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_desc}

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

        # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'my_custom_model',
                    '-x', '4.0',
                    '-z', '1.0',
                    '-Y', '4.0',
                    '-topic', '/robot_description'],
                 output='screen')

    bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"])


    flip_robot_node = Node(
        package='diff_drive',
        executable='flip',
        name='flip'
        # parameters=[yaml_path]
    )


    # # Create the launch description and populate
    # ld = LaunchDescription()
    
    # # Declare the launch options
    # ld.add_action(model_arg)
    # ld.add_action(declare_world_cmd)
    # ld.add_action(gz_sim)
    # ld.add_action(robot_state_publisher)
    # ld.add_action(spawn)


    # return ld

    return LaunchDescription([

        model_arg,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge_node,
        flip_robot_node

    ])

    # 'ign_args': '-v 4 -r ' + str(yaml_path)

    # '-v 4 -r /home/marno/Classes/Fall22/Embedde_Robotics_ROS/ros2_ws_hw3/src/homework3-Marnonel6/diff_drive/worlds/ddrive.world'