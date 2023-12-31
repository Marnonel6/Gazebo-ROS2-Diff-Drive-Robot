"""
Launch Gazebo server and client with command line arguments.

Spawn robot from URDF file. Launch rviz with odom.
"""
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch Gazebo and RVIZ with the URDF model."""
    pkg_gazebo_ros = get_package_share_directory('ros_ign_gazebo')
    urdf_tutorial_path = get_package_share_path('diff_drive')
    world_path = urdf_tutorial_path / 'ddrive.world'
    default_model_path = urdf_tutorial_path / 'ddrive.urdf.xacro'

    view_only = DeclareLaunchArgument(
        name='view_only',
        default_value='false',
        choices=[
            'false',
            'true'],
        description='Flag to choose rviz config')

    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'gz_args': '-v 4 -r ' + str(world_path)
            }.items()
    )

    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')

    robot_desc = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

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
                    '-x', '3.0',
                    '-z', '1.0',
                    '-y', '1.0',
                    '-topic', '/robot_description'],
                 output='screen')

    bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                       '/model/my_custom_model/odometry@nav_msgs/msg/' +
                       'Odometry@ignition.msgs.Odometry',
                       '/world/visualize_lidar_world/model/my_custom_model/' +
                       'joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                       '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
                       ],
            remappings=[('/world/visualize_lidar_world/model/my_custom_model' +
                         '/joint_state', '/joint_states')])

    flip_robot_node = Node(
        package='diff_drive',
        executable='flip',
        name='flip'
    )

    ddrive_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('diff_drive')), '/ddrive_rviz.launch.py']))

    return LaunchDescription([
        view_only,
        model_arg,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge_node,
        flip_robot_node,
        ddrive_launch
    ])
