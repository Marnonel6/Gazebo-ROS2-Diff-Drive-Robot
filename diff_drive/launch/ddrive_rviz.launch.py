"""
Launch rviz and load config file.
"""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('diff_drive')
    default_model_path = urdf_tutorial_path / 'ddrive.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'ddrive_urdf_odom.rviz'
    default_rviz_config_path2 = urdf_tutorial_path / 'ddrive_urdf.rviz'

    view_only = DeclareLaunchArgument(
        name='view_only',
        default_value='false',
        choices=[
            'false',
            'true'],
        description='Flag to choose rviz config')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')
    rviz_arg2 = DeclareLaunchArgument(
        name='rvizconfig2',
        default_value=str(default_rviz_config_path2),
        description='Absolute path to rviz config file')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('view_only', 'true')
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=LaunchConfigurationEquals('view_only', 'false')
    )

    rviz_node2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig2')],
        condition=LaunchConfigurationEquals('view_only', 'true')
    )

    return LaunchDescription([
        view_only,
        model_arg,
        rviz_arg,
        rviz_arg2,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        rviz_node2
    ])
