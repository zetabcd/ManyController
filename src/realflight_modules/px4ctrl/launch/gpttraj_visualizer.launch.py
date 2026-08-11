from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('px4ctrl')
    default_params = os.path.join(package_share, 'config', 'gpttraj.yaml')
    default_rviz = os.path.join(package_share, 'config', 'gpttraj.rviz')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Trajectory, CSTC, and visualization configuration')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')

    planner = Node(
        package='px4ctrl', executable='gpttraj_visualizer_node',
        name='gpttraj_visualizer', output='screen',
        parameters=[LaunchConfiguration('params_file')])
    rviz = Node(
        package='rviz2', executable='rviz2', name='gpttraj_rviz',
        arguments=['-d', default_rviz], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([params_arg, rviz_arg, planner, rviz])
