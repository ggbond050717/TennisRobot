from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    robot_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'),
         'launch', 'display_robot.launch.py']
    )
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory',  PathJoinSubstitution([FindPackageShare('slam_pkg'), 'config']),
            '-configuration_basename', 'my_robot.lua'],
        remappings = [
            ('scan', 'scan'),
            ('odom', '/odom'),
            ],
        output = 'screen'
        )
    
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        arguments=['-publish_period_sec', '1.0']
            
        )
    
    tf_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
        )
    
    rosbot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_file)
        )
    
    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')
    
    return LaunchDescription([
        use_sim_time_arg,
        # rosbot_description,
        # tf_node,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])

