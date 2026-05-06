import os
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_launch').find('car_launch')


    car_serial_node = Node(
        package='car_pkg',
        executable='car_serial',

    )

    car_controller_node = Node(
        package='car_pkg',
        executable='car_control',

    )

    diff_drive_node = Node(
        package='car_pkg',
        executable='diff_tf',

    )

    imu_node = Node(
        package='imu_pkg',
        executable='imu_node',
        
    )

    laser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'x3_ydlidar_launch.py'
            )
        )
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                pkg_share,
                'config/ekf.yaml',
            ),
            {'use_sim_time': False}
        ]
    )

    tf_node = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        node_name='static_tf_pub_base_link',
        arguments = ['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', '1.0', 'base_footprint', 'base_link']
    )

    tf_node2 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        node_name='static_tf_pub_imu_link',
        arguments = ['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', '1.0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        # tf_node,
        tf_node2,
        car_serial_node,
        # car_controller_node,
        diff_drive_node,
        imu_node,
        laser_node,
        # robot_localization_node
    ])

