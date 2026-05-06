import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf_package_path = get_package_share_directory('robot_description')
    xcaro_file_path = os.path.join(urdf_package_path, 'urdf', 'TennisRobot/TennisRobot.urdf.xacro')


    return launch.LaunchDescription([
    ])