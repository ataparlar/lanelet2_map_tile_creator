import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('dummy_lanelet2')

    dummy_lanelet2_file_param = os.path.join(path_package, 'params/dummy_lanelet2_params.yaml')
    dummy_lanelet2_node = Node(
        package='dummy_lanelet2',
        executable='dummy_lanelet2_exe',
        parameters=[dummy_lanelet2_file_param]
    )


    return launch.LaunchDescription(
        [dummy_lanelet2_node])