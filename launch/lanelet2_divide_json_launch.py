import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('dummy_lanelet2')

    lanelet2_divide_json_params = os.path.join(path_package, 'params/lanelet2_divide_json_params.yaml')
    lanelet2_divide_json_node = Node(
        package='dummy_lanelet2',
        executable='lanelet2_divide_json_exe',
        parameters=[lanelet2_divide_json_params]
    )


    return launch.LaunchDescription(
        [lanelet2_divide_json_node])