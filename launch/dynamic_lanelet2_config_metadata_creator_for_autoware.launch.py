import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('dummy_lanelet2')

    dynamic_lanelet2_config_metadata_creator_params = os.path.join(path_package,
                                               'config/dynamic_lanelet2_config_metadata_creator_for_autoware_params.yaml')
    dynamic_lanelet2_config_metadata_creator_node = Node(
        package='dummy_lanelet2',
        executable='dynamic_lanelet2_config_metadata_creator_exe',
        parameters=[dynamic_lanelet2_config_metadata_creator_params]
    )

    return launch.LaunchDescription(
        [dynamic_lanelet2_config_metadata_creator_node])
