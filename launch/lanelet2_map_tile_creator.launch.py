import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('dummy_lanelet2')

    lanelet2_map_tile_creator_params = os.path.join(path_package,
                                               'config/lanelet2_map_tile_creator_params.yaml')
    lanelet2_map_tile_creator_node = Node(
        package='lanelet2_map_tile_creator',
        executable='lanelet2_map_tile_creator_exe',
        parameters=[lanelet2_map_tile_creator_params]
    )

    return launch.LaunchDescription(
        [lanelet2_map_tile_creator_node])
