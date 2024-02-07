//
// Created by ataparlar on 01.02.2024.
//

#ifndef BUILD_LANELET2_MAP_TILE_CREATOR_HPP
#define BUILD_LANELET2_MAP_TILE_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>


namespace lanelet2_map_tile_creator {


class Lanelet2MapTileCreator : public rclcpp::Node
{
public:
  explicit Lanelet2MapTileCreator(const rclcpp::NodeOptions & options);

  // params
  std::string mgrs_grid;
  std::string lanelet2_map_path;
//  double lanelet2_map_origin_lat;
//  double lanelet2_map_origin_lon;
//  double lanelet2_map_origin_alt;

private:


};
}

#endif  // BUILD_LANELET2_MAP_TILE_CREATOR_HPP
