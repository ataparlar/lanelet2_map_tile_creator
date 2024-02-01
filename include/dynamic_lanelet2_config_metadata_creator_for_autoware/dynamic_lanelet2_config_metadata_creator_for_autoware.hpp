//
// Created by ataparlar on 01.02.2024.
//

#ifndef BUILD_DYNAMIC_LANELET2_CONFIG_METADATA_CREATOR_FOR_AUTOWARE_HPP
#define BUILD_DYNAMIC_LANELET2_CONFIG_METADATA_CREATOR_FOR_AUTOWARE_HPP


#include <rclcpp/rclcpp.hpp>


namespace dynamic_lanelet2_config_metadata_creator {


class DynamicLanelet2ConfigMetadataCreator : public rclcpp::Node
{
public:
  explicit DynamicLanelet2ConfigMetadataCreator(const rclcpp::NodeOptions & options);

  // params
  std::string mgrs_grid;
  std::string lanelet2_map_path;
//  double lanelet2_map_origin_lat;
//  double lanelet2_map_origin_lon;
//  double lanelet2_map_origin_alt;

private:


};
}

#endif  // BUILD_DYNAMIC_LANELET2_CONFIG_METADATA_CREATOR_FOR_AUTOWARE_HPP
