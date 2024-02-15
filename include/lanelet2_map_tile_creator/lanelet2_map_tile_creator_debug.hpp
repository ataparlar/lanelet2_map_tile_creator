//
// Created by ataparlar on 01.02.2024.
//

#ifndef BUILD_LANELET2_MAP_TILE_CREATOR_DEBUG_HPP
#define BUILD_LANELET2_MAP_TILE_CREATOR_DEBUG_HPP

#include <rclcpp/rclcpp.hpp>


namespace lanelet2_map_tile_creator {


class Lanelet2MapTileCreatorDebug : public rclcpp::Node
{
public:
  explicit Lanelet2MapTileCreatorDebug(const rclcpp::NodeOptions & options);

  // params
  std::string mgrs_grid;
  std::string lanelet2_map_path;
  double grid_edge_size;
  std::string output_directory;

private:



  template<typename T>
  std::vector<T> vectorDifference(const std::vector<T> &v1, const std::vector<T> &v2){
    //Make the result initially equal to v1
    std::vector<T> result = v1;

    //Remove the elements in v2 from the result
    for(const T &element: v2){
      const auto it = std::find(result.begin(), result.end(), element);
      if(it != result.end()){
        result.erase(it);
      }
    }
    return result;
  }

};
}

#endif  // BUILD_LANELET2_MAP_TILE_CREATOR_DEBUG_HPP
