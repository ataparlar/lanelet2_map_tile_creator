//
// Created by ataparlar on 25.01.2024.
//

#ifndef BUILD_LANELET2_DIVIDE_JSON_HPP
#define BUILD_LANELET2_DIVIDE_JSON_HPP


#include <rclcpp/rclcpp.hpp>


namespace lanelet2_divide {


class Lanelet2DivideJson : public rclcpp::Node
{
public:
  explicit Lanelet2DivideJson(const rclcpp::NodeOptions & options);

  // params
//  double origin_lat;
//  double origin_lon;
//  double origin_height;
  std::string save_directory;

private:


  //  std::string extract_creator(int n, int length, float lat, float lon);


};
}



#endif  // BUILD_LANELET2_DIVIDE_JSON_HPP
