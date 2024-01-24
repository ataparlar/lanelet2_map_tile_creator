//
// Created by ataparlar on 23.01.2024.
//

#ifndef BUILD_DUMMY_LANELET2_HPP
#define BUILD_DUMMY_LANELET2_HPP


#include <rclcpp/rclcpp.hpp>


namespace dummy_lanelet2 {


class DummyLanelet2 : public rclcpp::Node
{
public:
  explicit DummyLanelet2(const rclcpp::NodeOptions & options);

  // params
  double origin_lat;
  double origin_lon;
  double origin_height;
  std::string save_directory;

private:

};
}



#endif  // BUILD_DUMMY_LANELET2_HPP
