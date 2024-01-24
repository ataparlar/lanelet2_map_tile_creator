//
// Created by ataparlar on 23.01.2024.
//

#include "dummy_lanelet2/dummy_lanelet2.hpp"

#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>

#include "osmium/io/reader.hpp"
#include "osmium/io/writer.hpp"


namespace dummy_lanelet2
{

DummyLanelet2::DummyLanelet2(const rclcpp::NodeOptions & options)
: Node("dummy_lanelet2", options)
{
  this->declare_parameter("origin_lat", 0.0);
  this->declare_parameter("origin_lon", 0.0);
  this->declare_parameter("origin_height", 0.0);
  this->declare_parameter("save_directory", "");

  origin_lat = this->get_parameter("origin_lat").as_double();
  origin_lon = this->get_parameter("origin_lon").as_double();
  origin_height = this->get_parameter("origin_height").as_double();
  save_directory = this->get_parameter("save_directory").as_string();

  lanelet::GPSPoint gps_point({origin_lat, origin_lon, origin_height});
  lanelet::projection::UtmProjector proj((lanelet::Origin(gps_point)));
  lanelet::LaneletMap map;


  lanelet::Lanelet lanelet;
  lanelet.setId(45);
  for (int j=0; j<2; j++) {
    lanelet::LineString3d ls;
    ls.setId(j);
    for (int i=0; i<=20; i++) {
      if (j==0) {
        lanelet::Point3d p1(lanelet::utils::getId(), 10, i*10, 10);
        ls.push_back(p1);
      } else if (j==1) {
        lanelet::Point3d p1(lanelet::utils::getId(), 12, i*10, 10);
        ls.push_back(p1);
      }
    }
    if (j==0) {
      lanelet.setLeftBound(ls);
    } else if (j==1) {
      lanelet.setRightBound(ls);
    }
  }

  map.add(lanelet);

  std::cout << map.pointLayer.size() << std::endl;
  std::cout << map.lineStringLayer.size() << std::endl;
  std::cout << map.laneletLayer.size() << std::endl;

  std::string save_string = save_directory + "lanelet2_map.osm";

  lanelet::write(save_string, map, proj);




  rclcpp::shutdown();
}

}  // namespace lanelet2_divider


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_lanelet2::DummyLanelet2)