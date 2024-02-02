//
// Created by ataparlar on 01.02.2024.
//

#include "dynamic_lanelet2_config_metadata_creator_for_autoware/dynamic_lanelet2_config_metadata_creator_for_autoware.hpp"

#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "ogrsf_frmts.h"
#include "osmium/io/writer.hpp"
#include "proj.h"

#include <GeographicLib/MGRS.hpp>
#include <nlohmann/json.hpp>

#include <ogr_feature.h>

#include <fstream>

namespace dynamic_lanelet2_config_metadata_creator
{

DynamicLanelet2ConfigMetadataCreator::DynamicLanelet2ConfigMetadataCreator(
  const rclcpp::NodeOptions & options)
: Node("dynamic_lanelet2_config_metadata_creator", options)
{
  this->declare_parameter("mgrs_grid", "35TPF");
  this->declare_parameter("lanelet2_map_path", "/");
  //  this->declare_parameter("lanelet2_map_origin_lat", 0.0);
  //  this->declare_parameter("lanelet2_map_origin_lon", 0.0);
  //  this->declare_parameter("lanelet2_map_origin_alt", 0.0);
  mgrs_grid = this->get_parameter("mgrs_grid").as_string();
  lanelet2_map_path = this->get_parameter("lanelet2_map_path").as_string();
  //  lanelet2_map_origin_lat = this->get_parameter("lanelet2_map_origin_lat").as_double;
  //  lanelet2_map_origin_lon = this->get_parameter("lanelet2_map_origin_lon").as_double;
  //  lanelet2_map_origin_alt = this->get_parameter("lanelet2_map_origin_alt").as_double;

  // load lanelet2

  // find the point at most west, south, east and north. These are boundaries.

  // divide the 100km grids into 5km

  // find corresponding 5km grids with lanelet2 map.

  // note the polygon vertices.

  int zone, prec;
  bool northp;
  double x, y, origin_lat, origin_lon;
  GeographicLib::MGRS::Reverse(mgrs_grid, zone, northp, x, y, prec);
  GeographicLib::UTMUPS::Reverse(zone, northp, x, y, origin_lat, origin_lon);

  lanelet::projection::UtmProjector projector(
    lanelet::Origin({origin_lat, origin_lon}));  // we will go into details later
  lanelet::LaneletMapPtr map = load(lanelet2_map_path, projector);

  GDALAllRegister();
  GDALDataset * poDS;
  poDS = (GDALDataset *)GDALOpenEx(lanelet2_map_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }

  OGRLayer * line_layer = poDS->GetLayerByName("lines");

  OGRFeature * poFeature;
  while ((poFeature = line_layer->GetNextFeature()) != NULL) {
    OGRGeometry * geometry = poFeature->GetGeometryRef();
    std::cout << "geometry.getName(): " << geometry->getGeometryName() << std::endl;
//    geometry->Intersects()    REACH THE INTERSECTED GRID POLYGON WITH THIS
  }
  OGRFeature::DestroyFeature(poFeature);

  rclcpp::shutdown();
}

}  // namespace dynamic_lanelet2_config_metadata_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  dynamic_lanelet2_config_metadata_creator::DynamicLanelet2ConfigMetadataCreator)