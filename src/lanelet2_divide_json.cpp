//
// Created by ataparlar on 23.01.2024.
//

#include "lanelet2_divide_json/lanelet2_divide_json.hpp"

#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "ogrsf_frmts.h"
#include "osmium/io/reader.hpp"
#include "osmium/io/writer.hpp"

#include <nlohmann/json.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <ogr_feature.h>

#include <fstream>

namespace lanelet2_divide
{

Lanelet2DivideJson::Lanelet2DivideJson(const rclcpp::NodeOptions & options) : Node("lanelet2_divide_json", options)
{

  this->declare_parameter("save_directory", "");

  save_directory = this->get_parameter("save_directory").as_string();


  GDALAllRegister();

  GDALDataset * poDS;
  poDS = (GDALDataset *)GDALOpenEx(
    "/home/ataparlar/data/gis_data/mgrs_grids/UTM_Datum_WGS84_35T_polygons.gpkg", GDAL_OF_VECTOR,
    NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }

  // 10 km Grids
  OGRLayer * gridLayer_ = poDS->GetLayerByName("35T_10km");

  std::cout << "Count before filter:  " << gridLayer_->GetFeatureCount() << std::endl;
  gridLayer_->SetAttributeFilter("MGRS_10 LIKE '35TPF%'");
  std::cout << "Count after filter:  " << gridLayer_->GetFeatureCount() << std::endl;

  nlohmann::json extracts = nlohmann::json::array();
  for (auto & feat : gridLayer_) {
    nlohmann::json polygon_json = nlohmann::json::array({});
    nlohmann::json polygon_json2 = nlohmann::json::array({});
    OGRGeometry * geometry = feat->GetGeometryRef();
    OGRMultiPolygon * multiPolygon = geometry->toMultiPolygon();
    for (OGRPolygon * polygon : multiPolygon) {
      for (OGRLinearRing * linearring : polygon) {
        for (const OGRPoint & point : linearring) {
          nlohmann::json point_json = nlohmann::json::array({point.getX(), point.getY()});
          polygon_json.push_back(point_json);
        }
      }
    }
    polygon_json2.push_back(polygon_json);
    std::string map_name = "lanelet2_map_";
    map_name +=  feat->GetFieldAsString("MGRS_10");
    map_name += ".pbf";
    nlohmann::json extract({
      {"output", map_name},
      {"output_format", "pbf"},
      {"description", "optional description"},
      {"polygon", polygon_json2},
    });
    extracts.push_back(extract);
  }

  nlohmann::json j;
  j["extracts"] = extracts;
  j["directory"] = save_directory;


  std::ofstream o(save_directory + "test_config.json");
  o << std::setw(4) << j << std::endl;

  rclcpp::shutdown();
}


}  // namespace lanelet2_divide

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_divide::Lanelet2DivideJson)