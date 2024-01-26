//
// Created by ataparlar on 23.01.2024.
//

#include "lanelet2_divide_json/lanelet2_divide_json.hpp"

#include "ogrsf_frmts.h"
#include "osmium/io/writer.hpp"
#include "proj.h"

#include <nlohmann/json.hpp>

#include <ogr_feature.h>

#include <fstream>

namespace lanelet2_divide
{

Lanelet2DivideJson::Lanelet2DivideJson(const rclcpp::NodeOptions & options)
: Node("lanelet2_divide_json", options)
{
  this->declare_parameter("save_directory", "");

  save_directory = this->get_parameter("save_directory").as_string();

  GDALAllRegister();

  GDALDataset * poDS;
  poDS = (GDALDataset *)GDALOpenEx(
    "//home/ataparlar/projects/qgis/lanelet2_divider_project/data/35t_100m_small.gpkg",
    GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }

  PJ_CONTEXT * C = proj_context_create();

  PJ * P = proj_create(C, "+proj=utm +zone=35 +datum=WGS84 +type=crs");
  PJ * G = proj_crs_get_geodetic_crs(C, P);
  PJ_AREA * A = NULL;
  const char * const * options_proj = NULL;
  PJ * G2P = proj_create_crs_to_crs_from_pj(C, G, P, A, options_proj);

  // 10 km Grids
  OGRLayer * gridLayer_ = poDS->GetLayerByName("grid");

  //  std::cout << "Count before filter:  " << gridLayer_->GetFeatureCount() << std::endl;
  //  gridLayer_->SetAttributeFilter("MGRS_10 LIKE '35TPF%'");
  //  std::cout << "Count after filter:  " << gridLayer_->GetFeatureCount() << std::endl;

  nlohmann::json extracts = nlohmann::json::array();
  size_t counter = 0;
  for (auto & feat : gridLayer_) {
    if (counter >= 500) {
      break;
    }
    nlohmann::json polygon_json = nlohmann::json::array({});
    nlohmann::json polygon_json2 = nlohmann::json::array({});
    OGRGeometry * geometry = feat->GetGeometryRef();
    //    std::cout << geometry->getGeometryName() << std::endl;
    //    OGRMultiPolygon * multiPolygon = geometry->toMultiPolygon();
    OGRPolygon * polygon = geometry->toPolygon();
    //    for (OGRPolygon * polygon : multiPolygon) {
    for (OGRLinearRing * linearring : polygon) {
      for (const OGRPoint & point : linearring) {
        PJ_COORD c_in;
        c_in.lpzt.z = 0.0;
        c_in.lpzt.t = HUGE_VAL;  // important only for time-dependent projections
        c_in.lp.lam = point.getX();
        c_in.lp.phi = point.getY();
        //        std::cout << "X: " << std::setprecision(12) << c_in.lp.phi <<  std::endl;
        //        std::cout << "Y: " << std::setprecision(12) << c_in.lp.lam <<  std::endl;
        PJ_COORD c_out = proj_trans(G2P, PJ_INV, c_in);
        //        std::cout << "lat: " << std::setprecision(12) << c_out.enu.n <<  std::endl;
        //        std::cout << "lon: " << std::setprecision(12) << c_out.enu.e << "\n" << std::endl;

        nlohmann::json point_json = nlohmann::json::array({c_out.enu.e, c_out.enu.n});
        polygon_json.push_back(point_json);
      }
    }
    counter++;
    //    }
    polygon_json2.push_back(polygon_json);
    std::string map_name = "lanelet2_map_35TPF_";
    map_name += feat->GetFieldAsString("id");
    //        map_name +=  std::to_string(counter);
    map_name += ".osm";
    nlohmann::json extract({
      {"output", map_name},
      {"output_format", "osm"},
      {"description", "optional description"},
      {"polygon", polygon_json2},
    });
    extracts.push_back(extract);
  }

  nlohmann::json j;
  j["extracts"] = extracts;
  j["directory"] = save_directory;
    
  std::ofstream o(save_directory + "test_config2.json");
  o << std::setw(4) << j << std::endl;

  /* Clean up */
  proj_destroy(P);
  proj_destroy(G);
  proj_destroy(G2P);
  proj_context_destroy(C); /* may be omitted in the single threaded case */

  rclcpp::shutdown();
}

}  // namespace lanelet2_divide

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_divide::Lanelet2DivideJson)