//
// Created by ataparlar on 01.02.2024.
//

#include "lanelet2_map_tile_creator/lanelet2_map_tile_creator.hpp"

//#include "osmium/

#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "ogrsf_frmts.h"
#include "osmium/io/writer.hpp"
#include "proj.h"

#include <GeographicLib/MGRS.hpp>
#include <nlohmann/json.hpp>

#include <ogr_feature.h>

#include <fstream>

namespace lanelet2_map_tile_creator
{

Lanelet2MapTileCreator::Lanelet2MapTileCreator(const rclcpp::NodeOptions & options)
: Node("dynamic_lanelet2_config_metadata_creator", options)
{
  this->declare_parameter("mgrs_grid", "35TPF");
  this->declare_parameter("lanelet2_map_path", "/");
  this->declare_parameter("grid_edge_size", 5000.0);
  this->declare_parameter("osmium_config_file_path", "/");
  this->declare_parameter("metadata_file_path", "/");
  this->declare_parameter("osmium_extract_dir", "/");
  mgrs_grid = this->get_parameter("mgrs_grid").as_string();
  lanelet2_map_path = this->get_parameter("lanelet2_map_path").as_string();
  grid_edge_size = this->get_parameter("grid_edge_size").as_double();
  osmium_config_file_path = this->get_parameter("osmium_config_file_path").as_string();
  metadata_file_path = this->get_parameter("metadata_file_path").as_string();
  osmium_extract_dir = this->get_parameter("osmium_extract_dir").as_string();

  const char * pszDriverName = "GPKG";
  GDALDriver * poDriver;
  GDALAllRegister();
  poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
  if (poDriver == NULL) {
    printf("%s driver not available.\n", pszDriverName);
    exit(1);
  }


  std::string test = osmium_extract_dir + "output_layers.gpkg";
  GDALDataset * poDS;
  poDS = poDriver->Create(
    test.c_str(), 0, 0, 0,
    GDT_Unknown, NULL);
  if (poDS == NULL) {
    printf("Creation of output file failed.\n");
    exit(1);
  }

//  --------------------------------------------------------------------------------
  // create layer for all grids.
  OGRLayer * gridLayer;
  gridLayer = poDS->CreateLayer("grids", NULL, wkbPolygon, NULL);
  if (gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }

  int zone, prec;
  bool northp;
  double origin_x, origin_y, x, y;
  GeographicLib::MGRS::Reverse(mgrs_grid, zone, northp, origin_x, origin_y, prec);
  x = origin_x - 50000;
  y = origin_y - 50000;

  nlohmann::json extracts = nlohmann::json::array();
  double grid_count = 100000 / grid_edge_size;
  for (int i = 0; i < grid_count; i++) {
    for (int j = 0; j < grid_count; j++) {
      OGRFeature * gridFeature;

      gridFeature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());

      OGRPoint pt1;
      double pt1_x = x + (i * grid_edge_size);
      double pt1_y = y + (j * grid_edge_size);
      double pt1_lat;
      double pt1_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt1_x, pt1_y, pt1_lat, pt1_lon);
      pt1.setX(pt1_lon);
      pt1.setY(pt1_lat);

      OGRPoint pt2;
      double pt2_x = x + (i * grid_edge_size) + grid_edge_size;
      double pt2_y = y + (j * grid_edge_size);
      double pt2_lat;
      double pt2_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt2_x, pt2_y, pt2_lat, pt2_lon);
      pt2.setX(pt2_lon);
      pt2.setY(pt2_lat);

      OGRPoint pt3;
      double pt3_x = x + (i * grid_edge_size) + grid_edge_size;
      double pt3_y = y + (j * grid_edge_size) + grid_edge_size;
      double pt3_lat;
      double pt3_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt3_x, pt3_y, pt3_lat, pt3_lon);
      pt3.setX(pt3_lon);
      pt3.setY(pt3_lat);

      OGRPoint pt4;
      double pt4_x = x + (i * grid_edge_size);
      double pt4_y = y + (j * grid_edge_size) + grid_edge_size;
      double pt4_lat;
      double pt4_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt4_x, pt4_y, pt4_lat, pt4_lon);
      pt4.setX(pt4_lon);
      pt4.setY(pt4_lat);

      OGRPoint pt5;
      double pt5_x = x + (i * grid_edge_size);
      double pt5_y = y + (j * grid_edge_size);
      double pt5_lat;
      double pt5_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt5_x, pt5_y, pt5_lat, pt5_lon);
      pt5.setX(pt5_lon);
      pt5.setY(pt5_lat);

      OGRLinearRing ring;
      ring.addPoint(&pt1);
      ring.addPoint(&pt2);
      ring.addPoint(&pt3);
      ring.addPoint(&pt4);
      ring.addPoint(&pt5);

      OGRPolygon polygon;
      polygon.addRing(&ring);

      gridFeature->SetGeometry(&polygon);

      if (gridLayer->CreateFeature(gridFeature) != OGRERR_NONE) {
        printf("Failed to create feature in shapefile.\n");
        exit(1);
      }
      OGRFeature::DestroyFeature(gridFeature);
    }
  }

//  --------------------------------------------------------------------------------







  // create new layer for filtered grids
  OGRLayer * new_gridLayer;
  new_gridLayer = poDS->CreateLayer("new_grid_layer", nullptr, wkbPolygon, nullptr);
  if (new_gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }



  // Read lanelet2.osm
  GDALDataset * poDS_lanelet2;
  poDS_lanelet2 =
    (GDALDataset *)GDALOpenEx(lanelet2_map_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (poDS_lanelet2 == NULL) {
    printf("Open failed.\n");
    exit(1);
  }
  OGRLayer * line_layer_lanelet2 = poDS_lanelet2->GetLayerByName("lines");

  // copy the lanelet2 layer from .osm file to new .gpkg file
  poDS->CopyLayer(line_layer_lanelet2, "lanelet2_map");
  OGRLayer * lanelet2_layer = poDS->GetLayerByName("lanelet2_map");

  OGRFeature * poFeature_lanelet2;
  OGRMultiLineString linestring_lanelet2_whole;


  while ((poFeature_lanelet2 = lanelet2_layer->GetNextFeature()) != NULL) {
    OGRGeometry * geometry_lanelet2 = poFeature_lanelet2->GetGeometryRef();
    OGRLineString * linestring_lanelet2 = geometry_lanelet2->toLineString();

    linestring_lanelet2_whole.addGeometry(linestring_lanelet2);
//    linestring_lanelet2_whole.get
  }


  // filter the grids with lanelet2 map area
  gridLayer->SetSpatialFilter(&linestring_lanelet2_whole);
//  linestring_lanelet2_whole.getGeometryName()


  std::ofstream metadata(metadata_file_path);

  for (auto & feature : gridLayer) {
    OGRFeature * new_feature;
    new_feature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());

    new_feature->SetGeometry(feature->GetGeometryRef());

    if (new_gridLayer->CreateFeature(new_feature) != OGRERR_NONE) {
      printf("Failed to create lanelet2_feature in shapefile.\n");
      exit(1);
    }




//    for config.json and metadata.yaml
//    --------------------------------------------------------------------------------
    nlohmann::json polygon_json = nlohmann::json::array({});
    std::string metadata_object;
    metadata_object += std::to_string(feature->GetFID()) + ".osm:\n";
    metadata_object += "  id: " + std::to_string(feature->GetFID()) + "\n";
    metadata_object += "  mgrs_code: \"" + mgrs_grid + "\"\n";

    OGRGeometry * geometry = feature->GetGeometryRef();
    OGRPolygon * polygon = geometry->toPolygon();
    bool first_point_of_polygon = true;
    for (OGRLinearRing * linearring : polygon) {
      for (const OGRPoint & point : linearring) {
        nlohmann::json point_json = nlohmann::json::array({point.getX(), point.getY()});
        polygon_json.push_back(point_json);

        if (first_point_of_polygon) {
          metadata_object += "  origin_lat: " + std::to_string(point.getY()) + "\n";
          metadata_object += "  origin_lon: " + std::to_string(point.getX()) + "\n";
          first_point_of_polygon = false;
        }
      }
    }
    metadata << metadata_object;
    nlohmann::json polygon_json2 = nlohmann::json::array({});
    polygon_json2.push_back(polygon_json);
    std::string map_name = std::to_string(feature->GetFID()) + ".osm";
    nlohmann::json extract({
      {"output", map_name},
      {"output_format", "osm"},
      {"description", "optional description"},
      {"polygon", polygon_json2},
    });
    extracts.push_back(extract);
    //    --------------------------------------------------------------------------------


  }

  poDS->DeleteLayer(0);  // delete big gridLayer
  OGRFeature::DestroyFeature(poFeature_lanelet2);

  nlohmann::json j;
  j["extracts"] = extracts;
  j["directory"] = osmium_extract_dir;
  std::ofstream o(osmium_config_file_path);
  o << std::setw(4) << j << std::endl;

  rclcpp::shutdown();
}

}  // namespace lanelet2_map_tile_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_tile_creator::Lanelet2MapTileCreator)