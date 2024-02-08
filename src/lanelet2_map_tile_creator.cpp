//
// Created by ataparlar on 01.02.2024.
//

#include "lanelet2_map_tile_creator/lanelet2_map_tile_creator.hpp"

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
  mgrs_grid = this->get_parameter("mgrs_grid").as_string();
  lanelet2_map_path = this->get_parameter("lanelet2_map_path").as_string();
  grid_edge_size = this->get_parameter("grid_edge_size").as_double();

  const char * pszDriverName = "GPKG";
  GDALDriver * poDriver;
  GDALAllRegister();
  poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
  if (poDriver == NULL) {
    printf("%s driver not available.\n", pszDriverName);
    exit(1);
  }
  GDALDataset * poDS;
  poDS = poDriver->Create(
    "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/polygon_out_x.gpkg", 0, 0, 0,
    GDT_Unknown, NULL);
  if (poDS == NULL) {
    printf("Creation of output file failed.\n");
    exit(1);
  }

  auto * sourceSRS = new OGRSpatialReference();
  sourceSRS->importFromEPSG(4326);
  char * pszWKT = NULL;
  sourceSRS->exportToWkt(&pszWKT);

  OGRLayer * gridLayer;
  gridLayer = poDS->CreateLayer("5km_grid", NULL, wkbPolygon, NULL);
  if (gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField("Number", OFTInteger);
  if (gridLayer->CreateField(&oField) != OGRERR_NONE) {
    printf("Creating Number field failed.\n");
    exit(1);
  }
  std::cout << "grid layer created" << std::endl;

  int zone, prec;
  bool northp;
  double origin_x, origin_y, origin_lat, origin_lon, x, y;
  GeographicLib::MGRS::Reverse(mgrs_grid, zone, northp, origin_x, origin_y, prec);
  GeographicLib::UTMUPS::Reverse(zone, northp, origin_x, origin_y, origin_lat, origin_lon);
  x = origin_x - 50000;
  y = origin_y - 50000;

  nlohmann::json extracts = nlohmann::json::array();
  int number = 1;
  double grid_count = 100000 / grid_edge_size;
  for (int i = 0; i < grid_count; i++) {
    for (int j = 0; j < grid_count; j++) {
      OGRFeature * gridFeature;

      gridFeature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());
      gridFeature->SetField("Number", number);

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
      number++;
      OGRFeature::DestroyFeature(gridFeature);
    }
  }

  auto * sourceSRS2 = new OGRSpatialReference();
  sourceSRS2->importFromEPSG(4326);
  char * pszWKT2 = NULL;
  sourceSRS2->exportToWkt(&pszWKT2);

  OGRLayer * lanelet2_layer;
  lanelet2_layer = poDS->CreateLayer("lanelet2_layer", nullptr, wkbLineString, nullptr);
  if (lanelet2_layer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField_lanelet2("Number", OFTInteger);
  if (lanelet2_layer->CreateField(&oField_lanelet2) != OGRERR_NONE) {
    printf("Creating Number field for lanelet2 failed.\n");
    exit(1);
  }

  OGRLayer * new_gridLayer;
  new_gridLayer = poDS->CreateLayer("new_grid_layer", nullptr, wkbPolygon, nullptr);
  if (new_gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField_new_gridLayer("Number", OFTInteger);
  if (new_gridLayer->CreateField(&oField_new_gridLayer) != OGRERR_NONE) {
    printf("Creating Number field for lanelet2 failed.\n");
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
  OGRFeature * poFeature_lanelet2;

  std::vector<long> grid_numbers;
  int counter = 1;

  OGRLineString linestring_lanelet2_whole;

  //  OGRLineString * ls_to_combine;
  while ((poFeature_lanelet2 = line_layer_lanelet2->GetNextFeature()) != NULL) {
    OGRGeometry * geometry_lanelet2 = poFeature_lanelet2->GetGeometryRef();
    OGRLineString * linestring_lanelet2 = geometry_lanelet2->toLineString();

    for (const OGRPoint & point_lanelet2 : linestring_lanelet2) {
      OGRPoint new_pt;
      new_pt.setX(point_lanelet2.getX());
      new_pt.setY(point_lanelet2.getY());

      linestring_lanelet2_whole.addPoint(&new_pt);
    }

    // lanelet2 linestring part

    OGRLineString * linestring = geometry_lanelet2->toLineString();

    OGRFeature * lanelet2_feature;
    lanelet2_feature = OGRFeature::CreateFeature(lanelet2_layer->GetLayerDefn());
    lanelet2_feature->SetField("Number", counter);

    OGRLineString lanelet2_linestring;

    for (const OGRPoint & point : linestring) {
      OGRPoint point_lanelet2;
      point_lanelet2.setX(point.getX());
      point_lanelet2.setY(point.getY());

      lanelet2_linestring.addPoint(&point_lanelet2);
    }
    lanelet2_feature->SetGeometry(&lanelet2_linestring);
    if (lanelet2_layer->CreateFeature(lanelet2_feature) != OGRERR_NONE) {
      printf("Failed to create lanelet2_feature in shapefile.\n");
      exit(1);
    }
    counter++;
    OGRFeature::DestroyFeature(lanelet2_feature);
  }

  gridLayer->SetSpatialFilter(&linestring_lanelet2_whole);

  std::ofstream metadata(
    "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/maps/tmp_lanelet2_map_metadata.yaml");

  for (auto & feature : gridLayer) {
    OGRFeature * new_feature;
    new_feature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());
    new_feature->SetField("Number", feature->GetFID());

    new_feature->SetGeometry(feature->GetGeometryRef());

    if (new_gridLayer->CreateFeature(new_feature) != OGRERR_NONE) {
      printf("Failed to create lanelet2_feature in shapefile.\n");
      exit(1);
    }

    nlohmann::json polygon_json = nlohmann::json::array({});
    nlohmann::json polygon_json2 = nlohmann::json::array({});

    std::string metadata_object;
    metadata_object += std::to_string(feature->GetFID()) + ".osm:\n";
    metadata_object += "\tid: " + std::to_string(feature->GetFID()) + "\n";
    metadata_object += "\tmgrs_code: \"" + mgrs_grid + "\"\n";

    OGRGeometry * geometry = feature->GetGeometryRef();
    OGRPolygon * polygon = geometry->toPolygon();
    //    for (OGRPolygon * polygon : multiPolygon) {
    bool first_point_of_polygon = true;
    for (OGRLinearRing * linearring : polygon) {
      for (const OGRPoint & point : linearring) {
        nlohmann::json point_json = nlohmann::json::array({point.getY(), point.getX()});
        polygon_json.push_back(point_json);

        if (first_point_of_polygon) {
          metadata_object += "\torigin_lat: " + std::to_string(point.getY()) + "\n";
          metadata_object += "\torigin_lon: " + std::to_string(point.getX()) + "\n";
          first_point_of_polygon = false;
        }
      }
    }

    metadata << metadata_object;

    polygon_json2.push_back(polygon_json);
    std::string map_name = "lanelet2_map_" + mgrs_grid + "_";
    map_name += std::to_string(feature->GetFID());
    map_name += ".osm";
    nlohmann::json extract({
      {"output", map_name},
      {"output_format", "osm"},
      {"description", "optional description"},
      {"polygon", polygon_json2},
    });
    extracts.push_back(extract);
  }

  OGRFeature::DestroyFeature(poFeature_lanelet2);

  nlohmann::json j;
  j["extracts"] = extracts;
  j["directory"] = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/maps/";
  std::ofstream o("/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/maps/test_config2.json");
  o << std::setw(4) << j << std::endl;

  rclcpp::shutdown();
}

}  // namespace lanelet2_map_tile_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_tile_creator::Lanelet2MapTileCreator)