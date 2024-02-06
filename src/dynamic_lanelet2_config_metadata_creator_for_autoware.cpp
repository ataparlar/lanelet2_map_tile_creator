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
  mgrs_grid = this->get_parameter("mgrs_grid").as_string();
  lanelet2_map_path = this->get_parameter("lanelet2_map_path").as_string();

  int zone, prec;
  bool northp;
  double origin_x, origin_y, origin_lat, origin_lon, x, y;
  GeographicLib::MGRS::Reverse(mgrs_grid, zone, northp, origin_x, origin_y, prec);
  GeographicLib::UTMUPS::Reverse(zone, northp, origin_x, origin_y, origin_lat, origin_lon);
  x = origin_x - 50000;
  y = origin_y - 50000;

  lanelet::projection::UtmProjector projector(
    lanelet::Origin({origin_lat, origin_lon}));  // we will go into details later
  lanelet::LaneletMapPtr map = load(lanelet2_map_path, projector);

  const char * pszDriverName = "GPKG";
  GDALDriver * poDriver;
  GDALAllRegister();
  poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
  if (poDriver == NULL) {
    printf("%s driver not available.\n", pszDriverName);
    exit(1);
  }
  GDALDataset * poDS;
  poDS = poDriver->Create("polygon_out.gpkg", 0, 0, 0, GDT_Unknown, NULL);
  if (poDS == NULL) {
    printf("Creation of output file failed.\n");
    exit(1);
  }

  auto* sourceSRS = new OGRSpatialReference();
  sourceSRS->importFromEPSG(4326);
  char *pszWKT = NULL;
  sourceSRS->exportToWkt( &pszWKT );

  OGRLayer * gridLayer;
  gridLayer = poDS->CreateLayer("5km_grid", sourceSRS, wkbPolygon, &pszWKT);
  if (gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField("Number", OFTInteger);
  if (gridLayer->CreateField(&oField) != OGRERR_NONE) {
    printf("Creating Number field failed.\n");
    exit(1);
  }

  OGRLayer * lanelet2_layer;
  lanelet2_layer = poDS->CreateLayer("lanelet2_layer", sourceSRS, wkbLineString, &pszWKT);
  if (lanelet2_layer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField_lanelet2("Number", OFTInteger);
  if (lanelet2_layer->CreateField(&oField_lanelet2) != OGRERR_NONE) {
    printf("Creating Number field for lanelet2 failed.\n");
    exit(1);
  }

  int number = 1;
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      OGRFeature * gridFeature;

      gridFeature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());
      gridFeature->SetField("Number", number);

      OGRPoint pt1;
      double pt1_x = x + (i * 5000);
      double pt1_y = y + (j * 5000);
      double pt1_lat;
      double pt1_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt1_x, pt1_y, pt1_lat, pt1_lon);
      pt1.setX(pt1_lon);
      pt1.setY(pt1_lat);

      OGRPoint pt2;
      double pt2_x = x + (i * 5000) + 5000;
      double pt2_y = y + (j * 5000);
      double pt2_lat;
      double pt2_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt2_x, pt2_y, pt2_lat, pt2_lon);
      pt2.setX(pt2_lon);
      pt2.setY(pt2_lat);

      OGRPoint pt3;
      double pt3_x = x + (i * 5000) + 5000;
      double pt3_y = y + (j * 5000) + 5000;
      double pt3_lat;
      double pt3_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt3_x, pt3_y, pt3_lat, pt3_lon);
      pt3.setX(pt3_lon);
      pt3.setY(pt3_lat);

      OGRPoint pt4;
      double pt4_x = x + (i * 5000);
      double pt4_y = y + (j * 5000) + 5000;
      double pt4_lat;
      double pt4_lon;
      GeographicLib::UTMUPS::Reverse(zone, northp, pt4_x, pt4_y, pt4_lat, pt4_lon);
      pt4.setX(pt4_lon);
      pt4.setY(pt4_lat);
      
      OGRLinearRing ring;
      ring.addPoint(&pt1);
      ring.addPoint(&pt2);
      ring.addPoint(&pt3);
      ring.addPoint(&pt4);

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


//  poDS_lanelet2->SetSpatialRef(targetSRS);
//  std::cout << poDS->GetSpatialRef()->GetName() << std::endl;




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


  std::vector<int> grid_numbers;
  int counter = 0;
  size_t true_counter = 0;
  size_t false_counter = 0;
  while ((poFeature_lanelet2 = line_layer_lanelet2->GetNextFeature()) != NULL) {
    OGRGeometry * geometry_lanelet2 = poFeature_lanelet2->GetGeometryRef();
    
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
  std::cout << grid_numbers.size() << std::endl;
  std::cout << "true_counter: " << true_counter << std::endl;
  std::cout << "false_counter: " << false_counter << std::endl;
  OGRFeature::DestroyFeature(poFeature_lanelet2);

  rclcpp::shutdown();
}

}  // namespace dynamic_lanelet2_config_metadata_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  dynamic_lanelet2_config_metadata_creator::DynamicLanelet2ConfigMetadataCreator)