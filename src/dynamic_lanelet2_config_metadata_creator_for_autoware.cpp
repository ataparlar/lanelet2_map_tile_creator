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

  // create the map tile grids within an MGRS grid

  // load lanelet2

  // find intersected grids with lanelet2 map

  // note the polygon vertices.

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

  // Create a temp shp for identifying grids.
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

  OGRSpatialReference* oSRS = new OGRSpatialReference();
  std::string epsg_string = "32";
  if (northp) {
    epsg_string += "6";
  } else {
    epsg_string += "7";
  }
  epsg_string += std::to_string(zone);
  oSRS->importFromEPSG(std::stoi(epsg_string));
  char *pszWKT = NULL;
  oSRS->exportToWkt( &pszWKT );

  OGRLayer * gridLayer;
  gridLayer = poDS->CreateLayer("5km_grid", oSRS, wkbPolygon, &pszWKT);
  if (gridLayer == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }
  OGRFieldDefn oField("Number", OFTInteger);

  if (gridLayer->CreateField(&oField) != OGRERR_NONE) {
    printf("Creating Number field failed.\n");
    exit(1);
  }

  int number = 1;
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 20; j++) {
      OGRFeature * gridFeature;

      gridFeature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());
      gridFeature->SetField("Number", number);

      OGRPoint pt1;
      pt1.setX(x + (i * 5000));
      pt1.setY(y + (j * 5000));
      OGRPoint pt2;
      pt2.setX(x + (i * 5000) + 5000);
      pt2.setY(y + (j * 5000));
      OGRPoint pt3;
      pt3.setX(x + (i * 5000) + 5000);
      pt3.setY(y + (j * 5000) + 5000);
      OGRPoint pt4;
      pt4.setX(x + (i * 5000));
      pt4.setY(y + (j * 5000) + 5000);

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
  while ((poFeature_lanelet2 = line_layer_lanelet2->GetNextFeature()) != NULL) {
    OGRGeometry * geometry = poFeature_lanelet2->GetGeometryRef();
    //    geometry->Intersects()    REACH THE INTERSECTED GRID POLYGON WITH THIS
  }
  OGRFeature::DestroyFeature(poFeature_lanelet2);

  rclcpp::shutdown();
}

}  // namespace dynamic_lanelet2_config_metadata_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  dynamic_lanelet2_config_metadata_creator::DynamicLanelet2ConfigMetadataCreator)