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

  //  lanelet::projection::UtmProjector projector(
  //    lanelet::Origin({origin_lat, origin_lon}));  // we will go into details later
  //  lanelet::LaneletMapPtr map = load(lanelet2_map_path, projector);

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


//    if (counter % 2 == 0) {
//      OGRFeature * lanelet2_layer_poly_feature;
//      lanelet2_layer_poly_feature = OGRFeature::CreateFeature(lanelet2_layer_poly->GetLayerDefn());
//      lanelet2_layer_poly_feature->SetField("Number", counter);
//
//      OGRLinearRing ring_lanelet2_poly;
//      OGRPolygon polygon_lanelet2;
//
//      bool first_point = true;
//      OGRPoint first_point_in_ls;
//
//      for (const OGRPoint & point_lanelet2 : linestring_lanelet2) {
//        if (first_point) {
//          first_point_in_ls.setX(point_lanelet2.getX());
//          first_point_in_ls.setY(point_lanelet2.getY());
//
//          first_point = false;
//        }
//
//        OGRPoint pt;
//        pt.setX(point_lanelet2.getX());
//        pt.setY(point_lanelet2.getY());
//
//        ring_lanelet2_poly.addPoint(&pt);
//      }
//      for (const OGRPoint & point_to_combine : ls_to_combine) {
//        OGRPoint pt;
//        pt.setX(point_to_combine.getX());
//        pt.setY(point_to_combine.getY());
//
//        ring_lanelet2_poly.addPoint(&pt);
//      }
//
//      ring_lanelet2_poly.addPoint(&first_point_in_ls);
//
//      polygon_lanelet2.addRing(&ring_lanelet2_poly);
//      lanelet2_layer_poly_feature->SetGeometry(&polygon_lanelet2);
//
//      if (lanelet2_layer_poly->CreateFeature(lanelet2_layer_poly_feature) != OGRERR_NONE) {
//        printf("Failed to create feature in shapefile.\n");
//        exit(1);
//      }
//
//      for (auto & gridFeature : gridLayer) {
//        OGRGeometry * grid_geometry = gridFeature->GetGeometryRef();
//        OGRGeometry * lanelet2_geom_polygon = lanelet2_layer_poly_feature->GetGeometryRef();
//
//        if (grid_geometry->Intersects(lanelet2_geom_polygon)) {
//          if (
//            std::find(grid_numbers.begin(), grid_numbers.end(), gridFeature->GetFID()) ==
//            grid_numbers.end()) {
//            grid_numbers.push_back(gridFeature->GetFID());
//            true_counter++;
//          }
//        } else {
//          false_counter++;
//        }
//      }
//
//      OGRFeature::DestroyFeature(lanelet2_layer_poly_feature);
//    } else if (counter % 2 == 1) {
//      ls_to_combine = linestring_lanelet2;
//      ls_to_combine->reversePoints();
//    }

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
  std::cout << "gridLayer size: " << gridLayer->GetFeatureCount() << std::endl;

  for (auto & feature : gridLayer) {
    OGRFeature * new_feature;
    new_feature = OGRFeature::CreateFeature(gridLayer->GetLayerDefn());
    new_feature->SetField("Number", feature->GetFID());

    new_feature->SetGeometry(feature->GetGeometryRef());

    if (new_gridLayer->CreateFeature(new_feature) != OGRERR_NONE) {
      printf("Failed to create lanelet2_feature in shapefile.\n");
      exit(1);
    }
  }

  OGRFeature::DestroyFeature(poFeature_lanelet2);


//  std::vector<long> remove_vec;
//  std::vector<long> whole_vec;
//  for (long i = 1; i <= gridLayer->GetFeatureCount(); i++) {
//    whole_vec.push_back(i);
//  }
//
//  remove_vec = vectorDifference(whole_vec, grid_numbers);
//
//  for (long grid_to_remove : remove_vec) {
//    gridLayer->DeleteFeature(grid_to_remove);
//  }


  rclcpp::shutdown();
}

}  // namespace lanelet2_map_tile_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_tile_creator::Lanelet2MapTileCreator)