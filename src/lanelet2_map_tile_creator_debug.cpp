//
// Created by ataparlar on 01.02.2024.
//

#include "lanelet2_map_tile_creator/lanelet2_map_tile_creator_debug.hpp"

// #include "osmium/

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

Lanelet2MapTileCreatorDebug::Lanelet2MapTileCreatorDebug(const rclcpp::NodeOptions & options)
: Node("dynamic_lanelet2_config_metadata_creator", options)
{
  this->declare_parameter("mgrs_grid", "35TPF");
  this->declare_parameter("lanelet2_map_path", "/");
  this->declare_parameter("grid_edge_size", 5000.0);
  this->declare_parameter("output_directory", "/");
  mgrs_grid = this->get_parameter("mgrs_grid").as_string();
  lanelet2_map_path = this->get_parameter("lanelet2_map_path").as_string();
  grid_edge_size = this->get_parameter("grid_edge_size").as_double();
  output_directory = this->get_parameter("output_directory").as_string();


  const char * pszDriverName = "GPKG";
  GDALDriver * poDriver;
  GDALAllRegister();
  poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
  if (poDriver == NULL) {
    printf("%s driver not available.\n", pszDriverName);
    exit(1);
  }


  // Read grids
  GDALDataset * ds_grids;
  ds_grids = (GDALDataset *)GDALOpenEx(
    "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/debug/grids_100m.gpkg", GDAL_OF_VECTOR, NULL,
    NULL, NULL);
  if (ds_grids == NULL) {
    printf("Open failed.\n");
    exit(1);
  }
  OGRLayer * gridLayer;
  gridLayer = ds_grids->GetLayerByName( "grids_100m" );
  std::cout << gridLayer->GetName() << " layer read" << std::endl;



  // Read lanelet2.osm
  GDALDataset * ds_lanelet2;
  ds_lanelet2 =
    (GDALDataset *)GDALOpenEx(lanelet2_map_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (ds_lanelet2 == NULL) {
    printf("Open failed.\n");
    exit(1);
  }
  OGRLayer * line_layer_lanelet2 = ds_lanelet2->GetLayerByName("lines");
  std::cout << line_layer_lanelet2->GetName() << " layer read" << std::endl;





  OGRFeature * feature_lanelet2;
  OGRLineString linestring_lanelet2_whole;
  while ((feature_lanelet2 = line_layer_lanelet2->GetNextFeature()) != NULL) {
    OGRGeometry * geometry_lanelet2 = feature_lanelet2->GetGeometryRef();
    OGRLineString * linestring_lanelet2 = geometry_lanelet2->toLineString();

    for (const OGRPoint & point_lanelet2 : linestring_lanelet2) {
      OGRPoint new_pt;
      new_pt.setX(point_lanelet2.getX());
      new_pt.setY(point_lanelet2.getY());

      linestring_lanelet2_whole.addPoint(&new_pt);
    }
  }

  gridLayer->SetSpatialFilter(&linestring_lanelet2_whole);
  std::cout << "grid_layer feature count: " << gridLayer->GetFeatureCount() << std::endl;






  GDALDataset * ds_whole_lanelet2;
  ds_whole_lanelet2 = poDriver->Create(
    "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/debug/whole_lanelet2.gpkg", 0, 0, 0,
    GDT_Unknown, NULL);
  if (ds_whole_lanelet2 == NULL) {
    printf("Creation of output file failed.\n");
    exit(1);
  }

  OGRLayer * layer_whole_lanelet2;
  layer_whole_lanelet2 = ds_whole_lanelet2->CreateLayer("whole_lanelet2", NULL, wkbLineString, NULL);
  if (layer_whole_lanelet2 == NULL) {
    printf("Layer creation failed.\n");
    exit(1);
  }

  OGRFeature * feature_whole_lanelet2;
  feature_whole_lanelet2 = OGRFeature::CreateFeature( layer_whole_lanelet2->GetLayerDefn() );
  feature_whole_lanelet2->SetGeometry(&linestring_lanelet2_whole);
  if( layer_whole_lanelet2->CreateFeature( feature_whole_lanelet2 ) != OGRERR_NONE )
  {
    printf( "Failed to create feature in shapefile.\n" );
    exit( 1 );
  }
  //  feature_whole_lanelet2 -> SetG







  int grid_counter = 0;
  for (auto & feature_grid : gridLayer) {
    OGRGeometry * geometry_grid = feature_grid->GetGeometryRef();
    OGRPolygon * polygon_grid = geometry_grid->toPolygon();

    if (polygon_grid->Intersects(&linestring_lanelet2_whole)){
      grid_counter++;
    }
  }
  std::cout << "intersected grid counter: " << grid_counter << std::endl;



//  while ((feature_lanelet2 = line_layer_lanelet2->GetNextFeature()) != NULL) {
//    OGRGeometry * geometry_lanelet2 = feature_lanelet2->GetGeometryRef();
//    OGRLineString * linestring_lanelet2 = geometry_lanelet2->toLineString();
//  }


  OGRFeature::DestroyFeature( feature_whole_lanelet2 );
  GDALClose( ds_whole_lanelet2 );

  rclcpp::shutdown();
}

}  // namespace lanelet2_map_tile_creator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_tile_creator::Lanelet2MapTileCreatorDebug)