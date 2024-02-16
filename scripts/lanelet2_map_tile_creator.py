import sys
import mgrs
import utm
from osgeo import gdal
from osgeo import ogr

if __name__ == "__main__":

    mgrs_grid = "35TPF"
    grid_edge_size = 5000
    lanelet2_map_path = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/maps/lanelet2_map_sorted.osm"
    osmium_config_file_path = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/metadata.yaml"
    metadata_file_path  = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/test_config.json"
    osmium_extract_dir = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/"


    mgrs_object = mgrs.MGRS()
    zone, northp, y, x = mgrs_object.MGRSToUTM(mgrs_grid)
    print("x: ", x)
    print("y: ", y)
    print("zone: ", zone)
    print("northp: ", northp)




    driverName = "GPKG"
    drv = gdal.GetDriverByName( driverName )
    if drv is None:
        print("%s driver not available.\n" % driverName)
        sys.exit( 1 )

    ds_grids = drv.Create(osmium_extract_dir + "output_layers.gpkg", 0, 0, 0, gdal.GDT_Unknown)
    if ds_grids is None:
        print ("Creation of output file failed.\n")
        sys.exit( 1 )

    layer_grids = ds_grids.CreateLayer("grids", None, ogr.wkbPolygon)
    if layer_grids is None:
        print("Layer creation failed.\n")
        sys.exit( 1 )



# Create Grid Layer
# --------------------------------------------------------------------------------
    grid_count = 100000 / grid_edge_size
    for i in range(int(grid_count)):
        for j in range(int(grid_count)):
            feature_grid = ogr.Feature(layer_grids.GetLayerDefn())

            linear_ring = ogr.Geometry(ogr.wkbLinearRing)

            for a in range(4):
                pt_x, pt_y = 0.0, 0.0
                if (a == 0) or (a == 4):
                    pt_x = x + (i * grid_edge_size)
                    pt_y = y + (j * grid_edge_size)
                elif a == 1:
                    pt_x = x + (i * grid_edge_size)  + grid_edge_size
                    pt_y = y + (j * grid_edge_size)
                elif a == 2:
                    pt_x = x + (i * grid_edge_size)  + grid_edge_size
                    pt_y = y + (j * grid_edge_size)  + grid_edge_size
                elif a == 3:
                    pt_x = x + (i * grid_edge_size)
                    pt_y = y + (j * grid_edge_size)  + grid_edge_size
                pt_lat, pt_lon = utm.to_latlon(pt_y, pt_x, zone, northp)
                linear_ring.AddPoint(pt_lon, pt_lat)

            polygon = ogr.Geometry(ogr.wkbPolygon)
            polygon.AddGeometry(linear_ring)

            feature_grid.SetGeometry(polygon)
            if layer_grids.CreateFeature(feature_grid) != 0:
                print("Failed to create feature in shapefile.\n")
                sys.exit( 1 )

            feature_grid.Destroy()
# --------------------------------------------------------------------------------



# Create new layer for filtered grids
# --------------------------------------------------------------------------------
    layer_filtered_grids = ds_grids.CreateLayer("filtered_grids", None, ogr.wkbPolygon)
    if layer_filtered_grids is None:
        print("Layer creation failed.\n")
        sys.exit( 1 )


# Read Lanelet2 Map
# --------------------------------------------------------------------------------
    ds_lanelet2 = gdal.OpenEx(lanelet2_map_path, gdal.OF_VECTOR)
    if ds_lanelet2 is None:
        print("Open failed.\n")
        sys.exit( 1 )

# Copy the lanelet2 lines layer to export layer
# --------------------------------------------------------------------------------
    ds_grids.CopyLayer(ds_lanelet2.GetLayerByName("lines"), "lanelet2_map")
    layer_lanelet2 = ds_grids.GetLayerByName("lanelet2_map")

# Make a multilinestring in order to use in filtering
# --------------------------------------------------------------------------------
    lanelet2_whole_mls = ogr.Geometry(ogr.wkbMultiLineString)
    for feature_lanelet2_linestring in layer_lanelet2:
        lanelet2_linestring = feature_lanelet2_linestring.GetGeometryRef()
        lanelet2_whole_mls.AddGeometry(lanelet2_linestring)

    print(lanelet2_whole_mls.GetBoundary())

# Filter
# --------------------------------------------------------------------------------
    layer_grids.SetSpatialFilter(lanelet2_whole_mls)
    print("layer_grids.GetFeatureCount(): ", layer_grids.GetFeatureCount())





