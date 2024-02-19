import sys
import mgrs
import utm
from osgeo import gdal
from osgeo import ogr
import yaml
import json

if __name__ == "__main__":

    mgrs_grid = "35TPF"
    grid_edge_size = 50
    lanelet2_map_path = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/maps/lanelet2_map_sorted.osm"
    metadata_file_path = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/metadata.yaml"
    osmium_config_file_path = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/test_config.json"
    osmium_extract_dir = "/home/ataparlar/data/gis_data/lanelet2_map_tile_creator/test/"

    mgrs_object = mgrs.MGRS()
    zone, northp, y, x = mgrs_object.MGRSToUTM(mgrs_grid)
    print("x: ", x)
    print("y: ", y)
    print("zone: ", zone)
    print("northp: ", northp)

    # Create json
    # --------------------------------------------------------------------------------

    driverName = "GPKG"
    drv = gdal.GetDriverByName(driverName)
    if drv is None:
        print("%s driver not available.\n" % driverName)
        sys.exit(1)

    ds_grids = drv.Create(osmium_extract_dir + "output_layers.gpkg", 0, 0, 0, gdal.GDT_Unknown)
    if ds_grids is None:
        print("Creation of output file failed.\n")
        sys.exit(1)

    layer_grids = ds_grids.CreateLayer("grids", None, ogr.wkbPolygon)
    if layer_grids is None:
        print("Layer creation failed.\n")
        sys.exit(1)

    layer_lanelet2_whole = ds_grids.CreateLayer("lanelet2_whole", None, ogr.wkbMultiLineString)
    if layer_lanelet2_whole is None:
        print("Layer creation failed layer_lanelet2_whole.\n")
        sys.exit(1)

    # Create Grid Layer
    # --------------------------------------------------------------------------------
    grid_count = 100000 / grid_edge_size
    for i in range(int(grid_count)):
        print(i)
        for j in range(int(grid_count)):
            feature_grid = ogr.Feature(layer_grids.GetLayerDefn())

            linear_ring = ogr.Geometry(ogr.wkbLinearRing)

            for a in range(5):
                pt_x, pt_y = 0.0, 0.0
                if (a == 0) or (a == 4):
                    pt_x = x + (i * grid_edge_size)
                    pt_y = y + (j * grid_edge_size)
                elif a == 1:
                    pt_x = x + (i * grid_edge_size) + grid_edge_size
                    pt_y = y + (j * grid_edge_size)
                elif a == 2:
                    pt_x = x + (i * grid_edge_size) + grid_edge_size
                    pt_y = y + (j * grid_edge_size) + grid_edge_size
                elif a == 3:
                    pt_x = x + (i * grid_edge_size)
                    pt_y = y + (j * grid_edge_size) + grid_edge_size
                pt_lat, pt_lon = utm.to_latlon(pt_y, pt_x, zone, northp)
                linear_ring.AddPoint_2D(pt_lon, pt_lat)

            polygon = ogr.Geometry(ogr.wkbPolygon)
            polygon.AddGeometry(linear_ring)

            feature_grid.SetGeometry(polygon)
            if layer_grids.CreateFeature(feature_grid) != 0:
                print("Failed to create feature in shapefile.\n")
                sys.exit(1)

            feature_grid.Destroy()
    # --------------------------------------------------------------------------------

    # Create new layer for filtered grids
    # --------------------------------------------------------------------------------
    layer_filtered_grids = ds_grids.CreateLayer("filtered_grids", None, ogr.wkbPolygon)
    if layer_filtered_grids is None:
        print("Layer creation failed.\n")
        sys.exit(1)

    # Read Lanelet2 Map
    # --------------------------------------------------------------------------------
    ds_lanelet2 = gdal.OpenEx(lanelet2_map_path, gdal.OF_VECTOR)
    if ds_lanelet2 is None:
        print("Open failed.\n")
        sys.exit(1)

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

    feature_lanelet2_whole = ogr.Feature(layer_lanelet2_whole.GetLayerDefn())
    feature_lanelet2_whole.SetGeometry(lanelet2_whole_mls)
    if layer_lanelet2_whole.CreateFeature(feature_lanelet2_whole) != 0:
        print("Failed to create feature in shapefile.\n")
        sys.exit(1)

    # Filter and destroy feature
    # --------------------------------------------------------------------------------
    layer_grids.SetSpatialFilter(lanelet2_whole_mls)
    print("layer_grids.GetFeatureCount(): ", layer_grids.GetFeatureCount())
    feature_lanelet2_whole.Destroy()

    # Set filtered grid layer
    # --------------------------------------------------------------------------------
    for grid in layer_grids:
        geometry_grid = grid.GetGeometryRef()

        filtered_feature_grid = ogr.Feature(layer_filtered_grids.GetLayerDefn())
        filtered_feature_grid.SetGeometry(geometry_grid)
        if layer_filtered_grids.CreateFeature(filtered_feature_grid) != 0:
            print("Failed to create feature in shapefile.\n")
            sys.exit(1)

        filtered_feature_grid.Destroy()

    # Create yaml data and write to file
    # --------------------------------------------------------------------------------
    metadata_yaml = {}
    for filtered_grid in layer_filtered_grids:
        geometry_filtered_grid = filtered_grid.GetGeometryRef()
        point_lat = 0.0
        point_lon = 0.0
        for linearring in geometry_filtered_grid:
            point_lat = linearring.GetPoint(0)[0]
            point_lon = linearring.GetPoint(0)[1]
        file_id = str(filtered_grid.GetFID()) + ".osm"
        yaml_data = {
            file_id: {
                "mgrs_code": mgrs_grid,
                "origin_lat": point_lat,
                "origin_lon": point_lon
            }
        }
        metadata_yaml.update(yaml_data)
    yaml_print = yaml.dump(metadata_yaml, sort_keys=False)

    with open(metadata_file_path, 'w', ) as f:
        yaml.dump(metadata_yaml, f, sort_keys=False)


    # Create config.json for Osmium Extract
    # --------------------------------------------------------------------------------
    extracts = []
    for filtered_grid in layer_filtered_grids:
        polygon_inner = []
        geometry_filtered_grid = filtered_grid.GetGeometryRef()
        print(geometry_filtered_grid)

        # print("geometry_grid.GetGeometryCount(): ", geometry_filtered_grid.GetGeometryCount())

        # test_polygon = ogr.Geometry(ogr.wkbPolygon)
        # test_polygon.AddPoint()

        counter = 0
        for linearring in geometry_filtered_grid:
            for i in range(5):
                point_lat = linearring.GetPoint(i)[1]
                point_lon = linearring.GetPoint(i)[0]
                point_list = [point_lon, point_lat]
                polygon_inner.append(point_list)



        polygon_outer = [polygon_inner]
        extract_element = {
            "description" : "optional description",
            "output" : str(filtered_grid.GetFID()) + ".osm",
            "output_format" : "osm",
            "polygon" : polygon_outer
        }
        extracts.append(extract_element)

    config_json_ = {
        "directory" : osmium_extract_dir,
        "extracts" : extracts
    }

    config_json = json.dumps(config_json_, indent=2)

    with open(osmium_config_file_path, "w") as write_file:
        write_file.write(config_json)








