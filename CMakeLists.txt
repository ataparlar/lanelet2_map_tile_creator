cmake_minimum_required(VERSION 3.5)
project(lanelet2_map_tile_creator)

add_compile_options(-Wall -Wextra -Wpedantic)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(lanelet2 REQUIRED)
find_package(GDAL REQUIRED)
find_package(Osmium REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PROJ REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(include
        ${OSMIUM_INCLUDE_DIRS}
        ${GDAL_INCLUDE_DIRS}
        ${NLOHMANN_JSON_INCLUDE_DIRS}
        ${PROJ_INCLUDE_DIRS}
        ${lanelet2_INCLUDE_DIRS})

set(DUMMY_LANELET2_LIB_SRC
        src/lanelet2_map_tile_creator.cpp)

set(DUMMY_LANELET2_LIB_HEADERS
        include/lanelet2_map_tile_creator/lanelet2_map_tile_creator.hpp)


ament_auto_add_library(${PROJECT_NAME} SHARED
        ${DUMMY_LANELET2_LIB_HEADERS}
        ${DUMMY_LANELET2_LIB_SRC})
target_link_libraries(${PROJECT_NAME}
        ${OSMIUM_LIBRARIES}
        ${GDAL_LIBRARIES}
        ${NLOHMANN_JSON_LIBRARIES}
        PROJ::proj
        ${GeographicLib_LIBRARIES}
        ${lanelet2_LIBRARIES})

#target_compile_options(${PROJECT_NAME} PRIVATE
#  -Wall
#  -Werror
#  -Wextra
#  #-Wshadow             # causes issues with ROS 2 headers
#  #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
#  -pedantic
#  -Wcast-align
#  -Wunused
#  -Wconversion
#  -Wsign-conversion
#  -Wdouble-promotion
#  #-Wnull-dereference    # gcc6
#  #-Wduplicated-branches # gcc7
#  #-Wduplicated-cond     # gcc6
#  #-Wrestrict            # gcc7
#  -fvisibility=hidden)
# C++-only options

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "lanelet2_map_tile_creator::Lanelet2MapTileCreator"
        EXECUTABLE lanelet2_map_tile_creator_exe)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries({${PROJECT_NAME}_node})

ament_auto_package()