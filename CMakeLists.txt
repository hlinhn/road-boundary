cmake_minimum_required(VERSION 3.1)
project(road_boundary)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_CATKIN_DEPS
    dynamic_reconfigure
    enway_helper
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    std_srvs
    std_msgs
    lanelet2_io
    lanelet2_matching
    lanelet2_projection
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED)

generate_dynamic_reconfigure_options(
  config/RoadBoundary.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPS}
  DEPENDS
    OpenCV
)

add_definitions(${catkin_DEFINITIONS})

set(PPL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../point_projection")
include_directories(
  include
  ${PPL_INCLUDE_DIRS}
  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src/${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

########################################################
# Installs
########################################################

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
