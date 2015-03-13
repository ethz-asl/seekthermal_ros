cmake_minimum_required(VERSION 2.8.3)
project(seekthermal_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  camera_info_manager
  image_geometry
)

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SEEKTHERMAL libseekthermal) 
find_package(Boost REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thermal_inspection
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs camera_info_manager image_geometry
#  DEPENDS system_libs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SEEKTHERMAL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(seekthermal_ros
   src/seekthermal_ros.cpp
 )
 

 target_link_libraries(seekthermal_ros
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${SEEKTHERMAL_LIBRARIES}
   ${Boost_LIBRARIES}
 )

## Declare a cpp executable
 add_executable(seekthermal_ros_node src/seekthermal_ros_node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(seekthermal_ros_node
   seekthermal_ros
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

