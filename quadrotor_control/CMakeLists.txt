cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  roscpp
  sensor_msgs
  quadrotor_hardware_interface
)

include_directories(include
${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


###########
## Build ##
###########

add_library(pose_controller src/PoseController.cpp)
target_link_libraries(pose_controller ${catkin_LIBRARIES})

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
    INCLUDE_DIRS include
 #   LIBRARIES pid
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs hardware_interface controller_interface
    DEPENDS
)

