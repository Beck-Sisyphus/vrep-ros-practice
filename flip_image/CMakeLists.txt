cmake_minimum_required(VERSION 2.8.3)
project(flip_image)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONTPARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_flip
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/flip_image_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
