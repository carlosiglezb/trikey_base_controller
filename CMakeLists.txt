cmake_minimum_required(VERSION 3.0.2)
project(trikey_base_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
  realtime_tools
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controller_interface pluginlib roscpp realtime_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
        src/trikey_base_controller.cpp
        src/omniwheel_kinematics.cpp
        src/filters.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(${PROJECT_NAME}-test test/omniwheel_kinematics_test.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lib)
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
