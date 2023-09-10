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
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED HINTS /usr/include/eigen3/Eigen)
include_directories(${EIGEN3_INCLUDE_DIRS})

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
        src/filters.cpp
        src/karnopp_compensator.cpp
        )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_odom_filter ${catkin_LIBRARIES})


###########
## Build ##
###########

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_odom_filter src/odom_filter.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_odom_filter PROPERTIES OUTPUT_NAME odom_filter PREFIX "")


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_odom_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


catkin_install_python(PROGRAMS 
nodes/odom_lpf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)



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
