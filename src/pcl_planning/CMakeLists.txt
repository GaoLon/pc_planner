cmake_minimum_required(VERSION 3.0.2)
project(pcl_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules 
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GSL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS PCL Eigen3
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations




include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIR})
include_directories("/usr/include/pcl-1.8")

link_directories(${PCL_LIBRARY_DIRS} ${GSL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(LIBS ${LIBS} ${catkin_LIBRARIES})

file(GLOB NODE_SRC_FILES src/*.cpp src/*.c)
#   foreach(_node_file ${NODE_SRC_FILES})
#       get_filename_component(_node_name ${_node_file} NAME_WE)
#       # add_executable(${_node_name} ${_node_file})
#       add_executable(${_node_name} ${NODE_SRC_FILES})
#       target_link_libraries(${_node_name} ${LIBS})
#   endforeach()

add_executable(pcl_planner ${NODE_SRC_FILES})
target_link_libraries(pcl_planner ${LIBS} ${GSL_LIBRARIES} ${PCL_LIBRARIES}) 

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcl_planning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pcl_planning_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
