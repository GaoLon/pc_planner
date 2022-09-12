cmake_minimum_required(VERSION 3.0.2)
project(visualization_tools)
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
#set(BUILD_STATIC_LIBS ON)
#set(BUILD_SHARED_LIBS OFF)
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
message(Boost_LIBRARIES: ${Boost_LIBRARIES})
message(BOOST_VERSION: ${Boost_VERSION})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
  /usr/lib/x86_64-linux-gnu
)

## Specify additional locations for library files
link_directories(
 # /usr/local/lib # Location when using 'make system_install'
#很烦，老是报/usr/local/lib/libpcl_io.so：对‘boost::this_thread::hiden::sleep_until(timespec const&)’未定义的引用,可能是有boost版本库冲突的问题？不知道怎么解决，直接不链接libpcl_io不知道行不行
  /usr/lib       # More usual location (e.g. when installing using a package)
  /usr/lib/x86_64-linux-gnu
)

## Declare executables
add_executable(visualizationTools src/visualizationTools.cpp)
target_include_directories(visualizationTools PRIVATE ${Boost_INCLUDE_DIRS})
## Specify libraries to link a library or executable target against
target_link_libraries(visualizationTools ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(visualizationTools ${Boost_LIBRARIES})
install(TARGETS visualizationTools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(PROGRAMS scripts/realTimePlot.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

