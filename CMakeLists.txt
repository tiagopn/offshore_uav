cmake_minimum_required(VERSION 2.8.3)
project(offshore_uav)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  image_transport
  roscpp
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  mrs_lib
  mrs_msgs
  message_generation
  message_filters
  mavros_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES OffshoreUAV
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs mrs_msgs message_runtime message_filters
)

install(DIRECTORY launch Media worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Necessario pra fase um 
add_library(OffshoreUAV src/OffshoreUAV.cpp)
add_dependencies(OffshoreUAV ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OffshoreUAV ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
