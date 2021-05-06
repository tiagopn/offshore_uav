cmake_minimum_required(VERSION 2.8.3)
project(offshore_uav_pack)

# We need erb to process the model erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

find_package(catkin REQUIRED COMPONENTS
  xacro
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wamv
  CATKIN_DEPENDS xacro std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Model files to be processed with erb
set (xacro_erb_files
  worlds/ocean_waves/model.xacro.erb
)
# Process the xacro erb files
foreach(_erb ${xacro_erb_files})
  string(REGEX REPLACE ".xacro.erb" ".xacro" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND model_files ${_model})
endforeach()

install(DIRECTORY launch Media worlds models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

add_custom_target(model_erb_generation ALL
  DEPENDS
    ${model_files}
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate world files from xacro and install
xacro_add_files(
  worlds/offshore_rig.world.xacro
  ${XACRO_INORDER} INSTALL DESTINATION worlds
)

# Necessario pra WAMV
add_library(wamv src/wamv.cpp)
add_dependencies(wamv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wamv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
