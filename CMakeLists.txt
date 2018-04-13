cmake_minimum_required(VERSION 2.8)
project(whycon)

# set some flags
add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -g -O4 -march=native -Wfatal-errors")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O0 -g -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-configs")

# set some options
option(DISABLE_ROS "Do not build for ROS, but as standalone code" OFF)
option(ENABLE_FULL_UNDISTORT "Undistort the whole frame" OFF)
option(ENABLE_RANDOMIZED_THRESHOLD "Use rand() instead of binary-like search for threshold" OFF)
option(ENABLE_VERBOSE "Enable verbose console messages during detection" OFF)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_SOURCE_DIR}/include/whycon/config.h)

#### ROS CONFIGURATION ####
if(NOT DISABLE_ROS)
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    sensor_msgs
    std_msgs
    camera_info_manager
    message_generation
    geometry_msgs
    message_filters
    image_geometry
    image_transport
    angles
    visualization_msgs
    tf
    nodelet
    std_srvs
  )

  add_message_files(
    FILES
    Projection.msg
  )

  generate_messages(
    DEPENDENCIES geometry_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES whycon
    CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs message_runtime geometry_msgs
  )
endif()

### SYSTEM DEPENDENCIES ###
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version ${OpenCV_VERSION}")
find_package(Boost COMPONENTS program_options thread system REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)

### TARGETS ###
include_directories(
  ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include
)

add_library(whycon SHARED src/lib/circle_detector.cpp src/lib/many_circle_detector.cpp src/lib/localization_system.cpp)
target_link_libraries(whycon ${OpenCV_LIBS} ${Boost_LIBRARIES})

if(NOT DISABLE_ROS)
  add_executable(whycon-node src/ros/whycon_node.cpp src/ros/whycon_ros.cpp)
  set_target_properties(whycon-node PROPERTIES OUTPUT_NAME whycon)

  add_library(whycon_nodelet src/ros/whycon_nodelet.cpp src/ros/whycon_ros.cpp)

  add_executable(set_axis src/ros/set_axis_node.cpp src/ros/set_axis.cpp)
  #add_executable(triangulator src/ros/triangulator_node.cpp src/ros/triangulator.cpp)
  add_executable(robot_pose_publisher src/ros/robot_pose_publisher.cpp src/ros/robot_pose_publisher_node.cpp)  
  add_executable(transformer src/ros/transformer.cpp src/ros/transformer_node.cpp)

  add_dependencies(whycon-node whycon_generate_messages_cpp)
  add_dependencies(whycon_nodelet whycon_generate_messages_cpp)
  add_dependencies(set_axis whycon_generate_messages_cpp)
  #add_dependencies(triangulator whycon_generate_messages_cpp)
  add_dependencies(robot_pose_publisher whycon_generate_messages_cpp)
  add_dependencies(transformer whycon_generate_messages_cpp)

  target_link_libraries(whycon-node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)
  target_link_libraries(whycon_nodelet ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)
  target_link_libraries(set_axis ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)
  target_link_libraries(transformer ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)
  #target_link_libraries(triangulator ${catkin_LIBRARIES} whycon)
  target_link_libraries(robot_pose_publisher ${catkin_LIBRARIES} whycon)
else()
  add_executable(whycon-main src/main.cpp)
  set_target_properties(whycon-main PROPERTIES OUTPUT_NAME whycon)
  target_link_libraries(whycon-main whycon)

  add_executable(camera-calibrator src/camera_calibrator.cpp)
  target_link_libraries(camera-calibrator ${OpenCV_LIBS})
endif()

### INSTALL ###

### Mark cpp header files for installation
if(NOT DISABLE_ROS)
  install(DIRECTORY include/${PACKAGE_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  ## Mark executables and/or libraries for installation
  install(TARGETS whycon whycon-node robot_pose_publisher #triangulator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  ## Mark executables and/or libraries for installation
  install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
else()
  install(TARGETS whycon DESTINATION lib)
  install(TARGETS whycon-main camera-calibrator DESTINATION bin)
  install(DIRECTORY include/whycon DESTINATION include)
  install(FILES cmake-configs/FindWhyCon.cmake DESTINATION share/cmake/Modules)
endif()
