cmake_minimum_required(VERSION 2.8.3)
project(ewok_collision)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(SuiteSparse REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})

catkin_simple()

#cs_add_executable(collision_avoid src/collision_avoid.cpp)
#target_link_libraries(collision_avoid ${OCTOMAP_LIBRARIES})

cs_install()
cs_export()
