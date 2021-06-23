cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(implicit_shape_model)

find_package(PCL 1.10 REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (shape_model src/implicit_shape_model.cpp)
target_link_libraries (shape_model ${PCL_LIBRARIES})

add_executable (view src/viewer.cpp)
target_link_libraries (view ${PCL_LIBRARIES})

add_executable (alignment_prerejective src/alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})

add_executable (obj2pcd src/obj2pcd.cpp)
target_link_libraries (obj2pcd ${PCL_LIBRARIES})

