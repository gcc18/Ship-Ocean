cmake_minimum_required(VERSION 3.10)
project(ship_demo)

#############################################################################
# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT /home/dell/data/GitDownload/AirSim)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)


set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")
#############################################################################

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(airsim_node src/airsim_ros2_wrapper/airsim_ros2_wrapper.cpp)
target_link_libraries(airsim_node ${ament_LIBRARIES} AirLib)
ament_target_dependencies(airsim_node  rclcpp  sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs image_transport cv_bridge)


add_executable(camera src/image_handler/camera_node.cpp)
target_link_libraries(camera ${ament_LIBRARIES})
ament_target_dependencies(camera rclcpp sensor_msgs image_transport cv_bridge)


add_executable(lidar src/lidar_handler/lidar_node.cpp)
target_link_libraries(lidar ${ament_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(lidar rclcpp sensor_msgs geometry_msgs pcl_conversions)


install(TARGETS
  airsim_node
  camera
  lidar
  DESTINATION lib/${PROJECT_NAME})

ament_package()
