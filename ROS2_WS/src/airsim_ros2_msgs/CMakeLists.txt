cmake_minimum_required(VERSION 3.5)
project(airsim_ros2_msgs)


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




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Altimeter.msg"
  "msg/Environment.msg"
  "msg/GPSYaw.msg"
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs
 )
ament_export_dependencies(rosidl_default_runtime)


add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${ament_LIBRARIES} AirLib)

add_library(airsim_ros src/airsim_ros2_wrapper.cpp)
target_link_libraries(airsim_ros ${ament_LIBRARIES} AirLib airsim_settings_parser)
rosidl_target_interfaces(airsim_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(airsim_ros rclcpp std_msgs geometry_msgs geographic_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs rosgraph_msgs image_transport cv_bridge)



add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros)
ament_target_dependencies(airsim_node rclcpp)


install(TARGETS 
  airsim_ros 
  airsim_settings_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  airsim_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
