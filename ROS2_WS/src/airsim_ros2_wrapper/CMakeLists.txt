cmake_minimum_required(VERSION 3.10)
project(airsim_ros2_wrapper)

#############################################################################
# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT /home/dell/data/GitDownload/AirSim)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

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


#############################################################################
# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Altimeter.msg"
  "msg/Environment.msg"
  "msg/GPSYaw.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs 
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

ament_export_include_directories(include)
add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${ament_LIBRARIES} AirLib)

add_library(airsim_ros src/airsim_ros2_wrapper.cpp)
rosidl_target_interfaces(airsim_ros
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(airsim_ros rclcpp std_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs geometry_msgs geographic_msgs )
target_link_libraries(airsim_ros ${ament_LIBRARIES} ${OpenCV_LIBS} AirLib airsim_settings_parser)


add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros ${ament_LIBRARIES} AirLib)


install(TARGETS
    #list of nodes
    airsim_node
    DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
)


# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}

)
ament_package()
