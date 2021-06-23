#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

//include hpp from airsim
#include "airsim_settings_parser.hpp"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

//include hpp from stdlib of cpp
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <opencv2/opencv.hpp>
//include hpp from ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

//include custom msgs
#include "airsim_ros2_wrapper/msg/gps_yaw.hpp"
#include "airsim_ros2_wrapper/msg/altimeter.hpp"
#include "airsim_ros2_wrapper/msg/environment.hpp"



// todo move airlib typedefs to separate header file?
typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;
typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;

class AirsimROS2Wrapper:public rclcpp::Node{

public:
    AirsimROS2Wrapper(const std::string& host_ip);
    ~AirsimROS2Wrapper();
    void initialize_airsim();
    void initialize_ros2();
    
private:
    template<class msgType>
    struct SensorPublisher
    {
        SensorBase::SensorType sensor_type;
        std::string sensor_name;
        rclcpp::Publisher<msgType> publisher;
    };
    class VehicleROS
    {
        public: 
            VehicleROS();
            std::string vehicle_name;
            std::string odom_frame_id;

            rclcpp::Publisher<airsim_ros2_wrapper::msg::Environment>::SharedPtr env_pub;
            rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_gps_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_local_pub;
            std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_vec;
            
            //sensor pubs
            std::vector<struct SensorPublisher<airsim_ros2_wrapper::msg::Altimeter>> alt_pubs;
            std::vector<struct SensorPublisher<sensor_msgs::msg::Imu>> imu_pubs;
            std::vector<struct SensorPublisher<sensor_msgs::msg::NavSatFix>> gps_pubs;
            std::vector<struct SensorPublisher<sensor_msgs::msg::MagneticField>> mag_pubs;
            std::vector<struct SensorPublisher<sensor_msgs::msg::Range>> dist_pubs;
            std::vector<struct SensorPublisher<sensor_msgs::msg::PointCloud2>> lidar_pubs;

            std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs;
            std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs;
            std::vector<image_transport::Publisher> image_pubs;
            std::vector<ImageRequest> image_requests;
    };


private:

    // methods which parse setting json and generate ros pubs
    void create_ros2_pubs_from_setting_json();
    void append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting);
    void append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const LidarSetting& lidar_setting);
    void append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting);
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const;
    
    //callback function of timer
    void state_timer_callback();
    void image_timer_callback();
    void lidar_timer_callback();
    
    //time cast between airsim and ros2
    rclcpp::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;
    rclcpp::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    
    
    /// utils. todo parse into an Airlib<->ROS conversion class

    nav_msgs::msg::Odometry get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const;
    msr::airlib::Pose get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const;
    airsim_ros2_wrapper::msg::GPSYaw get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::msg::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const;
    airsim_ros2_wrapper::msg::Altimeter get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const;
    sensor_msgs::msg::Range get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const;
    sensor_msgs::msg::PointCloud2 get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name) const;
    sensor_msgs::msg::NavSatFix get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const;
    sensor_msgs::msg::MagneticField get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const;
    airsim_ros2_wrapper::msg::Environment get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const;
    
    /// camera helper methods
    sensor_msgs::msg::CameraInfo generate_cam_info(const std::string& camera_name, const CameraSetting& camera_setting, const CaptureSetting& capture_setting) const;
    cv::Mat manual_decode_depth(const ImageResponse& img_response) const;
    sensor_msgs::msg::Image * get_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);
    sensor_msgs::msg::Image * get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id);
    
    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const VehicleROS& vehicle_ros);
    



private:
    bool is_vulkan_;
    std::string host_ip_;
    msr::airlib::RpcLibClientBase airsim_client_;
    msr::airlib::RpcLibClientBase airsim_client_images_;
    msr::airlib::RpcLibClientBase airsim_client_lidar_;

    rclcpp::Publisher<airsim_ros2_wrapper::msg::GPSYaw>::SharedPtr origin_geo_point_pub_; // home geo coord of drones    
    msr::airlib::GeoPoint origin_geo_point_;// gps coord of unreal origin 
    airsim_ros2_wrapper::msg::GPSYaw origin_geo_point_msg_; // todo duplicate

    AirSimSettingsParser airsim_settings_parser_;
    std::unordered_map< std::string, std::unique_ptr< VehicleROS > > vehicle_name_ptr_map_;
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;
    /// ROS2 tf
    const std::string AIRSIM_FRAME_ID = "world_ned";
    const std::string AIRSIM_ODOM_FRAME_ID = "odom_local_ned";
    const std::string ENU_ODOM_FRAME_ID = "odom_local_enu";
    std::string world_frame_id_ = AIRSIM_FRAME_ID;
    std::string odom_frame_id_ = AIRSIM_ODOM_FRAME_ID;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;

    bool isENU_ = false;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /// ROS other publishers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rosgraph_msgs::msg::Clock ros_clock_;
    bool publish_clock_ = false;

    rclcpp::TimerBase::SharedPtr airsim_state_timer_;
    rclcpp::TimerBase::SharedPtr airsim_img_timer_;
    rclcpp::TimerBase::SharedPtr airsim_lidar_timer_;
    
};