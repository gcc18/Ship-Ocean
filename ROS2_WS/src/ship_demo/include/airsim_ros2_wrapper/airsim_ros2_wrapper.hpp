#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include <rpc/rpc_error.h>
STRICT_MODE_ON
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;
typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;



class AirsimROS2Wrapper:public rclcpp::Node
{
public:
    AirsimROS2Wrapper(const std::string& host_ip);
    void initialize_airsim();
    void initialize_ros2();

private:
    std::vector<ImageRequest> requests_{
		ImageRequest("front_center",ImageType::Scene),
		ImageRequest("front_center",ImageType::DepthPerspective,true)
	};

private:
 
    void image_timer_callback();
    void lidar_timer_callback();

    
    /// utils. todo parse into an Airlib<->ROS conversion class
    void process_and_publish_lidar_data(const msr::airlib::LidarData& lidar_data);
    void process_and_publish_img_response(const std::vector<ImageResponse>& img_responses);
    sensor_msgs::msg::Image get_scene_img_msg_from_response(const ImageResponse& img_response,const rclcpp::Time curr_ros_time, const std::string frame_id);
    sensor_msgs::msg::Image get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time,const std::string frame_id);
    void publish_lidar_tf(const msr::airlib::LidarData& lidar_data, const rclcpp::Time& ros_time);
    void publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time);


private:

    std::string host_ip_;
    msr::airlib::RpcLibClientBase airsim_client_image_;
    msr::airlib::RpcLibClientBase airsim_client_lidar_;
   
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr airsim_image_timer_;
    rclcpp::TimerBase::SharedPtr airsim_lidar_timer_;
    
    //sensor pubs
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    image_transport::Publisher scene_image_pub_;
    image_transport::Publisher depth_image_pub_;

};