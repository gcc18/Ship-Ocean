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

#include <px4_msgs/msg/timesync.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#define  MINIMUM_PIXELS_FOR_TARGET 5


typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

class ImageHandler: public rclcpp::Node{

public:
	ImageHandler(const std::string& host_ip);
private:
	struct Params{
		unsigned int width = 256;
        unsigned int height = 144;
        msr::airlib::real_T fov_hor =90.0/360.0*M_PI;
		msr::airlib::real_T fov_vel;
		sensor_msgs::msg::CameraInfo info_msg;
	};
	std::vector<ImageRequest> requests_{
		ImageRequest("front_center",ImageType::Scene),
		ImageRequest("front_center",ImageType::DepthPerspective,true)
	};
	
private:
	void initialize_params();
	void initialize_ros2();

	void timer_callback();
	bool get_image_from_airsim(cv::Mat &scene, cv::Mat &depth,geometry_msgs::msg::Pose &pose);
	bool publish_target_location_from_image(cv::Mat &scene, cv::Mat &depth,geometry_msgs::msg::Pose &pose);
	void publish_scene_image(const ImageResponse img_response);
	void publish_camera_tf(const ImageResponse img_response);

	rclcpp::Time px4_timestamp_to_ros(const std::atomic<uint64_t>& stamp);


private:
	//util
	Params params_;
	std::atomic<uint64_t> px4_timestamp_;
    //airsim
	std::vector<ImageResponse> responses_;
	msr::airlib::RpcLibClientBase airsim_client_;
	//ros2
	image_transport::Publisher scene_image_pub_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_position_pub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr px4_timesync_sub_;
	

};
