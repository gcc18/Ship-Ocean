#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#define  MINIMUM_PIXELS_FOR_TARGET 5


class CameraNode :public rclcpp::Node{


public:
    CameraNode();

private:
    struct Params{
		int width = 256;
        int height = 144;
        double fov_hor =90.0/360.0*M_PI;
		double fov_vel =90.0/360.0*M_PI*144.0/256.0;
	};

private:

    void timer_callback();
    bool publish_target_location(cv::Mat& scene,cv::Mat& depth);


private:
    Params params_;
    cv_bridge::CvImagePtr scene_img_;
    cv_bridge::CvImagePtr depth_img_;
    image_transport::Subscriber scene_sub_;
    image_transport::Subscriber depth_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_position_pub_;
};