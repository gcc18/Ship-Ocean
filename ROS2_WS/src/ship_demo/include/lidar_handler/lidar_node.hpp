#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class LidarNode: public rclcpp::Node{

public:
	LidarNode();
	
private:

	bool publish_target_location(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
	bool segmentation_from_ocean(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered); 

private:

	//ros2
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_position_pub_;
	

};


