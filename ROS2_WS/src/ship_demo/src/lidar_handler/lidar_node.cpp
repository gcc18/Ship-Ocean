#include "lidar_handler/lidar_node.hpp"
using namespace std::chrono_literals;

LidarNode::LidarNode():Node("Lidarhandler"){
    lidar_sub_=this->create_subscription<sensor_msgs::msg::PointCloud2>("PX4/lidar",5,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            publish_target_location(msg);
        });
    target_position_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("target_position/camera",5);
}

bool LidarNode::publish_target_location(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud);
    std::cerr<<*cloud<<std::endl;
    segmentation_from_ocean(cloud,cloud_filtered);
    pcl::io::savePCDFileASCII("cloud.pcd",*cloud);
    pcl::io::savePCDFileASCII("cloud_filtered.pcd",*cloud_filtered);
    std::cerr<<*cloud_filtered<<std::endl;

   // pcl::visualization::CloudViewer viewer1("Cloud viewer");
   // viewer1.showCloud(cloud);
    return true;
}



bool LidarNode::segmentation_from_ocean(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered){

    pcl::PointIndicesPtr ocean(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ocean->indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ocean);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    return true;
}





















int main(int argc, char* argv[]) {
	std::cout << "Starting lidar node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarNode>());
	rclcpp::shutdown();
	return 0;
}