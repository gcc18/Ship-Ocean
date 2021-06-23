#include <rclcpp/rclcpp.hpp>
#include "airsim_settings_parser.hpp"
#include "airsim_ros2_wrapper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    std::string host_ip ="localhost";
    rclcpp::Node* node = new rclcpp::Node("airsim_init_node");
    node.get_parameter_or("host_ip",host_ip,host_ip);
    delete node;
    AirsimROS2Wrapper airsim_node(host_ip);    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(airsim_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}