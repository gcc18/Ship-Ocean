#include "airsim_ros2_msgs/airsim_ros2_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto airsim_node =std::make_shared<AirsimROS2Wrapper>("192.168.1.108");
    exec.add_node(airsim_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
