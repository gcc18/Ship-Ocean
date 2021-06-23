#include "airsim_ros2_wrapper/airsim_ros2_wrapper.hpp"

using namespace std::chrono_literals;

AirsimROS2Wrapper::AirsimROS2Wrapper(const std::string& host_ip):
    Node("AirsimROS2Wrapper"),
    host_ip_(host_ip),
    airsim_client_image_(host_ip),
    airsim_client_lidar_(host_ip)
{
    initialize_ros2();
    initialize_airsim();
    std::cout<<"node initialized is finished\n";
}


void AirsimROS2Wrapper::initialize_airsim()
{
    try
    {   
        airsim_client_image_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
    }
    catch(rpc::rpc_error& e){
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimROS2Wrapper::initialize_ros2()
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scene_image_pub_=image_transport::create_publisher(this,"PX4/camera/scene");
    depth_image_pub_=image_transport::create_publisher(this,"PX4/camera/depth");
    lidar_pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("PX4/lidar",10);    

    airsim_image_timer_=this->create_wall_timer(1s,std::bind(&AirsimROS2Wrapper::image_timer_callback,this));
    airsim_lidar_timer_=this->create_wall_timer(1s,std::bind(&AirsimROS2Wrapper::lidar_timer_callback,this));

}


void AirsimROS2Wrapper::image_timer_callback()
{    
    try
    {
        const std::vector<ImageResponse>& img_response = airsim_client_image_.simGetImages(requests_, "PX4");
        if (img_response.size() == requests_.size()) 
        {
            process_and_publish_img_response(img_response);
        }
        else{
            std::cerr << "Images were not recerived" << std::endl;
        }    
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}

void AirsimROS2Wrapper::lidar_timer_callback()
{
    try
    {
        auto lidar_data = airsim_client_lidar_.getLidarData("lidar", "PX4");
        process_and_publish_lidar_data(lidar_data);  
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, didn't get lidar response." << std::endl << msg << std::endl;
    }
}


void AirsimROS2Wrapper::process_and_publish_lidar_data(const msr::airlib::LidarData& lidar_data)
{
    rclcpp::Time curr_ros_time = this->now(); 
    publish_lidar_tf(lidar_data,curr_ros_time);
    sensor_msgs::msg::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = curr_ros_time;
    lidar_msg.header.frame_id = "lidar";

    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x"; 
        lidar_msg.fields[1].name = "y"; 
        lidar_msg.fields[2].name = "z";

        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[d].count  = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }
    
    lidar_pub_->publish(lidar_msg);

    
}


void AirsimROS2Wrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec)
{    

    rclcpp::Time curr_ros_time = this->now(); 
    publish_camera_tf(img_response_vec.at(0), curr_ros_time);
    scene_image_pub_.publish(get_scene_img_msg_from_response(img_response_vec.at(0), curr_ros_time, "camera_optical"));        
    depth_image_pub_.publish(get_depth_img_msg_from_response(img_response_vec.at(1), curr_ros_time, "camera_optical"));
}

sensor_msgs::msg::Image AirsimROS2Wrapper::get_scene_img_msg_from_response(const ImageResponse& img_response,const rclcpp::Time curr_ros_time, const std::string frame_id)
{   
    cv::Mat s=cv::imdecode(img_response.image_data_uint8, cv::IMREAD_UNCHANGED);
    cv::imshow("color", s);
    cv::Mat mat=cv::imdecode(img_response.image_data_uint8, cv::IMREAD_GRAYSCALE);
    cv::imshow("gray", mat);
    cv::waitKey(5);
    sensor_msgs::msg::Image img_msg =*(cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mat).toImageMsg());
    img_msg.header.stamp = curr_ros_time;
    img_msg.header.frame_id = frame_id;
    return img_msg;
}

sensor_msgs::msg::Image AirsimROS2Wrapper::get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time,const std::string frame_id)
{

    cv::Mat mat(img_response.height,img_response.width,CV_32FC1,cv::Scalar(0)) ;
    
    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_response.width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_response.width + col];
    
    sensor_msgs::msg::Image img_msg =*(cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", mat).toImageMsg());
    img_msg.header.stamp = curr_ros_time;
    img_msg.header.frame_id = frame_id;
    return img_msg;
}


void AirsimROS2Wrapper::publish_lidar_tf(const msr::airlib::LidarData& lidar_data, const rclcpp::Time& ros_time)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.stamp=ros_time;
    lidar_tf_msg.header.frame_id = "PX4";
    lidar_tf_msg.child_frame_id = "lidar";
    lidar_tf_msg.transform.translation.x = lidar_data.pose.position.x();
    lidar_tf_msg.transform.translation.y = lidar_data.pose.position.y();
    lidar_tf_msg.transform.translation.z = lidar_data.pose.position.z();
    lidar_tf_msg.transform.rotation.x = lidar_data.pose.orientation.x();
    lidar_tf_msg.transform.rotation.y = lidar_data.pose.orientation.y();
    lidar_tf_msg.transform.rotation.z = lidar_data.pose.orientation.z();
    lidar_tf_msg.transform.rotation.w = lidar_data.pose.orientation.w();
    tf_broadcaster_->sendTransform(lidar_tf_msg);
}


void AirsimROS2Wrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time)
{
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = ros_time;
    cam_tf_body_msg.header.frame_id = "PX4";
    cam_tf_body_msg.child_frame_id = "camera_body";
    cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
    cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
    cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
    cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = ros_time;
    cam_tf_optical_msg.header.frame_id = "PX4";
    cam_tf_optical_msg.child_frame_id =  "camera_optical";
    cam_tf_optical_msg.transform.translation.x = cam_tf_body_msg.transform.translation.x;
    cam_tf_optical_msg.transform.translation.y = cam_tf_body_msg.transform.translation.y;
    cam_tf_optical_msg.transform.translation.z = cam_tf_body_msg.transform.translation.z;

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body); 

    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), 
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ()); 
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, cam_tf_optical_msg.transform.rotation);

    tf_broadcaster_->sendTransform(cam_tf_body_msg);
    tf_broadcaster_->sendTransform(cam_tf_optical_msg);
}

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto airsim_node =std::make_shared<AirsimROS2Wrapper>("192.168.1.108");
    exec.add_node(airsim_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}