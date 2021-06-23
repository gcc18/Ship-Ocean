#include "image_handler/image_handler.hpp"

using namespace std::chrono_literals;

ImageHandler::ImageHandler(const std::string& host_ip):Node("ImageHandler"),airsim_client_(host_ip){
	try{
		airsim_client_.confirmConnection();
        initialize_params();
        initialize_ros2();
        timer_ = this->create_wall_timer(1s,std::bind(&ImageHandler::timer_callback,this));
	}catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cerr << msg << std::endl;
	}
}


void ImageHandler::initialize_params(){
    try{
		airsim_client_.confirmConnection();
        responses_ = airsim_client_.simGetImages(requests_);
        params_.width  =  responses_.at(0).width;
        params_.height =  responses_.at(0).height;
        params_.fov_vel=  params_.fov_hor*((double)params_.height/(double)params_.width);
        params_.info_msg.header.frame_id = "camera_optical";
        params_.info_msg.height = params_.height;
        params_.info_msg.width = params_.width;
        float f_x = (params_.width / 2.0) / tan((params_.fov_hor / 2.0)/360*M_PI);
        params_.info_msg.k = {f_x, 0.0, params_.width / 2.0, 
                            0.0, f_x, params_.height / 2.0, 
                            0.0, 0.0, 1.0};
        params_.info_msg.p = {f_x, 0.0, params_.width / 2.0, 0.0,
                            0.0, f_x, params_.height / 2.0, 0.0, 
                            0.0, 0.0, 1.0, 0.0};    
    }catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cerr << msg << std::endl;
	}
}

void ImageHandler::initialize_ros2(){
    px4_timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
        [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
            px4_timestamp_.store(msg->timestamp);
        });
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default; 
	scene_image_pub_=image_transport::create_publisher(this,"front_center/scene", custom_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    target_position_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("target_position/camera",10);
    camera_info_pub_=this->create_publisher<sensor_msgs::msg::CameraInfo>("front_center/scene/camera_info",10);
}


void ImageHandler::timer_callback(){
	cv::Mat mat_scene;
	cv::Mat mat_depth;
    geometry_msgs::msg::Pose pose;
	if(get_image_from_airsim(mat_scene,mat_depth,pose)){
        publish_target_location_from_image(mat_scene,mat_depth,pose);
    }	
}


bool ImageHandler::get_image_from_airsim(cv::Mat &scene, cv::Mat &depth, geometry_msgs::msg::Pose &pose){
	try{
		responses_=airsim_client_.simGetImages(requests_);
		if (responses_.size() != requests_.size()) {
			std::cerr << "Images were not recerived" << std::endl;
			return false;
		}

		publish_scene_image(responses_.at(0));
        publish_camera_tf(responses_.at(0));
        cv::Mat s=cv::imdecode(responses_.at(0).image_data_uint8, cv::IMREAD_GRAYSCALE);
		cv::Mat t(responses_.at(1).height, responses_.at(1).width, CV_32FC1, (void*)responses_.at(1).image_data_float.data());
		scene =s.clone();
        depth =t.clone();
        pose.position.x = responses_.at(0).camera_position.x();
        pose.position.y = responses_.at(0).camera_position.y();
        pose.position.z = responses_.at(0).camera_position.z();
        pose.orientation.x = responses_.at(0).camera_orientation.x();
        pose.orientation.y = responses_.at(0).camera_orientation.y();
        pose.orientation.z = responses_.at(0).camera_orientation.z();
        pose.orientation.w = responses_.at(0).camera_orientation.w();
		
	}catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cerr << msg << std::endl;
	}
    return true;
}

bool ImageHandler::publish_target_location_from_image(cv::Mat &scene, cv::Mat &depth,geometry_msgs::msg::Pose &pose){

    cv::blur(scene,scene,cv::Size(3,3));
    
    int thresh = 50;
    cv::RNG rng(12345);
    cv::Mat canny_output;
    cv::Canny(scene, canny_output, thresh, thresh*2 );

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    
    
    cv::Moments mu_select;
    std::vector<cv::Point> contour_select; 
    std::vector<cv::Moments> mu(contours.size());
    double current_num = MINIMUM_PIXELS_FOR_TARGET;
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours.at(i));
        if (mu[i].m00 > current_num) {
            current_num = mu[i].m00;
            mu_select = mu[i];
            contour_select=contours.at(i);
        }
    }
    if (mu_select.m00 < MINIMUM_PIXELS_FOR_TARGET) {
        return false;
    }

    //display contour of target
    std::vector<cv::Point>  contour_poly;
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    cv::approxPolyDP( contour_select, contour_poly, 3, true );
    cv::Rect boundRect = cv::boundingRect( contour_poly );
    cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    cv::rectangle( drawing, boundRect.tl(), boundRect.br(), color, 2 );
    cv::imshow("Scene",scene);
    cv::imshow("Contours", drawing );
    cv::waitKey(5);



    //initialized the distance
    int gx = (int)(mu_select.m10 / mu_select.m00 + 1e-5);
    int gy = (int)(mu_select.m01 / mu_select.m00 + 1e-5);
    float dist = 1000;
    dist = depth.at<float>(gy, gx);
    

    //distance to local coordinate xyz of camera 
    float alpha, beta, theta;
    alpha = (float)(gx-params_.width/2.0) / (float)params_.width * params_.fov_hor *M_PI/360.0;
    beta = (float)(gy-params_.height/2.0) / (float)params_.height * params_.fov_vel *M_PI/360.0;
    theta =std::atan(std::sqrt(std::tan(alpha)*std::tan(alpha)+std::tan(beta)*std::tan(beta)));
    std::cout<<"alpha:"<<alpha<<" beta:"<<beta<<" theta:"<<theta<<std::endl;
    float dist_z = dist * std::cos(theta);
    float dist_y = dist * std::sin(theta) * std::tan(beta);
    float dist_x = dist * std::sin(theta) * std::tan(alpha);


    Eigen::Quaternionf q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    auto m=q.toRotationMatrix().inverse();
    Eigen::Vector3f v(dist_z,dist_y,dist_x);
    auto vv=m*v;


    std::cout<<"x:"<<vv.x()<<" y:"<<vv.y()<<" z:"<<vv.z()<<std::endl;
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp=px4_timestamp_to_ros(px4_timestamp_);
    msg.header.frame_id="odom_local_ned";
    msg.point.x=vv.x();
    msg.point.y=vv.y();
    msg.point.z=vv.z();
    target_position_pub_->publish(msg);

/*
    msg.timestamp=timestamp_;
    msg.position.x=pp.x();
    msg.position.y=pp.y();
    msg.position.z=pp.z();
    msg.correct=current_num; 
    msg.error=dist*std::tan(fov_/2)/height_/2; */
    return true;
}


void ImageHandler::publish_scene_image(const ImageResponse img_response)
{
    sensor_msgs::msg::Image img_msg;
    rclcpp::Time stamp = px4_timestamp_to_ros(px4_timestamp_);
    img_msg.data = img_response.image_data_uint8;
    img_msg.step = img_response.width; 
    img_msg.header.stamp = stamp;
    img_msg.header.frame_id = "camera_optical";
    img_msg.height = img_response.height;
    img_msg.width = img_response.width;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = 0;
    scene_image_pub_.publish(img_msg);
    params_.info_msg.header.stamp=stamp;
    camera_info_pub_->publish(params_.info_msg);

}


void ImageHandler::publish_camera_tf(const ImageResponse img_response)
{
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = px4_timestamp_to_ros(px4_timestamp_);
    cam_tf_body_msg.header.frame_id = "odom_local_ned";
    cam_tf_body_msg.child_frame_id = "camera_body";
    cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
    cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
    cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
    cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = px4_timestamp_to_ros(px4_timestamp_);
    cam_tf_optical_msg.header.frame_id = "odom_local_ned";
    cam_tf_optical_msg.child_frame_id = "camera_optical";
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

rclcpp::Time ImageHandler::px4_timestamp_to_ros(const std::atomic<uint64_t>& stamp)
{
    auto s = stamp/1000;
    auto ns = (stamp%1000)*1000;
    rclcpp::Time cur_time(s,ns);
    return cur_time;
}


int main(int argc, char* argv[]) {
	std::cout << "Starting camera node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageHandler>("192.168.1.108"));
	rclcpp::shutdown();
	return 0;
}