#include "image_handler/camera_node.hpp"

using namespace std::chrono_literals;


CameraNode::CameraNode():Node("ImageHandler"){
    scene_sub_=image_transport::create_subscription(this,"PX4/camera/scene",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg){
            scene_img_=cv_bridge::toCvCopy(msg);
        },"raw");
    depth_sub_=image_transport::create_subscription(this,"PX4/camera/depth",
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg){
            depth_img_=cv_bridge::toCvCopy(msg);
        },"raw");
    target_position_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("target_position/camera",5);
    timer_=this->create_wall_timer(1s,std::bind(&CameraNode::timer_callback,this));
}


void CameraNode::timer_callback(){

    if(scene_img_->header.stamp.sec==depth_img_->header.stamp.sec){
        if(scene_img_->header.stamp.nanosec==depth_img_->header.stamp.nanosec){
            publish_target_location(scene_img_->image,depth_img_->image);
        }
    }

}

bool CameraNode::publish_target_location(cv::Mat& scene,cv::Mat& depth){

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
    alpha = (float)(gx-params_.width/2.0f) / (float)params_.width * (float)params_.fov_hor;
    beta = (float)(gy-params_.height/2.0f) / (float)params_.height * (float)params_.fov_vel;
    theta =std::atan(std::sqrt(std::tan(alpha)*std::tan(alpha)+std::tan(beta)*std::tan(beta)));
    float dist_z = dist * std::cos(theta);
    float dist_y = dist_z * std::tan(beta);
    float dist_x = dist_z * std::tan(-alpha);

    std::cout<<"x:"<<dist_x<<" y:"<<dist_y<<" z:"<<dist_z<<std::endl;
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp=this->now();
    msg.header.frame_id="camera_optical";
    msg.point.x=dist_x;
    msg.point.y=dist_y;
    msg.point.z=dist_z;
    target_position_pub_->publish(msg);
    return true;
}

int main(int argc, char* argv[]) {
	std::cout << "Starting camera node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraNode>());
	rclcpp::shutdown();
	return 0;
}