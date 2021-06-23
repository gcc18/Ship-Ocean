#include "airsim_ros2_msgs/airsim_ros2_wrapper.hpp"

using namespace std::chrono_literals;

const std::unordered_map<int, std::string> AirsimROS2Wrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};


AirsimROS2Wrapper::AirsimROS2Wrapper(const std::string& host_ip):
    Node("AirsimROS2Wrapper"),
    host_ip_(host_ip),
    airsim_client_(host_ip),
    airsim_client_image_(host_ip),
    airsim_client_lidar_(host_ip),
    airsim_settings_parser_(host_ip)
{
    //ros_clock_=this->get_clock();
    initialize_ros2();
    initialize_airsim();
    std::cout<<"node initialized test\n";
}



void AirsimROS2Wrapper::initialize_airsim()
{
    try
    {   
        
        airsim_client_.confirmConnection();
        airsim_client_image_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
        origin_geo_point_ = airsim_client_.getHomeGeoPoint("");
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch(rpc::rpc_error& e){
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimROS2Wrapper::initialize_ros2()
{
    this->get_parameter("is_vulkan",is_vulkan_);
    this->get_parameter("is_publish_clock",is_publish_clock_);
    this->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
    odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID : ENU_ODOM_FRAME_ID;
    this->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
    is_ENU_ = !(odom_frame_id_ == AIRSIM_ODOM_FRAME_ID);
    this->get_parameter("is_coordinate_system_enu",is_ENU_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    create_ros2_pubs_from_setting_json();
    airsim_state_timer_=this->create_wall_timer(0.5s,std::bind(&AirsimROS2Wrapper::state_timer_callback,this));
    airsim_image_timer_=this->create_wall_timer(1s,std::bind(&AirsimROS2Wrapper::image_timer_callback,this));
    airsim_lidar_timer_=this->create_wall_timer(1s,std::bind(&AirsimROS2Wrapper::lidar_timer_callback,this));
    if (is_publish_clock_)
    {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);
    }
}

void AirsimROS2Wrapper::create_ros2_pubs_from_setting_json(){
    vehicle_name_ptr_map_.clear();
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default; 
    origin_geo_point_pub_=this->create_publisher<airsim_ros2_msgs::msg::GPSYaw>("origin_geo_point", 10); 

    for(const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles)
    {
        auto curr_vehicle_name = curr_vehicle_elem.first;
        auto& vehicle_setting = curr_vehicle_elem.second;
        std::unique_ptr<VehicleROS> vehicle_ros = std::unique_ptr<VehicleROS>(new VehicleROS());
        
        set_nans_to_zeros_in_pose(*vehicle_setting);
        
        vehicle_ros->vehicle_name=curr_vehicle_name;
        vehicle_ros->odom_frame_id=curr_vehicle_name+"/"+odom_frame_id_;
        append_static_vehicle_tf(vehicle_ros.get(),*vehicle_setting);
        vehicle_ros->env_pub = this->create_publisher<airsim_ros2_msgs::msg::Environment>(curr_vehicle_name + "/environment", 10);        
 
        

        vehicle_ros->alt_name_pub_map.clear();
        vehicle_ros->imu_name_pub_map.clear();
        vehicle_ros->gps_name_pub_map.clear();
        vehicle_ros->mag_name_pub_map.clear();
        vehicle_ros->dist_name_pub_map.clear();
        vehicle_ros->lidar_name_pub_map.clear();
        

        //iterate over cameras
        for(auto& curr_camera_elem:vehicle_setting->cameras)
        {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
            
            for(const auto&curr_capture_elem:camera_setting.capture_settings)
            {
                auto& capture_setting = curr_capture_elem.second;
                
                if ( !(std::isnan(capture_setting.fov_degrees)) )
                {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (capture_setting.image_type == 0 || capture_setting.image_type == 5 || capture_setting.image_type == 6 || capture_setting.image_type == 7)
                    {
                        vehicle_ros->image_requests.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else
                    {
                        vehicle_ros->image_requests.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }
                    vehicle_ros->image_pubs.push_back(image_transport::create_publisher(this,curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type), custom_qos));
                    vehicle_ros->camera_info_pubs.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo> (curr_vehicle_name + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type) + "/camera_info", 10));
                    vehicle_ros->camera_info_msgs.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                }
            }
        }
        
        //iterate over sensors
        for(const auto& curr_sensor_elem:vehicle_setting->sensors)
        {
            auto& sensor_name = curr_sensor_elem.first;
            auto& sensor_setting = curr_sensor_elem.second;
            if(sensor_setting->enabled)
            {
                switch (sensor_setting->sensor_type)
                {
                    case SensorBase::SensorType::Barometer:
                    {
                        std::cout << "Barometer" << std::endl; 
                        auto sensor_publisher=this->create_publisher<airsim_ros2_msgs::msg::Altimeter>(curr_vehicle_name + "/altimeter/" + sensor_name, 10);
                        vehicle_ros->alt_name_pub_map.emplace(sensor_name, *sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Imu:
                    {
                        std::cout << "Imu" << std::endl;
                        auto sensor_publisher=this->create_publisher<sensor_msgs::msg::Imu>(curr_vehicle_name + "/imu/" + sensor_name, 10);
                        vehicle_ros->imu_name_pub_map.emplace(sensor_name, *sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Gps:
                    {
                        std::cout << "GPS" << std::endl;
                        auto gps_publisher=this->create_publisher<sensor_msgs::msg::NavSatFix>(curr_vehicle_name + "/gps/" + sensor_name, 10);
                        vehicle_ros->gps_name_pub_map.emplace(sensor_name, *gps_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Magnetometer:
                    {
                        std::cout << "Magnetometer" << std::endl;
                        auto sensor_publisher=this->create_publisher<sensor_msgs::msg::MagneticField>(curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                        vehicle_ros->mag_name_pub_map.emplace(sensor_name, *sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Distance:
                    {
                        std::cout << "Distance" << std::endl; 
                        auto sensor_publisher=this->create_publisher<sensor_msgs::msg::Range>(curr_vehicle_name + "/distance/" + sensor_name, 10);
                        vehicle_ros->dist_name_pub_map.emplace(sensor_name, *sensor_publisher);
                        break;
                    }
                    case SensorBase::SensorType::Lidar:
                    {
                        std::cout << "Lidar" << std::endl;
                        auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                        set_nans_to_zeros_in_pose(*vehicle_setting, lidar_setting);
                        append_static_lidar_tf(vehicle_ros.get(), sensor_name, lidar_setting);
                        auto sensor_publisher=this->create_publisher<sensor_msgs::msg::PointCloud2>(curr_vehicle_name + "/lidar/" + sensor_name, 10);
                        vehicle_ros->lidar_name_pub_map.emplace(sensor_name, *sensor_publisher);
                        break;
                    }
                    default:
                    {
                        throw std::invalid_argument("Unexpected sensor type");
                    }
                }
            }    
        }
        // allows fast lookup in command callbacks in case of a lot of drones
        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros));
    }

    if (is_publish_clock_)
    {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);
    }

}


void AirsimROS2Wrapper::state_timer_callback()
{
    try
    {   
        origin_geo_point_pub_->publish(origin_geo_point_msg_);
        const auto now = update_state();
        if (!airsim_client_.simIsPaused())
        {
            // airsim_client needs to provide the simulation time in a future version of the API
      //      ros_clock_.clock = now;
        }
        // publish the simulation clock
        if (is_publish_clock_)
        {
     //       clock_pub_->publish(ros_clock_);
        }
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }
}

void AirsimROS2Wrapper::image_timer_callback()
{    
    try
    {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
        {
            const std::vector<ImageResponse>& img_response = airsim_client_image_.simGetImages(vehicle_name_ptr_pair.second->image_requests, vehicle_name_ptr_pair.first);
            if (img_response.size() == vehicle_name_ptr_pair.second->image_requests.size()) 
            {
                process_and_publish_img_response(img_response, vehicle_name_ptr_pair.second.get());
            }    
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
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
        {
            for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->lidar_name_pub_map)
            {
                auto lidar_data = airsim_client_lidar_.getLidarData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
                sensor_msgs::msg::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data, vehicle_name_ptr_pair.first);
                sensor_name_pub_pair.second.publish(lidar_msg);
            }
        }
    }
    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cerr << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }
}

rclcpp::Time AirsimROS2Wrapper::update_state()
{
    bool got_sim_time = false;
    rclcpp::Time curr_ros_time = this->now();

    // iterate over drones
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
    {
        rclcpp::Time vehicle_time;
        // get drone state from airsim 
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other truths
        auto env_data = airsim_client_.simGetGroundTruthEnvironment(vehicle_ros->vehicle_name);

        //vehicle_time = airsim_timestamp_to_ros(curr_drone_state.timestamp);
        vehicle_time=this->now();
        if (!got_sim_time)
        {
            curr_ros_time = vehicle_time;
            got_sim_time = true;
        }

        
        // convert airsim drone state to ROS msgs
        auto env_msg = get_environment_msg_from_airsim(env_data);
        env_msg.header.frame_id = vehicle_ros->vehicle_name;
        env_msg.header.stamp = vehicle_time;
        vehicle_ros->env_pub->publish(env_msg);

        
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec)
        {
            static_tf_msg.header.stamp = vehicle_time;
            tf_static_broadcaster_->sendTransform(static_tf_msg);
        } 
        
        //iterate over sensors
        for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->alt_name_pub_map)
        {
            auto alt_data = airsim_client_.getBarometerData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
            auto alt_msg = get_altimeter_msg_from_airsim(alt_data);
            sensor_name_pub_pair.second.publish(alt_msg);
        }
        for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->imu_name_pub_map)
        {
            auto imu_data = airsim_client_.getImuData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
            auto imu_msg = get_imu_msg_from_airsim(imu_data);
            sensor_name_pub_pair.second.publish(imu_msg);
        } 
        for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->gps_name_pub_map)
        {
            auto gps_data = airsim_client_.getGpsData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
            auto gps_msg = get_gps_msg_from_airsim(gps_data);
            sensor_name_pub_pair.second.publish(gps_msg);
        } 
        for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->mag_name_pub_map)
        {
            auto mag_data = airsim_client_.getMagnetometerData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
            auto mag_msg = get_mag_msg_from_airsim(mag_data);
            sensor_name_pub_pair.second.publish(mag_msg);
        } 
        for (auto& sensor_name_pub_pair : vehicle_name_ptr_pair.second->dist_name_pub_map)
        {
            auto dist_data = airsim_client_.getDistanceSensorData(sensor_name_pub_pair.first, vehicle_name_ptr_pair.first);
            auto dist_msg = get_range_msg_from_airsim(dist_data);
            sensor_name_pub_pair.second.publish(dist_msg);
        }  
    }
    return curr_ros_time;
}


void AirsimROS2Wrapper::append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::msg::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = this->now();
    vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name;
    vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
    vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
    vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
    vehicle_tf_msg.transform.rotation.x = quat.x();
    vehicle_tf_msg.transform.rotation.y = quat.y();
    vehicle_tf_msg.transform.rotation.z = quat.z();
    vehicle_tf_msg.transform.rotation.w = quat.w();

    if (is_ENU_)
    {
        std::swap(vehicle_tf_msg.transform.translation.x, vehicle_tf_msg.transform.translation.y);
        std::swap(vehicle_tf_msg.transform.rotation.x, vehicle_tf_msg.transform.rotation.y);
        vehicle_tf_msg.transform.translation.z = -vehicle_tf_msg.transform.translation.z;
        vehicle_tf_msg.transform.rotation.z = -vehicle_tf_msg.transform.rotation.z;
    }

    vehicle_ros->static_tf_msg_vec.emplace_back(vehicle_tf_msg);
}

void AirsimROS2Wrapper::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const LidarSetting& lidar_setting)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
    lidar_tf_msg.child_frame_id = vehicle_ros->vehicle_name + "/" + lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.position.x();
    lidar_tf_msg.transform.translation.y = lidar_setting.position.y();
    lidar_tf_msg.transform.translation.z = lidar_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(lidar_setting.rotation.roll, lidar_setting.rotation.pitch, lidar_setting.rotation.yaw);
    lidar_tf_msg.transform.rotation.x = quat.x();
    lidar_tf_msg.transform.rotation.y = quat.y();
    lidar_tf_msg.transform.rotation.z = quat.z();
    lidar_tf_msg.transform.rotation.w = quat.w();

    if (is_ENU_)
    {
        std::swap(lidar_tf_msg.transform.translation.x, lidar_tf_msg.transform.translation.y);
        std::swap(lidar_tf_msg.transform.rotation.x, lidar_tf_msg.transform.rotation.y);
        lidar_tf_msg.transform.translation.z = -lidar_tf_msg.transform.translation.z;
        lidar_tf_msg.transform.rotation.z = -lidar_tf_msg.transform.rotation.z;
    }

    vehicle_ros->static_tf_msg_vec.emplace_back(lidar_tf_msg);
}

void AirsimROS2Wrapper::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name + "/" + odom_frame_id_;
    static_cam_tf_body_msg.child_frame_id = camera_name + "_body/static";
    static_cam_tf_body_msg.transform.translation.x = camera_setting.position.x();
    static_cam_tf_body_msg.transform.translation.y = camera_setting.position.y();
    static_cam_tf_body_msg.transform.translation.z = camera_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(camera_setting.rotation.roll, camera_setting.rotation.pitch, camera_setting.rotation.yaw);
    static_cam_tf_body_msg.transform.rotation.x = quat.x();
    static_cam_tf_body_msg.transform.rotation.y = quat.y();
    static_cam_tf_body_msg.transform.rotation.z = quat.z();
    static_cam_tf_body_msg.transform.rotation.w = quat.w();

    if (is_ENU_)
    {
        std::swap(static_cam_tf_body_msg.transform.translation.x, static_cam_tf_body_msg.transform.translation.y);
        std::swap(static_cam_tf_body_msg.transform.rotation.x, static_cam_tf_body_msg.transform.rotation.y);
        static_cam_tf_body_msg.transform.translation.z = -static_cam_tf_body_msg.transform.translation.z;
        static_cam_tf_body_msg.transform.rotation.z = -static_cam_tf_body_msg.transform.rotation.z;
    }

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";
    
    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(static_cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body); 
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), 
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ()); 
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, static_cam_tf_optical_msg.transform.rotation);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec.emplace_back(static_cam_tf_optical_msg);
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS 
void AirsimROS2Wrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimROS2Wrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;

}

void AirsimROS2Wrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const
{
    if (std::isnan(lidar_setting.position.x()))
        lidar_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(lidar_setting.position.y()))
        lidar_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(lidar_setting.position.z()))
        lidar_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(lidar_setting.rotation.yaw))
        lidar_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(lidar_setting.rotation.pitch))
        lidar_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(lidar_setting.rotation.roll))
        lidar_setting.rotation.roll = vehicle_setting.rotation.roll;
}

sensor_msgs::msg::CameraInfo AirsimROS2Wrapper::generate_cam_info(const std::string& camera_name,const CameraSetting& camera_setting,const CaptureSetting& capture_setting) const
{
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan((capture_setting.fov_degrees / 2.0)/360*M_PI);

    cam_info_msg.k = {f_x, 0.0, capture_setting.width / 2.0, 
                        0.0, f_x, capture_setting.height / 2.0, 
                        0.0, 0.0, 1.0};
    cam_info_msg.p = {f_x, 0.0, capture_setting.width / 2.0, 0.0,
                        0.0, f_x, capture_setting.height / 2.0, 0.0, 
                        0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}

sensor_msgs::msg::PointCloud2 AirsimROS2Wrapper::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name) const
{
    sensor_msgs::msg::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = this->now();
    lidar_msg.header.frame_id = vehicle_name;

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
 
    if (is_ENU_)
    {
        try
        {
            sensor_msgs::msg::PointCloud2 lidar_msg_enu;
            auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, this->now());
            tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);

            lidar_msg_enu.header.stamp = lidar_msg.header.stamp;
            lidar_msg_enu.header.frame_id = lidar_msg.header.frame_id;

            lidar_msg = std::move(lidar_msg_enu);
        }
        catch (tf2::TransformException &ex)
        {
            std::string msg = ex.what();
            std::cerr<< msg << std::endl;
            rclcpp::sleep_for(100ms);
        }
    }

    return lidar_msg;
}




void AirsimROS2Wrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec , VehicleROS* vehicle_ros)
{    
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    rclcpp::Time curr_ros_time = this->now(); 
    int img_response_idx=0;
    for (const auto& curr_img_response : img_response_vec)
    {

        publish_camera_tf(curr_img_response, curr_ros_time, vehicle_ros->vehicle_name, curr_img_response.camera_name);

        vehicle_ros->camera_info_msgs[img_response_idx].header.stamp = curr_ros_time;
        vehicle_ros->camera_info_pubs[img_response_idx]->publish(vehicle_ros->camera_info_msgs[img_response_idx]);

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float)
        {
             vehicle_ros->image_pubs[img_response_idx].publish(get_depth_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else
        {
            vehicle_ros->image_pubs[img_response_idx].publish(get_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical"));
        }
        img_response_idx++;
    }

}

sensor_msgs::msg::Image AirsimROS2Wrapper::get_img_msg_from_response(const ImageResponse& img_response,const rclcpp::Time curr_ros_time, const std::string frame_id)
{
    sensor_msgs::msg::Image img_msg;
    img_msg.data = img_response.image_data_uint8;
    img_msg.step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg.header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    img_msg.header.frame_id = frame_id;
    img_msg.height = img_response.height;
    img_msg.width = img_response.width;
    img_msg.encoding = "bgr8";
    if (is_vulkan_) img_msg.encoding = "rgb8";
    img_msg.is_bigendian = 0;
    return img_msg;
}

sensor_msgs::msg::Image AirsimROS2Wrapper::get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time,const std::string frame_id)
{

    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::msg::Image depth_img_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_img).toImageMsg());
    depth_img_msg.header.stamp = airsim_timestamp_to_ros(img_response.time_stamp);
    depth_img_msg.header.frame_id = frame_id;
    return depth_img_msg;
}

cv::Mat AirsimROS2Wrapper::manual_decode_depth(const ImageResponse& img_response) const
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

airsim_ros2_msgs::msg::GPSYaw AirsimROS2Wrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    airsim_ros2_msgs::msg::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

airsim_ros2_msgs::msg::Environment AirsimROS2Wrapper::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    airsim_ros2_msgs::msg::Environment env_msg;
    env_msg.position.x = env_data.position.x();
    env_msg.position.y = env_data.position.y();
    env_msg.position.z = env_data.position.z();
    env_msg.geo_point.latitude = env_data.geo_point.latitude;
    env_msg.geo_point.longitude = env_data.geo_point.longitude;
    env_msg.geo_point.altitude = env_data.geo_point.altitude;
    env_msg.gravity.x = env_data.gravity.x();
    env_msg.gravity.y = env_data.gravity.y();
    env_msg.gravity.z = env_data.gravity.z();
    env_msg.air_pressure = env_data.air_pressure;
    env_msg.temperature = env_data.temperature;
    env_msg.air_density = env_data.temperature;

    return env_msg;
}

sensor_msgs::msg::MagneticField AirsimROS2Wrapper::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    std::copy(std::begin(mag_data.magnetic_field_covariance), 
        std::end(mag_data.magnetic_field_covariance), 
        std::begin(mag_msg.magnetic_field_covariance));
    mag_msg.header.stamp = airsim_timestamp_to_ros(mag_data.time_stamp);
    return mag_msg;
}

sensor_msgs::msg::NavSatFix AirsimROS2Wrapper::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = airsim_timestamp_to_ros(gps_data.time_stamp);
    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude; 
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    gps_msg.status.status = gps_data.gnss.fix_type;

    return gps_msg;
}

sensor_msgs::msg::Range AirsimROS2Wrapper::get_range_msg_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::msg::Range dist_msg;
    dist_msg.header.stamp = airsim_timestamp_to_ros(dist_data.time_stamp);
    dist_msg.range = dist_data.distance;
    dist_msg.min_range = dist_data.min_distance;
    dist_msg.max_range = dist_data.min_distance;

    return dist_msg;
}

airsim_ros2_msgs::msg::Altimeter AirsimROS2Wrapper::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    airsim_ros2_msgs::msg::Altimeter alt_msg;
    alt_msg.header.stamp = airsim_timestamp_to_ros(alt_data.time_stamp);
    alt_msg.altitude = alt_data.altitude;
    alt_msg.pressure = alt_data.pressure;
    alt_msg.qnh = alt_data.qnh;

    return alt_msg;
}

sensor_msgs::msg::Imu AirsimROS2Wrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple drones
    imu_msg.header.stamp = airsim_timestamp_to_ros(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m 
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    return imu_msg;
}

void AirsimROS2Wrapper::publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id; 
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(odom_tf);
}

void AirsimROS2Wrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = ros_time;
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = child_frame_id + "_body";
    cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
    cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
    cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
    cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

    if (is_ENU_)
    {
        std::swap(cam_tf_body_msg.transform.translation.x, cam_tf_body_msg.transform.translation.y);
        std::swap(cam_tf_body_msg.transform.rotation.x, cam_tf_body_msg.transform.rotation.y);
        cam_tf_body_msg.transform.translation.z = -cam_tf_body_msg.transform.translation.z;
        cam_tf_body_msg.transform.rotation.z = -cam_tf_body_msg.transform.rotation.z;
    }

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = ros_time;
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
    cam_tf_optical_msg.transform.translation.x = cam_tf_body_msg.transform.translation.x;
    cam_tf_optical_msg.transform.translation.y = cam_tf_body_msg.transform.translation.y;
    cam_tf_optical_msg.transform.translation.z = cam_tf_body_msg.transform.translation.z;

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body); 
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body * matrix_cam_body_to_optical_inverse_;
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body;
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

rclcpp::Time AirsimROS2Wrapper::airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const
{
    // airsim appears to use chrono::system_clock with nanosecond precision
    std::chrono::nanoseconds dur(stamp);
    std::chrono::time_point<std::chrono::system_clock> tp(dur);
    rclcpp::Time cur_time = chrono_timestamp_to_ros(tp);
    return cur_time;
}


rclcpp::Time AirsimROS2Wrapper::chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const
{
    auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
    rclcpp::Time ros_time(dur.count(),0);
    return ros_time;
}