#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"
#include "more_interfaces/msg/gps_yaw.hpp"

using namespace std::chrono_literals;

class MsgsPublisher : public rclcpp::Node
{
public:
  MsgsPublisher()
  : Node("msgs_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);
    gps_yaw_publisher_ =this->create_publisher<more_interfaces::msg::GPSYaw>("GPS_Yaw",10);
    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    auto publish_gps=[this]()->void{
        auto message=more_interfaces::msg::GPSYaw();
        message.latitude=22;
        message.longitude=120;
        message.altitude=200;
        message.yaw=60;
        std::cout<<"Publishing gps information"<<std::endl;
        this->gps_yaw_publisher_->publish(message);
      };
    timer1_ = this->create_wall_timer(2s, publish_msg);
    timer2_ =this->create_wall_timer(0.5s,publish_gps);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<more_interfaces::msg::GPSYaw>::SharedPtr gps_yaw_publisher_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<more_interfaces::msg::Environment>::SharedPtr env_publisher_;
  rclcpp::TimerBase::SharedPtr timer3_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgsPublisher>());
  rclcpp::shutdown();

  return 0;
}