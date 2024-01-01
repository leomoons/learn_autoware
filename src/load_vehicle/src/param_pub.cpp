#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ParamPublisher : public rclcpp::Node
{
  public:
    ParamPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      this->declare_parameter("map_path", "default value");
      this->declare_parameter("vehicle_model", "default value");
      this->declare_parameter("sensor_model", "default value");
      this->declare_parameter("data_path", "default value");
      this->declare_parameter("planning_module_preset", "default value");
      this->declare_parameter("lanelet2_map_file", "default value");
      this->declare_parameter("pointcloud_map_file", "default value");
      this->declare_parameter("launch_system_monitor", "default value");
      this->declare_parameter("launch_dummy_diag_publisher", "default value");
      this->declare_parameter("initial_engage_state", "default value");
      this->declare_parameter("perception/enable_detection_failure", "default value");
      this->declare_parameter("perception/enable_object_recognition", "default value");
      this->declare_parameter("perception/use_base_link_z", "default value");
      this->declare_parameter("sensing/visible_range", "default value");
      this->declare_parameter("scenario_simulation", "default value");

      auto mess = std_msgs::msg::String();
      mess.data = "all the arguments or parameters";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", mess.data.c_str());
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      2000ms, std::bind(&ParamPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std::string param_name, param_value;

      param_name = "map_path";
      param_value = this->get_parameter(param_name).as_string();
      RCLCPP_INFO(this->get_logger(), "%s: %s", param_name.c_str(), param_value.c_str());

      param_name = "vehicle_model";
      param_value = this->get_parameter(param_name).as_string();
      RCLCPP_INFO(this->get_logger(), "%s: %s", param_name.c_str(), param_value.c_str());


      // auto message = std_msgs::msg::String();
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamPublisher>());
  rclcpp::shutdown();
  return 0;
}