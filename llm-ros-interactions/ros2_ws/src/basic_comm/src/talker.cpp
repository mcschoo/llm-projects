#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker_node") {
    // Declare the message parameter with a default
    this->declare_parameter<std::string>("message", "Hello world");

    // Set up the publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("commands", 10);

    // Timer to publish periodically
    timer_ = this->create_wall_timer(1s, std::bind(&Talker::publish_message, this));

    // Set up callback for parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Talker::on_parameter_change, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &params) {
    for (const auto &param : params) {
      if (param.get_name() == "message" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        message_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Updated message to: '%s'", message_.c_str());
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameter accepted";
    return result;
  }

  void publish_message() {
    auto msg = std_msgs::msg::String();
    msg.data = message_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  std::string message_ = "Hello world";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
