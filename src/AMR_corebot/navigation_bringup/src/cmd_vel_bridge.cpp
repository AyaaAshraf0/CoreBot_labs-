#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelBridge : public rclcpp::Node
{
public:
  CmdVelBridge() : Node("cmd_vel_bridge")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelBridge::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_controller/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Twist → TwistStamped bridge started");
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = this->get_clock()->now();
    stamped.header.frame_id = "base_link";  // important
    stamped.twist = *msg;

    pub_->publish(stamped);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}