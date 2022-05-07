#include <iostream>
#include <sensor_msgs/msg/joint_state.h>

// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Publisher for the 'cmd_vel'(Twist) topic. Subscribed by the simulation
// Subscribes to the 'cmd' topic (float32MultiArray) published by the slider_publisher

//To do - The kinematic model for the 1,1 robot

class fwd_kinematics_node : public rclcpp::Node
{
public:
    fwd_kinematics_node()
    : Node("fwd_kinematics_node")
    {
      publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("/bike/cmd_vel", 1);

      subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("cmd", 1,
                  std::bind(&fwd_kinematics_node::topic_callback, this, _1));

      timer_ = create_wall_timer(
      50ms, std::bind(&fwd_kinematics_node::timer_callback, this));
    }

private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 10.0;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = 1.0;

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing: '%f'\n\n", message.linear.x);
      publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
        {
          RCLCPP_INFO(this->get_logger(), "v_control '%f'", msg->data[0]);
          RCLCPP_INFO(this->get_logger(), "beta_dot_control '%f'", msg->data[1]);
        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    //size_t count_;

    //subscriber
    std_msgs::msg::Float32MultiArray sub_cmd;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    public:

};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fwd_kinematics_node>());
  rclcpp::shutdown();
  return 0;
}
