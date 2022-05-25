// Publisher for the 'cmd_vel'(Twist) topic. Subscribed by the simulation
// Publisher fo the joint states ( sensor_msgs). Subscribed
// Subscribes to the 'cmd' topic (float32MultiArray) published by the slider_publisher

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

//std_msgs::msg::Float32MultiArray::SharedPtr get_msg;
float v_control, beta_one_dot_control, beta_two_dot_control;

class two_steering_kinematics_node : public rclcpp::Node
{
public:
    two_steering_kinematics_node()
    : Node("two_steering_kinematics_node")
    {
        subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("cmd", 1,
                    std::bind(&two_steering_kinematics_node::topic_callback, this, _1));

        publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("/two_steering/cmd_vel", 1);

        publisher_jointState  = this->create_publisher<sensor_msgs::msg::JointState>("/two_steering/joint_states", 1);

        timer_ = create_wall_timer(10ms, std::bind(&two_steering_kinematics_node::timer_callback, this));
    }

private:
    void timer_callback()
    {

//      RCLCPP_INFO(this->get_logger(), "timer_callback"); // just to test

      auto message = geometry_msgs::msg::Twist();

     /* message.linear.x = 0.1;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = 0.1; */

      message = twist_measure();

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing Twist message\n\n");

      publisher_->publish(message);


      // --------------------------------------------------

      sensor_msgs::msg::JointState jointStateData;
      jointStateData.header.stamp = clock->now();
      jointStateData.name.push_back("front_wheel_steering");
      jointStateData.name.push_back("front_wheel");
      jointStateData.name.push_back("rear_wheel_steering");
      jointStateData.name.push_back("rear_wheel");
      jointStateData.position.push_back(beta_one_dot_control);
      jointStateData.position.push_back(v_control);
      jointStateData.position.push_back(beta_two_dot_control);
      jointStateData.position.push_back(v_control);

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing joint state data \n\n");

      publisher_jointState->publish(jointStateData);

    }

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
    {
        v_control = msg->data[0];
        beta_one_dot_control = msg->data[1];
        beta_two_dot_control = msg->data[2];
    }

    geometry_msgs::msg::Twist twist_measure()
    {
        auto message = geometry_msgs::msg::Twist();
        float beta1 = beta_one_dot_control;
        float beta2 = beta_two_dot_control;

       // static float beta_dot = beta_dot_control;
       // static float dt = 0.001; // 100 hz or 10ms
       //beta += dt*beta_dot;

        message.linear.x = 0.1;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = 0.1;

        return message;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_jointState;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_jointState;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<two_steering_kinematics_node>());
  rclcpp::shutdown();
  return 0;
}
