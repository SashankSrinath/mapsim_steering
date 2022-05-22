#include <iostream>
#include <sensor_msgs/msg/joint_state.h>

// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Publisher for the 'cmd_vel'(Twist) topic. Subscribed by the simulation
// Subscribes to the 'cmd' topic (float32MultiArray) published by the slider_publisher

//To do - The kinematic model for the 1,1 robot
// joint state publisher from the slider publisher commands


std_msgs::msg::Float32MultiArray::SharedPtr get_msg;
float v_x, w_z;

class fwd_kinematics_node : public rclcpp::Node
{
public:
    fwd_kinematics_node()
    : Node("fwd_kinematics_node")
    {
        subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("cmd", 1,
                    std::bind(&fwd_kinematics_node::topic_callback, this, _1));

        publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("/bike/cmd_vel", 1);

        timer_ = create_wall_timer(10ms, std::bind(&fwd_kinematics_node::timer_callback, this));
    }

private:
    void timer_callback()
    {

      //RCLCPP_INFO(this->get_logger(), "timer_callback"); // just to test

      auto message = geometry_msgs::msg::Twist();
      message.linear.x =1.0;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = 0.1;

      //message = velocity_measure();
      //v_x = message.linear.x;
      //w_z = message.angular.z;

     // RCLCPP_INFO(this->get_logger(), "message.linear.x: '%f'", v_x);
      //RCLCPP_INFO(this->get_logger(), "message.angular.z: '%f'", w_z);

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing: '%f'\n\n", message.linear.x);
      //RCLCPP_INFO_ONCE(this->get_logger(), "\n\n wz: '%f'\n\n", message.angular.z);

      publisher_->publish(message);

    }

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
    {
        get_msg = msg;
       // RCLCPP_INFO(this->get_logger(), "v_control: '%f'", get_msg->data[0]);
       // RCLCPP_INFO(this->get_logger(), "beta_dot_control: '%f'", get_msg->data[1]);
    }

    geometry_msgs::msg::Twist velocity_measure()
    {
        auto message = geometry_msgs::msg::Twist();
        static float v = get_msg->data[0];
        static float beta = 0.0;
        static float dt = 0.01; // 100 hz or 10ms
        beta += dt*get_msg->data[1];
        message.linear.x = v*cos(beta);
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = v*sin(beta)/0.95; // L = 0.95
        return message;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fwd_kinematics_node>());
  rclcpp::shutdown();
  return 0;
}




/*
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
        {
          fwd_kinematics_node::v_control = msg->data[0];
          fwd_kinematics_node::beta_dot_control = msg->data[1];
          RCLCPP_INFO(this->get_logger(), "v_control: '%f'", fwd_kinematics_node::v_control);
          RCLCPP_INFO(this->get_logger(), "beta_dot_control: '%f'", fwd_kinematics_node::beta_dot_control);
        }
*/
