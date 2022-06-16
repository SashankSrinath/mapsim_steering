// Publisher for the 'cmd_vel'(Twist) topic. Subscribed by the simulation
// Publisher fo the joint states ( sensor_msgs).
// Subscribes to the 'cmd' topic (float32MultiArray) published by the slider_publisher

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

constexpr auto L{1.39};
constexpr auto r{0.95};
constexpr auto dt{0.01};

class bike_kinematics_node : public rclcpp::Node
{
public:
    bike_kinematics_node()
    : Node("bike_kinematics_node")
    {
        subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>("cmd", 1,
                                                                            [&](std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
                v_control = msg->data[0];
                beta_dot_control= msg->data[1];
    });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/bike/cmd_vel", 1);

        publisher_jointState = this->create_publisher<sensor_msgs::msg::JointState>("/bike/joint_states", 1);

        timer_ = create_wall_timer(10ms, std::bind(&bike_kinematics_node::timer_callback, this));

        state.name.push_back("frame_to_handlebar");
        state.name.push_back("handlebar_to_frontwheel");
        state.name.push_back("frame_to_backwheel");
        state.position = {0,0,0};

    }

private:
    void timer_callback()
    {

      auto message = geometry_msgs::msg::Twist();

      double &beta{state.position[0]};
      beta += dt*beta_dot_control;

      message.linear.x = v_control * cos(beta);
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = v_control*sin(beta)/L; // L = 1.39

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing Twist message \n\n");

      publisher_->publish(message);

      state.header.stamp = clock->now();
      state.position[1]+=dt*v_control/r;
      state.position[2]+=dt*v_control*cos(beta)/r;

      RCLCPP_INFO_ONCE(this->get_logger(), "\n\nPublishing joint state data \n\n");

      publisher_jointState->publish(state);

    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_jointState;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    float v_control, beta_dot_control;
    sensor_msgs::msg::JointState state;

};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bike_kinematics_node>());
  rclcpp::shutdown();
  return 0;
}
