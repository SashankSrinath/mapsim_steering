// Publisher for the 'cmd_vel'(Twist) topic. Subscribed by the simulation
// Publisher fo the joint states ( sensor_msgs). Subscribed
// Subscribes to the 'cmd' topic (float32MultiArray) published by the slider_publisher

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
//#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
//using std::placeholders::_1;

constexpr auto L{1.39};
constexpr auto dt{0.00001};
constexpr auto d{0.5};
constexpr auto a{10};

float current_beta;

class static_feedback_node : public rclcpp::Node
{
public:
    static_feedback_node()
    : Node("static_feedback_node")
    {
        subscriber_jointState = create_subscription<sensor_msgs::msg::JointState>("/bike/joint_states", 1,
                                                                            [&](sensor_msgs::msg::JointState::SharedPtr js_msg)
        {
        current_beta = js_msg->position[0];
    });
        subscriber_odom = create_subscription<nav_msgs::msg::Odometry>("/bike/odom", 1,
                                                                            [&](nav_msgs::msg::Odometry::SharedPtr od_msg)
        {

    });



        publisher_feedback_v = this->create_publisher<std_msgs::msg::Float32>("/bike/feedback_control_v", 1);
        publisher_feedback_betaDot = this->create_publisher<std_msgs::msg::Float32>("/bike/feedback_control_betaDot", 1);

        timer_ = create_wall_timer(10ms, std::bind(&static_feedback_node::timer_callback, this));

    }

private:
    void timer_callback()
    {
        x =  a * cos(angle);
        y =  a * sin(angle);

        angle += dt;
        if (angle > 2*M_PI){angle = 0.0;}

        x_dot = a * sin(angle);
        y_dot = a * cos(angle);

        xp = x + d*cos(current_beta);
        yp = y + d*sin(current_beta);

        K = {{cos(current_beta) - d/L*v_control*sin(current_beta)*sin(current_beta),   -d*sin(current_beta)},
             {sin(current_beta) + d/L*v_control*cos(current_beta)*sin(current_beta),    d*cos(current_beta)}};

        det_K = K[0][0]*K[1][1]-K[0][1]*K[1][0];

        inv_K = {{  K[1][1]/det_K, -K[0][1]/det_K},
                 { -K[1][0]/det_K,  K[0][0]/det_K}};

        v_control = inv_K[0][0]*x_dot + inv_K[0][1]*y_dot;
        beta_dot_control = inv_K[1][0]*x_dot + inv_K[1][1]*y_dot;

    auto message_v = std_msgs::msg::Float32();
    message_v.data = v_control;
    publisher_feedback_v->publish(message_v);

    auto message_betaDot = std_msgs::msg::Float32();
    message_betaDot.data = beta_dot_control;
    publisher_feedback_betaDot->publish(message_betaDot);
    }


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_jointState;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_feedback_v;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_feedback_betaDot;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    double v_control, beta_dot_control, angle, det_K,x, y, xp, yp, x_dot, y_dot;
    std::vector<std::vector<double>> K, inv_K;

};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<static_feedback_node>());
  rclcpp::shutdown();
  return 0;
}
