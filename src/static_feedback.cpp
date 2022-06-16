#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

constexpr auto L{1.39};
constexpr auto dt{0.00001};
constexpr auto d{0.5};
constexpr auto a{-100};
constexpr auto kp{1};

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
        current_x = od_msg->pose.pose.position.x;
        current_y = od_msg->pose.pose.position.y;

    });

        publisher_feedback = this->create_publisher<std_msgs::msg::Float32MultiArray>("/bike/feedback_control", 1);

        timer_ = create_wall_timer(10ms, std::bind(&static_feedback_node::timer_callback, this));

        message.data = {0.0,0.0};
        angle = 0.0; x = 0; y = 0; xp = 0; yp = 0; x_dot = 0; y_dot = 0;

    }

private:
    void timer_callback()
    {
        x =  a * cos(angle);
        y =  a * sin(angle);

        x_dot = a * sin(angle);
        y_dot = a * cos(angle);

        angle += dt;
        if (angle > 2*M_PI){angle = 0.0;}

        xp = current_x + d*cos(current_beta);
        yp = current_y + d*sin(current_beta);

        x_error = x - xp;
        y_error = y - yp;

        K = {{cos(current_beta) - d/L*sin(current_beta)*sin(current_beta),   -d*sin(current_beta)},
             {sin(current_beta) + d/L*cos(current_beta)*sin(current_beta),    d*cos(current_beta)}};

        det_K = K[0][0]*K[1][1] - K[0][1]*K[1][0];

        inv_K = {{  K[1][1]/det_K, -K[0][1]/det_K},
                 { -K[1][0]/det_K,  K[0][0]/det_K}};

        v_control = (inv_K[0][0]*(x_dot+ kp*x_error)) + (inv_K[0][1]*(y_dot+kp*y_error));

        beta_dot_control = (inv_K[1][0]*(x_dot + kp*x_error)) + (inv_K[1][1]*(y_dot+kp*y_error));

        if (v_control>1.2){v_control = 1.2;}
        if (v_control<-1.2){v_control = -1.2;}

        if (beta_dot_control>1.2){beta_dot_control = 1.2;}
        if (beta_dot_control<-1.2){beta_dot_control = -1.2;}

        message.data[0] = v_control;
        message.data[1] = beta_dot_control;

        publisher_feedback->publish(message);

    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_jointState;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_feedback;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    double v_control, beta_dot_control, angle, det_K,x, y, xp, yp, x_dot, y_dot,x_error, y_error;
    double current_beta, current_x,current_y;
    std::vector<std::vector<double>> K, inv_K;
    std_msgs::msg::Float32MultiArray message;
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<static_feedback_node>());
  rclcpp::shutdown();
  return 0;
}
