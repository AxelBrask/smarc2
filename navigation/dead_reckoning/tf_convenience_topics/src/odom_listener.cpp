#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "dead_reckoning_msgs/msg/Topics.hpp"

std_msgs::msg::Float64 current_roll, current_pitch, current_yaw, current_depth, current_x, current_y, current_u, current_v, current_w, current_p, current_q, current_r, current_alt;
double r,p,y;
tf2::Quaternion tfq;

void DVLCallback(const smarc_msgs::msg::DVL::SharedPtr dvl_msg)
{
    current_alt.data = dvl_msg->altitude;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[ odom_listener]  Altitude from DVL: %f", dvl_msg->altitude);
}

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    tf2::fromMsg(odom_msg->pose.pose.orientation, tfq);

    tf2::Matrix3x3(tfq).getEulerYPR(y,p,r);
    //orientation
    current_pitch.data= p;
    current_roll.data= r;
    current_yaw.data= y;
    current_depth.data= -odom_msg->pose.pose.position.z;
    current_x.data= odom_msg->pose.pose.position.x;
    current_y.data= odom_msg->pose.pose.position.y;

    //Velocity
    current_u.data= odom_msg->twist.twist.linear.x;
    current_v.data= odom_msg->twist.twist.linear.y;
    current_w.data= odom_msg->twist.twist.linear.z;
    current_p.data= odom_msg->twist.twist.angular.x;
    current_q.data= odom_msg->twist.twist.angular.y;
    current_r.data= odom_msg->twist.twist.angular.z;
}

int main(int argc, char** argv){

  std::string node_name = "odom_listener";
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(node_name);

  std::string odom_topic_;
  std::string dvl_topic_;
  double freq;

  auto node_priv = std::make_shared<rclcpp::Node>("odom_listener");
  node_priv->declare_parameter<std::string>("odom_topic", "/sam/dr/local/odom/filtered");
  node_priv->declare_parameter<std::string>("dvl_topic", "/sam/core/dvl");
  node_priv->declare_parameter<double>("loop_freq", 10);

  node_priv->get_parameter("odom_topic", odom_topic_);
  node_priv->get_parameter("dvl_topic", dvl_topic_);
  node_priv->get_parameter("loop_freq", freq);

  auto dvl_sub = node->create_subscription<smarc_msgs::msg::DVL>(dvl_topic_, 10, DVLCallback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, OdomCallback);
  
  auto feedback_pitch = node->create_publisher<std_msgs::msg::Float64>("/pitch", 10);
    auto feedback_roll = node->create_publisher<std_msgs::msg::Float64>("/roll", 10);
    auto feedback_yaw = node->create_publisher<std_msgs::msg::Float64>("/yaw", 10);
    auto feedback_depth = node->create_publisher<std_msgs::msg::Float64>("/depth", 10);
    auto feedback_x = node->create_publisher<std_msgs::msg::Float64>("/x", 10);
    auto feedback_y = node->create_publisher<std_msgs::msg::Float64>("/y", 10);
    auto feedback_u = node->create_publisher<std_msgs::msg::Float64>("/u", 10);
    auto feedback_v = node->create_publisher<std_msgs::msg::Float64>("/v", 10);
    auto feedback_w = node->create_publisher<std_msgs::msg::Float64>("/w", 10);
    auto feedback_p = node->create_publisher<std_msgs::msg::Float64>("/p", 10);
    auto feedback_q = node->create_publisher<std_msgs::msg::Float64>("/q", 10);
    auto feedback_r = node->create_publisher<std_msgs::msg::Float64>("/r", 10);
    auto feddback_alt = node->create_publisher<std_msgs::msg::Float64>("/alt", 10);
    rclcpp::Rate rate(10.0);
    while(rclcpp::ok())
    {
        feedback_pitch->publish(current_pitch);
        feedback_roll->publish(current_roll);
        feedback_yaw->publish(current_yaw);
        feedback_depth->publish(current_depth);
        feedback_x->publish(current_x);
        feedback_y->publish(current_y);
        feedback_u->publish(current_u);
        feedback_v->publish(current_v);
        feedback_w->publish(current_w);
        feedback_p->publish(current_p);
        feedback_q->publish(current_q);
        feedback_r->publish(current_r);
        feddback_alt->publish(current_alt);
        ROS_INFO_THROTTLE(1.0, "[ odom_listener ] roll: %f, pitch: %f, yaw: %f, depth: %f, x: %f, y: %f, u: %f, v: %f, w: %f, p: %f, q: %f, r: %f alt:%f", current_roll.data,current_pitch.data,current_yaw.data,current_depth.data, current_x.data, current_y.data, current_u.data, current_v.data, current_w.data, current_p.data, current_q.data, current_r.data, current_alt.data);

        rate.sleep();
        rclcpp::spin_some(node);
    }
  return 0;
}