#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>

namespace my_components
{

class PreApproach : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions & options);

private:
  enum class State
  {
    DRIVING,
    ROTATING,
    DONE
  };

  void timer_callback();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double normalize_angle(double angle);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist twist_;
  State state_ = State::DRIVING;

 
  const double obstacle_value_ = 1.2;
  const int degree_value_ = 30;

  float linearx_;
  float angularspeed_;
  double current_yaw_ = 0.0;
  double start_yaw_ = 0.0;
};

}  

#endif