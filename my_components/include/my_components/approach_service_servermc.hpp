#ifndef COMPOSITION__APPROACH_SERVICE_SERVER_HPP_
#define COMPOSITION__APPROACH_SERVICE_SERVER_HPP_


#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>
#include "custom_interfaces/srv/go_to_loading.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <thread>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace my_components
{

class ApproachService : public rclcpp::Node
{

    public:
        
        explicit ApproachService(const std::string& node_name);
    private:
    
        void get_approach_callback(std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
        std::vector<geometry_msgs::msg::Point> detect_legs(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
        rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
        rclcpp::CallbackGroup::SharedPtr scan_cb_group_;
        rclcpp::CallbackGroup::SharedPtr service_cb_group_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped frameConnection_;
        bool twolegs_;
        bool response_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;




};
}


#endif