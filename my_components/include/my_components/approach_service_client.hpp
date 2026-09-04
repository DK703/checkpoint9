#ifndef COMPOSITION__APPROACH_SERVICE_CLIENT_HPP_
#define COMPOSITION__APPROACH_SERVICE_CLIENT_HPP_

#include "my_components/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "custom_interfaces/srv/go_to_loading.hpp"

namespace my_components
{

class AttachClient : public rclcpp::Node
{
    public:
        COMPOSITION_PUBLIC
        explicit AttachClient(const rclcpp::NodeOptions & options);
    private:
        void send_request();
        void response_callback(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture future);
        void timer_callback();
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
        //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist twist_;
        //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

        rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
        rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
        double obstacle_value;
        int degree_value;
        float linearx_;
        float angularspeed_;
        double current_yaw_;
        double start_yaw_;
        bool final_approach_value = false;
        bool justOneRequest_ = true;
        bool callbackDone_;
        const double obstacle_value_ = 1.2;
        const int degree_value_ = 30;


};

}
#endif