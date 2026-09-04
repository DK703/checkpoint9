#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "my_components/pre_approach_mc.hpp"

using namespace std::chrono_literals;

namespace my_components
{

//how we create the nodes

//this constructor has an initializer list. 
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: rclcpp::Node("pre_approach_node", options),
  obstacle_value_(this->declare_parameter<double>("obstacle", 0.4)),
  degree_value_(this->declare_parameter<int>("degrees", -90))
{

    //constructor function itself is PreApproach::PreApproach(const rclcpp::NodeOptions & options).
    //For these type of functions the part rclcpp::Node("pre_approach_node", options) is needed cuz  PreApproach inherits from node

    //To summarize, for these header functions, expect that I will consistently use this format, have the constructor function
    //and always an initlaizer list that containts the base class needed for inheritence


    //PreApproach::PreApproach(const rclcpp::NodeOptions & options) : rclcpp::Node("pre_approach_node", options){}


    //initializer list parts, rclcpp::Node("pre_approach_node", options) is needed to explicitly call one of its constructors.
    //construct base class with name pre_apporach_node
    //obstacle_value_ and degree_value are the parameters

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    
    timer_ = this->create_wall_timer(50ms, std::bind(&PreApproach::timer_callback, this));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos,
            std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
    
    linearx_ = 0.3;

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    state_ = State::DRIVING;

}
void PreApproach::timer_callback()
{

    switch(state_)
    {
        case State::DRIVING:
            {
                twist_.linear.x = linearx_;
                twist_.angular.z = angularspeed_;
                cmd_publisher_->publish(twist_);
                RCLCPP_INFO(this->get_logger(), "publisher runs with linearx_ = %f", linearx_);
                break;
            }
        case State::ROTATING:
            {
                linearx_ = 0.0;
                twist_.linear.x = linearx_;
                if (degree_value_ > 0) {
                    twist_.angular.z = 0.3;
                } 
                else {
                    twist_.angular.z = -0.3;
                }
                RCLCPP_INFO(this->get_logger(), "rotating");


                cmd_publisher_->publish(twist_);



                double rotated = std::abs(normalize_angle(current_yaw_ - start_yaw_));
                double target = std::abs(degree_value_ * M_PI / 180.0);

                if (rotated >= target)
                    {
                        RCLCPP_INFO(this->get_logger(),
        "Stopping rotation: rotated=%.4f rad (%.2f deg), target=%.4f rad (%.2f deg), overshoot=%.2f deg",
        rotated, rotated * 180.0 / M_PI,
        target, target * 180.0 / M_PI,
        (rotated - target) * 180.0 / M_PI);
                        RCLCPP_INFO(this->get_logger(), "moving to state done");
                        state_ = State::DONE;
                    }
                break;
            
            }
        case State::DONE:
        {
            linearx_ = 0.0;
            angularspeed_ = 0.0;
            twist_.linear.x = linearx_;
            twist_.angular.z = angularspeed_;
            cmd_publisher_->publish(twist_);
            //RCLCPP_INFO(this->get_logger(), "in state DONE");
            //rclcpp::shutdown(); //needed so robot shut down cleanly!
            break;
        }
    }

}

void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {  //300
        //double howmany = 0.0;
        std::vector<float>::iterator it;
        //float min_index = 150;
        float min_distance = 999999.0;
        //for (it = scan->ranges.begin(); it != scan->ranges.end(); ++it) {
  
        //howmany++;
        //}
       // RCLCPP_INFO(this->get_logger(), "how many=%f",howmany);

       //assumption on calculating the front
       for(int i = 125; i < 176; i++)
       {
       
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance) {
            min_distance = scan->ranges[i];
            //min_index = i;
            }

        
        //RCLCPP_INFO(this->get_logger(), "min index = %f min dinstance = %f" , min_index, min_distance);
        //RCLCPP_INFO(this->get_logger(), "obstacle_value_ = %f", obstacle_value_);
        if(min_distance < obstacle_value_ )
        {
        
            //RCLCPP_INFO(this->get_logger(), "stopping the robot");
            linearx_ = 0.0;
            //a dummy value so I can check that I can use parameters. I will have a proper algorithm
            //later when I can think about it.
            //angularspeed_ = degree_value_;
            if(state_ == State::DRIVING)
            {
                start_yaw_ = current_yaw_;  
                state_ = State::ROTATING;
            
            }

            
            //angular speed = angle of rotation/time


        
        }
       
       }


       // RCLCPP_INFO(this->get_logger(), "front=%f, right=%f left=%f back=%f",scan->ranges[150], scan->ranges[225], scan->ranges[75], scan->ranges[0]);
        
    }

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }


double PreApproach::normalize_angle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
    
    
        return angle;

    }

}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
