
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
#include "my_components/approach_service_servermc.hpp"
#include "my_components/visibility_control.h"
using namespace std::chrono_literals;

namespace my_components
{


ApproachService::ApproachService(const std::string& node_name) : Node(node_name)
    {
        
        
    //two separate callback groups that dont interfere with each other
    scan_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_cb_group_;


    std::string name_service = "/approach_shelf";
    //receives request from pre_approach_v2
    service_ = this->create_service<custom_interfaces::srv::GoToLoading>(
    name_service,std::bind(&ApproachService::get_approach_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_cb_group_
    );


    //use this subscriber to help figure out odom point
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&ApproachService::scan_callback, this, std::placeholders::_1),
    scan_options);
          

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Buffer needs the node's clock so it can time-stamp/interpolate transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Listener writes incoming /tf data into the buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    response_ = false;
        
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_up_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    }

void ApproachService::get_approach_callback(std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response)
    {

    RCLCPP_INFO(this->get_logger(), "service has recieved a request and a response");
    
     //how it knows to get the request
     if(request->attach_to_shelf == true)
     {
            rclcpp::Rate rate(10);  // 10 Hz control loop
        while (rclcpp::ok()) {
                geometry_msgs::msg::TransformStamped t;
                try {
                    t = tf_buffer_->lookupTransform("robot_base_footprint", "cart_frame", tf2::TimePointZero);
                } 
                catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                    rate.sleep();
                    continue;
                }

                double error_x = t.transform.translation.x;
                double error_y = t.transform.translation.y;
                double distance = std::sqrt(error_x * error_x + error_y * error_y);

                RCLCPP_INFO(this->get_logger(), "error is %f", distance);

                if (distance < 0.5) {  // close enough threshold
                    break;
                }

                RCLCPP_INFO(this->get_logger(), "still in while");
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = std::min(0.15, distance * 0.5);   // simple proportional control, capped
                cmd.angular.z = std::atan2(error_y, error_x) * 1.0;  // steer toward the target
                cmd_vel_publisher_->publish(cmd);

                rate.sleep();
            }
//while loop finished already

                geometry_msgs::msg::Twist stop;
                cmd_vel_publisher_->publish(stop);
                
                //a second geometry_msgs::msg::Twist command to move the robot 30 feet!
                RCLCPP_INFO(this->get_logger(), "30 feet mode");

                geometry_msgs::msg::Twist cmd2;
                cmd2.linear.x = 0.43;
                //cmd_vel_publisher_->publish(cmd2);
                //sleep(3);
                RCLCPP_INFO(this->get_logger(), "begin wait");
                
                
      
                std::chrono::time_point<std::chrono::_V2::steady_clock> start;

                start = std::chrono::_V2::steady_clock::now();
                std::chrono::time_point<std::chrono::_V2::steady_clock> end = std::chrono::_V2::steady_clock::now();
                
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
                //twist messages need to be in a loop for the commands to run.
                while (elapsed.count() < 3)
                {
                
                //RCLCPP_INFO(this->get_logger(), "time is %d", elapsed.count());
                end = std::chrono::_V2::steady_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
                cmd_vel_publisher_->publish(cmd2);
                }
                
                //ros2 topic pub /elevator_up std_msgs/msg/String --once
                

                std_msgs::msg::String el_up; 
                el_up.data = "up";

                std::chrono::time_point<std::chrono::_V2::steady_clock> start2 = std::chrono::_V2::steady_clock::now();
                std::chrono::time_point<std::chrono::_V2::steady_clock> end2 = std::chrono::_V2::steady_clock::now();
                auto elapsed2 = std::chrono::duration_cast<std::chrono::seconds>(end2 - start2);



                //does not need a loop
                elevator_up_publisher_->publish(el_up);

                //for safety
                while(elapsed2.count() < 2)
                {
                    //RCLCPP_INFO(this->get_logger(), "time is %d", elapsed2.count());
                    end2 = std::chrono::_V2::steady_clock::now();
                    elapsed2 = std::chrono::duration_cast<std::chrono::seconds>(end2 - start2);
                    cmd_vel_publisher_->publish(stop);
                }




                response_ = true;


     }
     //response->complete = true;
       response->complete = response_;
       rclcpp::shutdown();

    }

void ApproachService::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

        //set the frameConnection parameters that dont rely on the leg # at the start
        frameConnection_.header.stamp = this->get_clock()->now();
        frameConnection_.header.frame_id = "odom";
        frameConnection_.child_frame_id = "cart_frame";
        //RCLCPP_INFO(this->get_logger(), "scan runs");
        std::vector<geometry_msgs::msg::Point> v = detect_legs(scan_msg);


        int idx = 0;
        //printout statement
        for (const auto& p : v) {
 

  

            
        }
    
        if(v.size() == 2)
        {
            float firstx;
            float firsty;
            float secondx;
            float secondy;
            idx = 0;

            for (const auto& p : v)
            {
            
                if(idx == 0)
                {
                    firstx = p.x;
                    firsty = p.y;
                }
                else if(idx == 1)
                {
                    secondx = p.x;
                    secondy = p.y;
                }
                idx++;
                
            
            }
            //RCLCPP_INFO(this->get_logger(), "  x1 %f: y1 %f, x2 %f, y2 %f", firstx, firsty, secondx, secondy);
            float midx = ((firstx+secondx)/2);
            float midy = ((firsty+secondy)/2);

            //point built in laser frame
            geometry_msgs::msg::PointStamped laser_point;
            laser_point.header.stamp = scan_msg->header.stamp;
            laser_point.header.frame_id = scan_msg->header.frame_id;
            laser_point.point.x = midx;
            laser_point.point.y = midy;
            laser_point.point.z = 0.0;

            try
            {
                    geometry_msgs::msg::PointStamped odom_point =
                    tf_buffer_->transform(laser_point, "odom", tf2::durationFromSec(0.1));
                    //tf convert point to odom frame.
                    frameConnection_.transform.translation.x = odom_point.point.x;
                    frameConnection_.transform.translation.y = odom_point.point.y;
                    frameConnection_.transform.translation.z = 0.0;
                    frameConnection_.transform.rotation.x = 0;
                    frameConnection_.transform.rotation.y = 0;
                    frameConnection_.transform.rotation.z = 0;
                    frameConnection_.transform.rotation.w = 1;

                    tf_broadcaster_->sendTransform(frameConnection_);
            }
            catch(const tf2::TransformException& ex)
            {
                    RCLCPP_WARN(this->get_logger(), "Could not transform leg midpoint to odom");
            }
 
            

            //RCLCPP_INFO(this->get_logger(), "midx %f midy %f", midx, midy);

        

        
        }
        else
        {
            response_ = false;
        
        }
     
}

std::vector<geometry_msgs::msg::Point> ApproachService::detect_legs(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    std::vector<geometry_msgs::msg::Point> legs;
    std::vector<int> current_cluster;
    const float intensity_threshold = 3000.0;

        auto finalize_cluster = [&]() {
            if (current_cluster.empty()) return;
            double sum_x = 0.0, sum_y = 0.0;
            for (int idx : current_cluster) {
                double angle = scan->angle_min + idx * scan->angle_increment;
                double r = scan->ranges[idx];
                sum_x += r * std::cos(angle);
                sum_y += r * std::sin(angle);
            }
            double n = current_cluster.size();

            geometry_msgs::msg::Point p;
            p.x = sum_x / n;
            p.y = sum_y / n;
            p.z = 0.0;
            legs.push_back(p);

            current_cluster.clear();
        };

            for (size_t i = 0; i < scan->intensities.size(); ++i) {
                if (scan->intensities[i] > intensity_threshold) {
                    current_cluster.push_back(i);
                    } else {
                    finalize_cluster();
                    }
                }
                finalize_cluster();

                return legs;

}

}


#include "rclcpp_components/register_node_macro.hpp"
//not needed for manual composition
//RCLCPP_COMPONENTS_REGISTER_NODE(my_components::ApproachService)