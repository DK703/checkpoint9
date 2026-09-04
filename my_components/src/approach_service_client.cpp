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
#include "my_components/approach_service_client.hpp"

using namespace std::chrono_literals;

namespace my_components
{


AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("approach_client", options)
{


    //declare parameter

    //this->declare_parameter<double>("obstacle", 2);
    //this->declare_parameter("my_parameter", "world");
    //this->declare_parameter<int>("degrees", 35.0);
    //this->declare_parameter<bool>("final_approach", false);

    //parameters at startup
    //obstacle_value = this->get_parameter("obstacle").as_double();
    //degree_value = this->get_parameter("degrees").as_int();
    //final_approach_value = this->get_parameter("final_approach").as_bool();
    final_approach_value = true;



    
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    
    timer_ = this->create_wall_timer(50ms, std::bind(&AttachClient::timer_callback, this));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
    /*
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos,
            std::bind(&AttachClient::scan_callback, this, std::placeholders::_1));
  */  
    linearx_ = 0.3;
/*
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&AttachClient::odom_callback, this, std::placeholders::_1));
*/

    client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    current_yaw_ = 0.0;
    start_yaw_ = 0.0;
    callbackDone_ = false;

    




}


void AttachClient::send_request()
    {
    
    justOneRequest_ = false;
    auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_value;

    RCLCPP_INFO(this->get_logger(), "sending request");


    auto result = client_->async_send_request(
            request,
            std::bind(&AttachClient::response_callback, this, std::placeholders::_1));


    
    }

void AttachClient::response_callback(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "response callback");

        auto response = future.get();

        //RCLCPP_INFO(this->get_logger(), "Read parameter final_approach = %s", final_approach_value ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "value of response->complete is %s", response->complete ? "true" : "false");
        rclcpp::shutdown();
        //response_complete = response->complete;
        callbackDone_ = true;
        
    
    }

void AttachClient::timer_callback()
    {

    //may need to add a while loop
            
        linearx_ = 0.0;
        angularspeed_ = 0.0;
        twist_.linear.x = linearx_;
        twist_.angular.z = angularspeed_;
        cmd_publisher_->publish(twist_);
           
        if(justOneRequest_){
            send_request();
            
        }
           
        if(callbackDone_ == true)
        {
                RCLCPP_INFO(this->get_logger(), "calling rclcpp::shutdown()");
                //rclcpp::shutdown();
        }
        //break;
        
    
    }


}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)