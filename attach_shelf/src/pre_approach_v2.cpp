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

using namespace std::chrono_literals;
class ApproachNode : public rclcpp::Node
{
public:
    ApproachNode() : Node("approach_node")
    {
    


    //declare parameter

    this->declare_parameter<double>("obstacle", 2);
    //this->declare_parameter("my_parameter", "world");
    this->declare_parameter<int>("degrees", 35.0);
    this->declare_parameter<bool>("final_approach", false);

    //parameters at startup
    obstacle_value = this->get_parameter("obstacle").as_double();
    degree_value = this->get_parameter("degrees").as_int();
    final_approach_value = this->get_parameter("final_approach").as_bool();



    
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    
    timer_ = this->create_wall_timer(50ms, std::bind(&ApproachNode::timer_callback, this));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos,
            std::bind(&ApproachNode::scan_callback, this, std::placeholders::_1));
    
    linearx_ = 0.3;

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&ApproachNode::odom_callback, this, std::placeholders::_1));


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

private:
    enum class State {  
        DRIVING,
        ROTATING,
        DONE
    };

    State state_ = State::DRIVING;


    void send_request()
    {
    
    justOneRequest_ = false;
    auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_value;

    RCLCPP_INFO(this->get_logger(), "sending request");


    auto result = client_->async_send_request(
            request,
            std::bind(&ApproachNode::response_callback, this, std::placeholders::_1));
    /*
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=  rclcpp::FutureReturnCode::SUCCESS)
    {
    RCLCPP_ERROR(this->get_logger(), "Failed");
    }
    else
    {
    RCLCPP_INFO(this->get_logger(), "service syccess");
    }
*/

    
    }

    void response_callback(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "response callback");

        auto response = future.get();

        //RCLCPP_INFO(this->get_logger(), "Read parameter final_approach = %s", final_approach_value ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "value of response->complete is %s", response->complete ? "true" : "false");
        //rclcpp::shutdown();
        //response_complete = response->complete;
        callbackDone_ = true;
        
    
    }


    void timer_callback()
    {

    switch(state_)
    {
        case State::DRIVING:
            {
                twist_.linear.x = linearx_;
                twist_.angular.z = angularspeed_;
                cmd_publisher_->publish(twist_);
                //RCLCPP_INFO(this->get_logger(), "publisher runs with linearx_ = %f", linearx_);
                break;
            }
        case State::ROTATING:
            {
                linearx_ = 0.0;
                twist_.linear.x = linearx_;
                if (degree_value > 0) {
                    twist_.angular.z = 0.3;
                } 
                else {
                    twist_.angular.z = -0.3;
                }
                //RCLCPP_INFO(this->get_logger(), "rotating");


                cmd_publisher_->publish(twist_);



                double rotated = std::abs(normalize_angle(current_yaw_ - start_yaw_));
                double target = std::abs(degree_value * M_PI / 180.0);

                if (rotated >= target)
                    {

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
           
            if(justOneRequest_){
            send_request();
            
            }
           
            if(callbackDone_ == true)
            {
                rclcpp::shutdown();
            }
            break;
        }
    }
    }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {  
        
        std::vector<float>::iterator it;
        
        float min_distance = 999999.0;


       //assumption on calculating the front
       for(int i = 125; i < 176; i++)
       {
       
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < min_distance) {
            min_distance = scan->ranges[i];
            //min_index = i;
            }

        
      
        if(min_distance < obstacle_value )
        {
        
            //RCLCPP_INFO(this->get_logger(), "stopping the robot");
            linearx_ = 0.0;
            //a dummy value so I can check that I can use parameters. I will have a proper algorithm
            //later when I can think about it.
            //angularspeed_ = degree_value;
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

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    double normalize_angle(double angle)
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



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

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
    

    
    

};

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachNode>();
    //simple_publisher = std::make_shared<PatrolNode>("PatrolNode", 1.0);
    //signal(SIGINT, signal_handler);
    //rclcpp::spin(simple_publisher);
    //rclcpp::shutdown();

    rclcpp::spin(node);
    //node->stop_robot();
    rclcpp::shutdown();


    return 0;
}