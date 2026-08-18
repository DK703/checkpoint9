#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include "sensor_msgs/msg/laser_scan.hpp"
using namespace std::chrono_literals;
class ApproachNode : public rclcpp::Node
{
public:
    ApproachNode() : Node("approach_node")
    {
    


    //declare parameter

    this->declare_parameter<double>("obstacle", 1.2);
    //this->declare_parameter("my_parameter", "world");
    this->declare_parameter<double>("degrees", 30.0);

    //parameters at startup
    obstacle_value = this->get_parameter("obstacle").as_double();
    degree_value = this->get_parameter("degrees").as_double();



    
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    
    timer_ = this->create_wall_timer(1000ms, std::bind(&ApproachNode::timer_callback, this));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);  
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos,
            std::bind(&ApproachNode::scan_callback, this, std::placeholders::_1));
    
    linearx_ = 0.3;



    
    }

private:


    void timer_callback()
    {
    twist_.linear.x = linearx_;
    twist_.angular.z = angularspeed_;
    cmd_publisher_->publish(twist_);
    RCLCPP_INFO(this->get_logger(), "publisher runs with linearx_ = %f", linearx_);

    
    }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {  //300
        double howmany = 0.0;
        std::vector<float>::iterator it;
        float min_index = 150;
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
            min_index = i;
            }

        
        //RCLCPP_INFO(this->get_logger(), "min index = %f min dinstance = %f" , min_index, min_distance);
        //RCLCPP_INFO(this->get_logger(), "obstacle_value = %f", obstacle_value);
        if(min_distance < obstacle_value )
        {
        
            RCLCPP_INFO(this->get_logger(), "stopping the robot");
            linearx_ = 0.0;
            //a dummy value so I can check that I can use parameters. I will have a proper algorithm
            //later when I can think about it.
            angularspeed_ = degree_value;

            
            //angular speed = angle of rotation/time


        
        }
       
       }


       // RCLCPP_INFO(this->get_logger(), "front=%f, right=%f left=%f back=%f",scan->ranges[150], scan->ranges[225], scan->ranges[75], scan->ranges[0]);
        
    }



    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_;
    double obstacle_value;
    double degree_value;
    float linearx_;
    float angularspeed_;

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