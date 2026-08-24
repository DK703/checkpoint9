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

using namespace std::chrono_literals;

//from geometry_msgs.msg import TransformStamped
//from nav_msgs.msg import Odometry



//if attach_to_shelf is false, final approach will not happen. that being said, cart_frame will still be published!
class ApproachService : public rclcpp:: Node
{
//one leg-2-4 8000 values?
public:
    ApproachService(const std::string& node_name) : Node(node_name)
    {
    
        scan_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      

        rclcpp::SubscriptionOptions scan_options;
        scan_options.callback_group = scan_cb_group_;


        std::string name_service = "/approach_shelf";

        service_ = this->create_service<custom_interfaces::srv::GoToLoading>(
        name_service,std::bind(&ApproachService::get_approach_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_cb_group_
        );


        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ApproachService::scan_callback, this, std::placeholders::_1),
        scan_options);

          tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

          // Buffer needs the node's clock so it can time-stamp/interpolate transforms
          tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
          // Listener writes incoming /tf data into the buffer
          tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
          response_ = false;


            client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
      


    
    }

private:
    void get_approach_callback(std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request, std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response)
    {
    //publish the cart_frame_/frameConnection_ no matter what. 

     RCLCPP_INFO(this->get_logger(), "service has recieved a request and a response");
     response->complete = response_;
    
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
    
        //set the frameConnection parameters that dont rely on the leg # at the start
        frameConnection_.header.stamp = this->get_clock()->now();
        frameConnection_.header.frame_id = scan_msg->header.frame_id;
        frameConnection_.child_frame_id = "cart_frame";
        RCLCPP_INFO(this->get_logger(), "scan runs");
        std::vector<geometry_msgs::msg::Point> v = detect_legs(scan_msg);

        RCLCPP_INFO(this->get_logger(), "Detected %zu legs:", v.size());
        int idx = 0;
        //printout statement
        for (const auto& p : v) {
            RCLCPP_INFO(this->get_logger(), "  Leg %d: x=%f, y=%f, z=%f", idx++, p.x, p.y, p.z);
            RCLCPP_INFO(this->get_logger(), "p is %f", p);

  

            
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
            RCLCPP_INFO(this->get_logger(), "  x1 %f: y1 %f, x2 %f, y2 %f", firstx, firsty, secondx, secondy);
            float midx = ((firstx+secondx)/2);
            float midy = ((firsty+secondy)/2);
            frameConnection_.transform.translation.x = midx;
            frameConnection_.transform.translation.y = midy;
            frameConnection_.transform.rotation.x = 0;
            frameConnection_.transform.rotation.y = 0;
            frameConnection_.transform.rotation.z = 0;
            frameConnection_.transform.rotation.w = 1;
            tf_broadcaster_->sendTransform(frameConnection_);
            

            RCLCPP_INFO(this->get_logger(), "midx %f midy %f", midx, midy);

            //need to change this to be true only when final approach is succesful, not when 2 legs detected
            response_ = true;

        
        }
        else
        {
            response_ = false;
        
        }
     
 
        
        

        


    
    }


    std::vector<geometry_msgs::msg::Point> detect_legs(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
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
    rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
    rclcpp::CallbackGroup::SharedPtr scan_cb_group_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped frameConnection_;
    bool response_;
    rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
    



};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>("approach_service_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}