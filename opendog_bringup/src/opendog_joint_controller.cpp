#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>


#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


// using namespace std::chrono_literals;
using std::placeholders::_1;


class opendogGazeboJointCtrl : public rclcpp::Node
{   
    public:
        opendogGazeboJointCtrl()
        : Node("opendog_joint_ctrl_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "opendog_jointController/commands", 30, std::bind(&opendogGazeboJointCtrl::topic_callback, this, _1));
            
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gazebo_joint_controller/commands", 30);
        }
        
    
    private:
        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg_rx) const
        {   
            auto joint_angles = std_msgs::msg::Float64MultiArray();
            std::vector<double> angles[12];
            for(float ang : msg_rx->data){
                joint_angles.data.push_back(double(ang*M_PI/180));
            }


            // std::vector<double> constant_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> constant_angles = {-0.05, -0.0, -0.05, -0.05, -0.3, -0.7, 0.0, 0.3, -0.9, -0.0, 0.75, -0.6};
            
            // Ensure both vectors have the same size
            for (size_t i = 0; i < joint_angles.data.size(); ++i) {
                joint_angles.data[i] += constant_angles[i];  // Element-wise addition
            }

            // Print received joint angles before publishing
            RCLCPP_INFO(this->get_logger(), "Received Joint Angles (radians):");
            for (double angle : joint_angles.data) {
                RCLCPP_INFO(this->get_logger(), "%f", angle);
            }

            publisher_->publish(joint_angles);
        } 
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;    
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<opendogGazeboJointCtrl>());
    rclcpp::shutdown();
    return 0;
}


 