#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <mutex>

class PandaMoveIt : public rclcpp::Node
{
public:
    PandaMoveIt()
    : Node("panda_moveit_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        joint_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/joint_angles", 10,
            std::bind(&PandaMoveIt::joint_angles_callback, this, std::placeholders::_1)
        );
        moveit_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PandaMoveIt::planAndExecute, this));
        moveit_mutex_ = std::make_shared<std::mutex>();
        success_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/moveit_success", 10);
    }

    void joint_angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(*moveit_mutex_);
        joint_values_.assign(msg->data.begin(), msg->data.end());
    }

    void planAndExecute()
    {
        auto move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        std::lock_guard<std::mutex> lock(*moveit_mutex_);
        if (joint_values_.size() != 7)
        {
            RCLCPP_WARN(this->get_logger(), "Received joint values do not have 7 elements.");
            return;
        }

        move_group_interface_->setJointValueTarget(joint_values_);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing movement.");
            move_group_interface_->execute(plan);
            
            // Publish success message
            std_msgs::msg::Bool success_msg;
            success_msg.data = true;
            success_publisher_->publish(success_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            
            // Publish failure message
            std_msgs::msg::Bool success_msg;
            success_msg.data = false;
            success_publisher_->publish(success_msg);
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_angle_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::vector<double> joint_values_;
    rclcpp::TimerBase::SharedPtr moveit_timer_;
    std::shared_ptr<std::mutex> moveit_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaMoveIt>();
    RCLCPP_INFO(node->get_logger(), "Panda MoveIt Node has started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
