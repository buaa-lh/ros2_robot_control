#include "control_node/control_manager.h"



namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                                 const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option), 
          executor_(executor), 
          param_listener_(std::make_shared<ParamListener>(this->get_node_parameters_interface()))
    {
    
        params_ = param_listener_->get_params();
        update_rate_ = params_.update_rate;
        robot_description_ = this->get_parameter("robot_description").as_string();
        robot_model_.initString(robot_description_);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }
}

void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
{
    auto states = std::make_shared<sensor_msgs::msg::JointState>();
    states->name = {"fr3_joint1", "fr3_joint2" ,"fr3_joint3" ,"fr3_joint4", "fr3_joint5" ,"fr3_joint6", "fr3_joint7"};
    states->position = {std::sin(2 * t.seconds()),std::cos(2 * t.seconds()),0,0,0,0,0};
    states->header.stamp = t;
    if(real_time_publisher_->trylock())
    {
        real_time_publisher_->msg_ = *states;
        real_time_publisher_->unlockAndPublish();
    }
    
    
}

void control_node::ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
{
}

void control_node::ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
{

}
