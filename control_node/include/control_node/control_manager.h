#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control_node_parameters.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
namespace control_node
{

    class ControlManager: public rclcpp::Node
    {
        public:
            ControlManager(std::shared_ptr<rclcpp::Executor> executor, const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option);
            int get_update_rate();
            void read(const rclcpp::Time &t, const rclcpp::Duration & period);
            void update(const rclcpp::Time &t, const rclcpp::Duration & period);
            void write(const rclcpp::Time &t, const rclcpp::Duration & period);
        protected:
            std::shared_ptr<rclcpp::Executor> executor_;
            int update_rate_;
            Params params_;
            std::string robot_description_;
            std::shared_ptr<ParamListener> param_listener_;
            realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> real_time_buffer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> real_time_publisher_;
            urdf::Model robot_model_;

    };



}