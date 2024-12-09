#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control_node_parameters.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "std_msgs/msg/string.hpp"
#include "robot_math/robot_math.hpp"

namespace control_node
{

    class ControlManager: public rclcpp::Node
    {
        public:
            ControlManager(std::shared_ptr<rclcpp::Executor> executor, const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option);
            ~ControlManager();
            int get_update_rate();
            virtual void init_robot();
            virtual void shutdown_robot();
            virtual void read(const rclcpp::Time &t, const rclcpp::Duration & period);
            virtual void update(const rclcpp::Time &t, const rclcpp::Duration & period);
            virtual void write(const rclcpp::Time &t, const rclcpp::Duration & period);
        protected:
            void robot_description_callback(std_msgs::msg::String::SharedPtr desp);
            void config_robot();
        protected:
            std::shared_ptr<rclcpp::Executor> executor_;
            int update_rate_;
            Params params_;
            std::string robot_description_;
            std::shared_ptr<ParamListener> param_listener_;
            realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> real_time_buffer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
            std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> real_time_publisher_;
            urdf::Model robot_model_;
            robot_math::Robot robot;
            std::vector<std::string> joint_names_;
            std::mutex robot_desp_mutex_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_position_;

    };



}