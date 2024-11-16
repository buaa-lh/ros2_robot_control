#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control_node_parameters.hpp"

namespace control_node
{

    class ControlManager: public rclcpp::Node
    {
        public:
            ControlManager(std::shared_ptr<rclcpp::Executor> executor, const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option);
            int get_update_rate();
        protected:
            std::shared_ptr<rclcpp::Executor> executor_;
            int update_rate_;
            Params params_;

    };



}