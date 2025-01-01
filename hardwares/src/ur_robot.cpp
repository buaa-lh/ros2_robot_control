#include "hardware_interface/robot_interface.hpp"
#include <iostream>
#include <vector>

namespace hardwares
{
    class URRobot : public hardware_interface::RobotInterface
    {
    public:
        URRobot()
        {
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
        {
            if (RobotInterface::on_configure(previous_state) == CallbackReturn::SUCCESS)
            {
                node_->get_parameter_or<std::string>("robot_ip", robot_ip_, "");
                
            }
            
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
        {
            return CallbackReturn::SUCCESS;
        }

    protected:
        std::string robot_ip_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::URRobot, hardware_interface::RobotInterface)