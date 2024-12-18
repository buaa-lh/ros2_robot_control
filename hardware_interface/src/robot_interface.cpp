#include "hardware_interface/robot_interface.hpp"

namespace hardware_interface
{

    RobotInterface::RobotInterface() : dof_(0)
    {
    }

    int RobotInterface::configure_urdf(const std::string &robot_description)
    {
        if (!robot_description.empty() && robot_model_.initString(robot_description))
        {
            robot_ = robot_math::urdf_to_robot(robot_description);
            for (auto j : robot_model_.joints_)
            {
                if (j.second->type != urdf::Joint::FIXED)
                {
                    joint_names_.push_back(j.first);
                }
            }
            dof_ = joint_names_.size();
            state_names_.emplace_back("position");
            state_names_.emplace_back("velocity");
            state_names_.emplace_back("torque");

            command_names_.emplace_back("position");
            command_names_.emplace_back("velocity");
            command_names_.emplace_back("torque");
            for (auto &name : state_names_)
            {
                state_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            for (auto &name : command_names_)
            {
                command_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            state_.emplace("force", std::vector<double>(6, 0.0));
            return 1;
        }
        return 0;
    }
    

    CallbackReturn RobotInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        if (configure_urdf(description_))
            return CallbackReturn::SUCCESS;
        else
            return CallbackReturn::ERROR;
    }

    CallbackReturn RobotInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotInterface::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
}