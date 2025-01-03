#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/hardware_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_math/robot_math.hpp"
#include "urdf/model.h"
#include <functional>

namespace hardware_interface
{
    class RobotInterface : public HardwareInterface
    {
    public:
        ~RobotInterface() {}
        RobotInterface();
        // 根据URDF文件配置 robot_, joint_names_, dof_, state_, command_, state_names_, command_names_
        int configure_urdf(const std::string &robot_description);
        // 关闭和清理各个组件
        void finalize() override;

        const std::vector<std::string> &get_joint_names() { return joint_names_; }
        int get_dof() { return dof_; }
        const urdf::Model &get_urdf_model() { return robot_model_; }
        const robot_math::Robot &get_robot_model() { return robot_; }

        std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> get_all_nodes();
        void robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t,
                            std::function<Eigen::MatrixXd(double)> f_external,
                            std::function<std::vector<double>(double, const std::vector<double> &, const Eigen::MatrixXd &)> controller);
        void write(const rclcpp::Time &t, const rclcpp::Duration &period) override;
        // for simulation only
        void write_state(const std::vector<double> &state, const std::vector<double> &force)
        {
            std::copy(state.begin(), state.begin() + dof_, state_["position"].begin());
            std::copy(state.begin() + dof_, state.begin() + 2 * dof_, state_["velocity"].begin());
            state_["force"] = force;
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::vector<std::string> joint_names_;
        int dof_;
        urdf::Model robot_model_;
        robot_math::Robot robot_;
        // 组件map，包含各传感器和硬件
        std::map<std::string, hardware_interface::HardwareInterface::SharedPtr> components;
    };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP