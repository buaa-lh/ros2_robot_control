#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_math/robot_math.hpp"
namespace controller_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class ControllerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
    {
    public:
        using SharedPtr = std::shared_ptr<ControllerInterface>;
        virtual ~ControllerInterface() {}
        ControllerInterface();
        // 获取内部状态
        const std::vector<double> &get_internal_state() { return internal_state_; }

        // 写入内部状态
        // for simulation only
        void write_state(std::vector<double>::const_iterator s, std::vector<double>::const_iterator e)
        {
            std::copy(s, e, internal_state_.begin());
        }
        // 初始化节点
        int initialize(const std::string &name, const std::string &description,
                       const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                       bool lcn_service = false);

        void finalize();

        void loarn_interface(hardware_interface::CommandInterface *command,
                             const hardware_interface::StateInterface *state);

        const rclcpp_lifecycle::State &get_state() { return node_->get_current_state(); }

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() { return node_; }

        // 需要子类实现的虚函数
        virtual void update(const rclcpp::Time &t, const rclcpp::Duration &period) {};

        // 默认的虚函数实现
        // unconfigured -> inactive
        virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
        // inactive -> unconfigured
        virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
        // all -> finalized
        virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
        // inactive -> active
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        // active -> inactive
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        // all -> errorprocessing
        virtual CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

        template <typename ParameterT>
        auto auto_declare(const std::string &name, const ParameterT &default_value)
        {
            if (!node_->has_parameter(name))
            {
                return node_->declare_parameter<ParameterT>(name, default_value);
            }
            else
            {
                return node_->get_parameter(name).get_value<ParameterT>();
            }
        }

    protected:
        // 节点描述、节点
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::string description_;
        // 机器人对象、命令接口、状态接口指针、内部状态
        std::vector<std::string> joint_names_;
    robot_math::Robot robot_;
        hardware_interface::CommandInterface *command_;
        const hardware_interface::StateInterface *state_;
        std::vector<double> internal_state_; // e.g. integration of state
    };

} // namespace controller_interface

#endif // HARDWARE_INTERFACE_HPP