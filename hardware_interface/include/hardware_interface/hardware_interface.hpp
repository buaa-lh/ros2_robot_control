#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace hardware_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HardwareInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
    {
    public:
        using SharedPtr = std::shared_ptr<HardwareInterface>;
        virtual ~HardwareInterface() {}
        HardwareInterface();
        // 初始化节点
        int initialize(const std::string &name, const std::string &description, const std::string &name_space = "",
                       const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                       bool lcn_service = false);

        // 需要子类实现的虚函数
        virtual void finalize();
        virtual void read(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) {}
        virtual void write(const rclcpp::Time & /*t*/, const rclcpp::Duration & /*period*/) {}
        const rclcpp_lifecycle::State &get_state() { return node_->get_current_state(); }

        // 获取节点的命令接口、状态接口、节点名称、节点
        CommandInterface &get_command_interface() { return command_; }
        const StateInterface &get_state_interface() { return state_; }
        const std::vector<std::string> &get_state_names() { return state_names_; }
        const std::vector<std::string> &get_command_names() { return command_names_; }
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() { return node_; }

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

    protected:
        // 节点描述、节点
        std::string description_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        // 状态、命令string
        std::vector<std::string> state_names_;
        std::vector<std::string> command_names_;
        // 状态、命令接口
        CommandInterface command_;
        StateInterface state_;
    };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP