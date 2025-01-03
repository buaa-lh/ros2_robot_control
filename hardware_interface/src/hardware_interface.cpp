#include "hardware_interface/hardware_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace hardware_interface
{
    HardwareInterface::HardwareInterface()
    {
    }

    // 初始化节点
    int HardwareInterface::initialize(const std::string &name, const std::string &description, const std::string &name_space,
                                      const rclcpp::NodeOptions &options,
                                      bool lcn_service)
    {
        rclcpp::NodeOptions node_options(options);
        // 允许节点声明未知参数
        node_options.allow_undeclared_parameters(true);
        // 如果参数从命令行或其他配置源（如参数服务器）传递过来，会自动声明这些参数
        node_options.automatically_declare_parameters_from_overrides(true);
        description_ = description;
        // 创建一个生命周期节点
        // 传入节点名称、命名空间、节点选项、是否启用生命周期节点服务接口
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            name, name_space, node_options, lcn_service); // disable LifecycleNode service interfaces or, bad_alloc exception occur!

        // 将HardwareInterface类生命周期状态机中的不同状态回调函数与节点的生命周期管理器进行绑定
        // 当节点状态发生变化时，会回调相应的函数
        node_->register_on_configure(
            std::bind(&HardwareInterface::on_configure, this, std::placeholders::_1));

        node_->register_on_cleanup(
            std::bind(&HardwareInterface::on_cleanup, this, std::placeholders::_1));

        node_->register_on_activate(
            std::bind(&HardwareInterface::on_activate, this, std::placeholders::_1));

        node_->register_on_deactivate(
            std::bind(&HardwareInterface::on_deactivate, this, std::placeholders::_1));

        node_->register_on_shutdown(
            std::bind(&HardwareInterface::on_shutdown, this, std::placeholders::_1));

        node_->register_on_error(
            std::bind(&HardwareInterface::on_error, this, std::placeholders::_1));

        // 配置节点，使其进入inactive状态
        auto state = node_->configure();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_INFO(node_->get_logger(), "%s initialized", name.c_str());
            return 1;
        }
        return 0;
    }

    // 关闭节点
    void HardwareInterface::finalize()
    {
        node_->shutdown();
    }

    CallbackReturn HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HardwareInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }
}