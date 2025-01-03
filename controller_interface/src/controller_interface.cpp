#include "controller_interface/controller_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
namespace controller_interface
{
    ControllerInterface::ControllerInterface() : command_(nullptr), state_(nullptr)
    {
    }
    int ControllerInterface::initialize(const std::string &name, const std::string &description,
                                        const std::string &name_space, const rclcpp::NodeOptions &options,
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
            name, name_space, node_options, lcn_service); // disable LifecycleNode service interfaces

        // 将ControllerInterface类生命周期状态机中的不同状态回调函数与节点的生命周期管理器进行绑定
        // 当节点状态发生变化时，会回调相应的函数
        node_->register_on_configure(
            std::bind(&ControllerInterface::on_configure, this, std::placeholders::_1));

        node_->register_on_cleanup(
            std::bind(&ControllerInterface::on_cleanup, this, std::placeholders::_1));

        node_->register_on_activate(
            std::bind(&ControllerInterface::on_activate, this, std::placeholders::_1));

        node_->register_on_deactivate(
            std::bind(&ControllerInterface::on_deactivate, this, std::placeholders::_1));

        node_->register_on_shutdown(
            std::bind(&ControllerInterface::on_shutdown, this, std::placeholders::_1));

        node_->register_on_error(
            std::bind(&ControllerInterface::on_error, this, std::placeholders::_1));

        auto state = node_->configure();
        if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            RCLCPP_INFO(node_->get_logger(), "%s initilized!", name.c_str());
            return 1;
        }

        return 0;
    }

    void ControllerInterface::loarn_interface(hardware_interface::CommandInterface *command,
                                              const hardware_interface::StateInterface *state)
    {
        command_ = command;
        state_ = state;
    }

    void ControllerInterface::finalize()
    {
        node_->shutdown();
    }

    CallbackReturn ControllerInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!description_.empty())
        {
            robot_ = robot_math::urdf_to_robot(description_);
            return CallbackReturn::SUCCESS;
        }
        return CallbackReturn::ERROR;
    }

    CallbackReturn ControllerInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ControllerInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

} // namespace controller_interface
