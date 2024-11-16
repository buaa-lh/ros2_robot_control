#include "control_node/control_manager.h"


namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                                 const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option), executor_(executor)
    {
       ;
        auto param_listener = std::make_shared<ParamListener>(this->get_node_parameters_interface());
        params_ = param_listener->get_params();
        update_rate_ = params_.update_rate;
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }
}
