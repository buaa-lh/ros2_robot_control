#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_math/robot_math.hpp"
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
namespace controller_interface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class ControllerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  {
  public:
    using SharedPtr = std::shared_ptr<ControllerInterface>;
    virtual ~ControllerInterface() {}
    ControllerInterface();
    const std::vector<double> & get_internal_state() { return internal_state_;} 
    // for simulation only
    void write_state(std::vector<double>::const_iterator s,  std::vector<double>::const_iterator e) 
    {
      std::copy(s, e, internal_state_.begin());
    }
    int initialize(const std::string &name, const std::string &description,
                   const std::string &name_space = "", const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    void finalize();

    void loarn_interface(hardware_interface::CommandInterface *command,
                         const hardware_interface::StateInterface *state);

    rclcpp_lifecycle::State get_state() { node_->get_current_state(); }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() { return node_; }

    virtual void update(const rclcpp::Time &t, const rclcpp::Duration &period) = 0;

    virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

    virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

    virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

    virtual CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

  protected:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string description_;
    robot_math::Robot robot_;
    hardware_interface::CommandInterface *command_;
    const hardware_interface::StateInterface *state_;
    std::vector<double> internal_state_; // e.g. integration of state
  };

} // namespace controller_interface

#endif // HARDWARE_INTERFACE_HPP