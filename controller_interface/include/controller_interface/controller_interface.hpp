#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
namespace robot_math
{
  class Robot;
}

namespace controller_interface
{
  class ControllerInterface
  {
  public:
    virtual ~ControllerInterface() {}
    virtual void update(const robot_math::Robot* robot, const hardware_interface::StateInterface *state,
                        hardware_interface::CommandInterface *command) = 0;

  protected:
    ControllerInterface() {}
  };

} // namespace controller_interface

#endif // HARDWARE_INTERFACE_HPP