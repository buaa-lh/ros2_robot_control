#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP

namespace controller_interface
{
  class ControllerInterface
  {
    public:
      virtual ~ControllerInterface(){}

    protected:
      ControllerInterface(){}
  };
  
}  // namespace hardware

#endif  // HARDWARE_INTERFACE_HPP