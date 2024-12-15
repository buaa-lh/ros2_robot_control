#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP
#include <vector>
#include <string>
#include <map>
namespace hardware_interface
{
  typedef std::map<std::string, std::vector<double>> CommandInterface;
  typedef std::map<std::string, std::vector<double>> StateInterface;
  class HardwareInterface
  {
  public:
    virtual void initialize() = 0;
    virtual ~HardwareInterface() {}
    virtual void expose_interface() = 0;
    virtual void finalize() = 0;
    CommandInterface & get_command_interface() { return command_;}
    const StateInterface & get_state_interface() { return state_;}
    const std::vector<std::string> & get_joint_name() {return joint_names_;}
  protected:
    HardwareInterface() {}
    CommandInterface command_;
    StateInterface state_;
    std::vector<std::string> joint_names_;
  };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP