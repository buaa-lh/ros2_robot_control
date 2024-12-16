#ifndef COMMAND_INTERFACE_HPP
#define COMMAND_INTERFACE_HPP
#include <vector>
#include <string>
#include <map>


namespace hardware_interface
{
  typedef std::map<std::string, std::vector<double>> CommandInterface;

} // namespace hardware

#endif // COMMAND_INTERFACE_HPP