#ifndef COMMAND_INTERFACE_HPP
#define COMMAND_INTERFACE_HPP
#include <map>
#include <string>
#include <vector>

namespace hardware_interface
{
    typedef std::map<std::string, std::vector<double>> CommandInterface;

} // namespace hardware

#endif // COMMAND_INTERFACE_HPP