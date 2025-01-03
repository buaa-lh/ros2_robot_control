#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP
#include <map>
#include <string>
#include <vector>
namespace hardware_interface
{
    // 状态接口 从string到vector<double>的映射
    typedef std::map<std::string, std::vector<double>> StateInterface;
} // namespace hardware

#endif // STATE_INTERFACE_HPP