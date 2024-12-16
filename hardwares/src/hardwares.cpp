#include "hardware_interface/hardware_interface.hpp"
#include <iostream>
#include <vector>

namespace hardwares
{
    class SimulationHardware : public hardware_interface::HardwareInterface
    {
    public:
        typedef std::vector<double> state_type;
        SimulationHardware()
        {
        }
        int initialize(const std::string &robot_description) override
        {
            return hardware_interface::HardwareInterface::initialize(robot_description);
            
        }
        void finalize() override
        {
        }
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationHardware, hardware_interface::HardwareInterface)