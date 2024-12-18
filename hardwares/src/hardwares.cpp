#include "hardware_interface/robot_interface.hpp"
#include <iostream>
#include <vector>

namespace hardwares
{
    class SimulationRobot : public hardware_interface::RobotInterface
    {
    public:
        SimulationRobot()
        {
        }

    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::SimulationRobot, hardware_interface::RobotInterface)