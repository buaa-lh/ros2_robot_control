#include "hardware_interface/hardware_interface.hpp"
#include <iostream>

namespace hardwares
{
    class MyHardware: public hardware_interface::HardwareInterface
    {
        public:
        MyHardware()
        {

        }
        void initialize() override
        {
            std::cout << "My hardware\n";
        }
        void finalize() override
        {

        }
        void expose_interface() override
        {
            
        }

    };

}  // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::MyHardware, hardware_interface::HardwareInterface)