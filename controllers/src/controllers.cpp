#include "controller_interface/controller_interface.hpp"
#include <iostream>

namespace controllers
{
    class MyController: public controller_interface::ControllerInterface
    {
        public:
        MyController()
        {
            std::cout << "in my controller\n";
        }

    };

}  // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::MyController, controller_interface::ControllerInterface)