#include "controller_interface/controller_interface.hpp"
#include <iostream>
#include "robot_math/robot_math.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
namespace controllers
{
    class DummyController : public controller_interface::ControllerInterface
    {
    public:
        DummyController() 
        {
        }
       
    protected:
      
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::DummyController, controller_interface::ControllerInterface)