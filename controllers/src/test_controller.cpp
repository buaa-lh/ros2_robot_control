#include <pluginlib/class_loader.hpp>
#include <controller_interface/controller_interface.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader("controller_interface", "controller_interface::ControllerInterface");

  try
  {
    std::shared_ptr<controller_interface::ControllerInterface> my_hard = loader.createSharedInstance("controllers::RobotController");
   
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}