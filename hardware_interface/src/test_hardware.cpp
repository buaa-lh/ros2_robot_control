#include <pluginlib/class_loader.hpp>
#include <hardware_interface/hardware_interface.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<hardware_interface::HardwareInterface> loader("hardware_interface", "hardware_interface::HardwareInterface");

  try
  {
    std::shared_ptr<hardware_interface::HardwareInterface> my_hard = loader.createSharedInstance("hardwares::MyHardware");
    my_hard->initialize();
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}