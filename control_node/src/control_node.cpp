#include <errno.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "control_node/control_manager.h"
#include "realtime_tools/realtime_helpers.hpp"

using namespace std::chrono_literals;


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int kSchedPriority = 50;

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "control_node";

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  std::vector<std::string> node_arguments = node_options.arguments();
  for (int i = 1; i < argc; ++i)
  {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      // A simple way to reject non ros args
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  node_options.arguments(node_arguments);

  auto cm = std::make_shared<control_node::ControlManager>(
    executor, manager_node_name, "", node_options);

  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", true);
  std::string message;
  if (lock_memory && !realtime_tools::lock_memory(message))
  {
    RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory : '%s'", message.c_str());
  }
  
  const int cpu_affinity = cm->get_parameter_or<int>("cpu_affinity", -1);
  if (cpu_affinity >= 0)
  {
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
    if (!affinity_result.first)
    {
      RCLCPP_WARN(
        cm->get_logger(), "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str());
    }
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  std::thread cm_thread(
    [cm, thread_priority]()
    {
      if (!realtime_tools::configure_sched_fifo(thread_priority))
      {
        RCLCPP_WARN(
          cm->get_logger(),
          "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.",
          errno, strerror(errno));
      }
      else
      {
        RCLCPP_INFO(
          cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
          thread_priority);
      }
      RCLCPP_INFO(cm->get_logger(), "%i.",cm->get_update_rate());
      // for calculating sleep time
      auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      auto const cm_now = std::chrono::nanoseconds(cm->now().nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        next_iteration_time{cm_now};

      // for calculating the measured period of the loop
      rclcpp::Time previous_time = cm->now();

      while (rclcpp::ok())
      {
        // calculate measured period
        auto const current_time = cm->now();
        auto const measured_period = current_time - previous_time;
        previous_time = current_time;

        // execute update loop
        cm->read(current_time, measured_period);
        cm->update(current_time, measured_period);
        cm->write(current_time, measured_period);

        // wait until we hit the end of the period
        next_iteration_time += period;
        // printf("%.6f\n", measured_period.nanoseconds()/1e9);
        std::this_thread::sleep_until(next_iteration_time);
      }

      // cm->shutdown_async_controllers_and_components();
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}