#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "urdf/model.h"
#include "robot_math/robot_math.hpp"

namespace hardware_interface
{
  class HardwareInterface
  {
  public:
    virtual ~HardwareInterface() {}
    virtual int initialize(const std::string &robot_description)
    {
       if(robot_description.empty())
                return 0;
            else 
                return configure(robot_description);
    }
    virtual void finalize() {}
    virtual void read() {}
    virtual void write() {}
    CommandInterface &get_command_interface() { return command_; }
    const StateInterface &get_state_interface() { return state_; }
    const std::vector<std::string> &get_joint_name() { return joint_names_; }
    int get_dof() {return dof_;}
    const urdf::Model& get_urdf_model() { return robot_model_;}
    const robot_math::Robot& get_robot_model() { return robot_;}
    void write_state(const std::vector<double> &state, const std::vector<double> &force) // for simulation
    {
      std::copy(state.begin(), state.begin() + dof_, state_["position"].begin());
      std::copy(state.begin() + dof_, state.begin() + 2 * dof_, state_["velocity"].begin());
      state_["force"] = force;
    }
    int configure(const std::string &robot_description)
    {
      if (robot_model_.initString(robot_description))
      {
        robot_ = robot_math::urdf_to_robot(robot_description);
        for (auto j : robot_model_.joints_)
        {
          if (j.second->type != urdf::Joint::FIXED)
          {
            joint_names_.push_back(j.first);
          }
        }
        dof_ = joint_names_.size();
        command_["position"].resize(dof_);
        std::fill(command_["position"].begin(), command_["position"].end(), 0);
        command_["velocity"].resize(dof_);
        std::fill(command_["velocity"].begin(), command_["velocity"].end(), 0);
        command_["torque"].resize(dof_);
        std::fill(command_["torque"].begin(), command_["torque"].end(), 0);

        state_["position"].resize(dof_);
        std::fill(state_["position"].begin(), state_["position"].end(), 0);
        state_["velocity"].resize(dof_);
        std::fill(state_["velocity"].begin(), state_["velocity"].end(), 0);
        state_["torque"].resize(dof_);
        std::fill(state_["torque"].begin(), state_["torque"].end(), 0);
        state_["force"].resize(6);
        std::fill(state_["force"].begin(), state_["force"].end(), 0);
        return 1;
      }
      return 0;
    }

  protected:
    HardwareInterface() : dof_(0) {}
    CommandInterface command_;
    StateInterface state_;
    std::vector<std::string> joint_names_;
    int dof_;
    urdf::Model robot_model_;
    robot_math::Robot robot_;
  };

} // namespace hardware

#endif // HARDWARE_INTERFACE_HPP