#include "hardware_interface/robot_interface.hpp"

namespace hardware_interface
{

    RobotInterface::RobotInterface() : dof_(0)
    {
    }
    void RobotInterface::finalize()
    {
        for(auto &c : components)
            c.second->get_node()->shutdown();
        HardwareInterface::finalize();
    }
    int RobotInterface::configure_urdf(const std::string &robot_description)
    {
        if (!robot_description.empty() && robot_model_.initString(robot_description))
        {
            robot_ = robot_math::urdf_to_robot(robot_description);
            for (auto & j : robot_model_.joints_)
            {
                if (j.second->type != urdf::Joint::FIXED)
                {
                    joint_names_.push_back(j.first);
                }
            }
            dof_ = joint_names_.size();
            state_names_.emplace_back("position");
            state_names_.emplace_back("velocity");
            state_names_.emplace_back("torque");

            command_names_.emplace_back("position");
            command_names_.emplace_back("velocity");
            command_names_.emplace_back("torque");
            for (auto &name : state_names_)
            {
                state_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            for (auto &name : command_names_)
            {
                command_.emplace(name, std::vector<double>(dof_, 0.0));
            }
            state_.emplace("force", std::vector<double>(6, 0.0));
            return 1;
        }
        return 0;
    }
    std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> RobotInterface::get_all_nodes()
    {
        std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes{node_->get_node_base_interface()};
        for(auto &p : components)
        {
            nodes.push_back(p.second->get_node()->get_node_base_interface());
        }
        return nodes;
    }
    CallbackReturn RobotInterface::on_configure(const rclcpp_lifecycle::State &/*previous_state*/)
    {
        if (configure_urdf(description_))
            return CallbackReturn::SUCCESS;
        else
            return CallbackReturn::FAILURE;
    }
    void RobotInterface::write(const rclcpp::Time &/*t*/, const rclcpp::Duration &/*period*/) 
    {
        state_["position"] = command_["position"];
        state_["velocity"] = command_["velocity"];
        state_["torque"] = command_["torque"];
    }
    void RobotInterface::robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t,
                                        std::function<Eigen::MatrixXd (double)> f_external,
                                        std::function<std::vector<double> (double, const std::vector<double> &, const Eigen::MatrixXd &)> controller)
    {
        int n = dof_;
        std::vector<double> q(n), dq(n), ddq(n);
        std::fill(ddq.begin(), ddq.end(), 0.0);
        std::copy(x.begin(), x.begin() + n, q.begin());
        std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        Eigen::MatrixXd f_ext = f_external(t);
        Eigen::VectorXd gvtao = robot_math::inverse_dynamics(&robot_, q, dq, ddq, f_ext);
        std::vector<double> cmd = controller(t, x, f_ext);
        Eigen::Map<Eigen::VectorXd> tau(&cmd[0], n);
        int m = cmd.size() - n;
        dx.resize(2 * n + m);
        std::copy(dq.begin(), dq.end(), dx.begin());
        Eigen::MatrixXd M = mass_matrix(&robot_, q);
        Eigen::VectorXd damping = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < dof_; i++)
            damping(i) = dq[i] * robot_model_.joints_.at(joint_names_[i])->dynamics->damping;
        Eigen::Map<Eigen::VectorXd>(&dx[n], n) = M.ldlt().solve(tau - gvtao - damping);
        std::copy(cmd.begin() + n, cmd.end(), dx.begin() + 2 * n);
    }
}