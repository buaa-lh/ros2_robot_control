#include "controller_interface/controller_interface.hpp"
#include <iostream>
#include"robot_math/robot_math.hpp"
namespace controllers
{
    class MyController : public controller_interface::ControllerInterface
    {
    public:
        MyController()
        {
            std::cout << "in my controller\n";
        }
        void update(const robot_math::Robot *robot, const hardware_interface::StateInterface *state,
                    hardware_interface::CommandInterface *command) override
        {
            std::vector<double> &cmd_torque = (*command)["torque"];
            int n = static_cast<int>(robot->dof);
            for (int i = 0; i < n; i++)
                cmd_torque[i] = -state->at("velocity")[i];
            // std::vector<double> q(n), dq(n), ddq(n);
        // // controller_->update()
        // std::fill(ddq.begin(), ddq.end(), 0.0);
        // std::copy(x.begin(), x.begin() + n, q.begin());
        // std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        // Eigen::MatrixXd M, C, Jb, dJb, dM;
        // Eigen::VectorXd g;
        // Eigen::Matrix4d Tb, dTb;
        // std::vector<double> cmd(n);
        // m_c_g_matrix(robot_, q, dq, M, C, g, Jb, dJb, dM, dTb, Tb);
        // for (int i = 0; i < n; i++)
        //     cmd[i] = -dq[i];
        // std::fill(cmd.begin(), cmd.end(), 0.0);
            
        }
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::MyController, controller_interface::ControllerInterface)