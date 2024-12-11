#include "robot_math/robot_math.hpp"
#include <iostream>
#include <fstream>
using namespace robot_math;

int main()
{
    std::ifstream fin("/home/wjc/ros2_ws/urdf/fr3.urdf");
    std::string description((std::istreambuf_iterator<char>(fin)),
                            std::istreambuf_iterator<char>());
    Robot robot = urdf2Robot(description);
    print_robot(robot);
    std::vector<double> q{1,2,3,4,5,6,7};
    std::vector<double> dq{1,2,3,4,5,6,7};
    std::vector<double> ddq{1,2,3,4,5,6,7};
    Eigen::MatrixXd J;
    Eigen::Matrix4d T;
    coder::array<double, 3> JJ;
    robot_math::jacobian_matrix_all(&robot, q, JJ);
    robot_math::print_code_array(JJ);
    Eigen::Matrix6x7d Fext;
    Fext << 1, 0, 0, 0, 0, 0, 1,
            0, 1, 0, 0, 0, 0, 2,
            0, 0, 1, 0, 0, 0, 3,
            0, 0, 0, 1, 0, 0, 4,
            0, 0, 0, 0, 1, 0, 5,
            0, 0, 0, 0, 0, 1, 6;
    //std::cout << get_ext_torque(&robot, q, Fext) << "\n";
    //  std::cout << inverse_dynamics(&robot, q, dq, ddq, Fext) << "\n";
    std::cout << mass_matrix(&robot, q) << "\n";
    //std::cout << description << std::endl;
    return 0;
}