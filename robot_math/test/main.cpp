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
    Eigen::MatrixXd J;
    Eigen::Matrix4d T;
    jacobian_matrix(&robot, q, J, T);
    std::cout << J << std::endl;
    //std::cout << description << std::endl;
    return 0;
}