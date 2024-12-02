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
    //std::cout << description << std::endl;
    return 0;
}