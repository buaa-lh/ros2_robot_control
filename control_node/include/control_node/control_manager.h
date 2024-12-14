#pragma once
#include "rclcpp/rclcpp.hpp"
#include "control_node_parameters.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "std_msgs/msg/string.hpp"
#include "robot_math/robot_math.hpp"
#include <functional>
#include <chrono>
namespace control_node
{

    class ControlManager: public rclcpp::Node
    {
        public:
            ControlManager(std::shared_ptr<rclcpp::Executor> executor, const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option);
            ~ControlManager();
            int get_update_rate();
            virtual void init_robot();
            virtual void shutdown_robot();
            virtual void read(const rclcpp::Time &t, const rclcpp::Duration & period);
            virtual void update(const rclcpp::Time &t, const rclcpp::Duration & period);
            virtual void write(const rclcpp::Time &t, const rclcpp::Duration & period);
            void robot_dynamics(const std::vector<double>& x, std::vector<double>& dx, double t);
            void start_simulation(double time = 10.0);// seconds
            std::vector<double> simulation_controller(double t, const std::vector<double> & x, const Eigen::MatrixXd & fext);
            Eigen::MatrixXd simulation_external_force(double t);
            void simulation_observer(const std::vector<double> &x, double t);
            bool is_simulation();
        protected:
            void robot_description_callback(std_msgs::msg::String::SharedPtr desp);
            void robot_joint_command_callback(sensor_msgs::msg::JointState::SharedPtr js);
            void config_robot();
        protected:
            std::shared_ptr<rclcpp::Executor> executor_;
            int update_rate_;
            Params params_;
            std::string robot_description_;
            std::string joint_command_topic_name_;
            std::shared_ptr<ParamListener> param_listener_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_receiver_;
            realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> real_time_buffer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
            std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> real_time_publisher_;
            urdf::Model robot_model_;
            robot_math::Robot robot_;
            std::vector<std::string> joint_names_;
            std::mutex robot_desp_mutex_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_torque_command_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_torque_;
            volatile bool is_new_cmd_available_;
            int dof_;
            bool is_simulation_;
            bool is_sim_real_time_;
            std::chrono::time_point<std::chrono::steady_clock>  sim_start_time_;
    };



}