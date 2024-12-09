#include "control_node/control_manager.h"


namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor),
          param_listener_(std::make_shared<ParamListener>(this->get_node_parameters_interface()))
    {

        params_ = param_listener_->get_params();
        update_rate_ = params_.update_rate;
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

        description_sub_ = this->create_subscription<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local(), std::bind(&ControlManager::robot_description_callback, this, std::placeholders::_1));

        real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);

        robot_description_ = this->declare_parameter("robot_description", "");
        config_robot();
    }

    ControlManager::~ControlManager()
    {
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::init_robot()
    {
        bool ready;
        do
        {
            RCLCPP_INFO(get_logger(), "waiting for robot");
            std::cerr << "waiting for robot\n";
            robot_desp_mutex_.lock();
            ready = robot_description_.empty();
            robot_desp_mutex_.unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        } while (ready);
        RCLCPP_INFO(get_logger(), "robot ok");
    }

    void ControlManager::shutdown_robot()
    {
    }

    void ControlManager::robot_description_callback(std_msgs::msg::String::SharedPtr desp)
    {
        std::lock_guard<std::mutex> guard(robot_desp_mutex_);
        robot_description_ = desp->data;
        config_robot();
    }

    void ControlManager::config_robot()
    {
        if (!robot_description_.empty())
        {
            robot = robot_math::urdf2Robot(robot_description_);
            robot_math::print_robot(robot);
            robot_model_.initString(robot_description_);
            for (auto j : robot_model_.joints_)
            {
                if (j.second->type != urdf::Joint::FIXED)
                {
                    joint_names_.push_back(j.first);
                    RCLCPP_INFO(get_logger(), j.first.c_str());
                }
            }
            joint_position_command_.resize(joint_names_.size());
            std::fill(joint_position_command_.begin(), joint_position_command_.end(), 0);
            joint_position_.resize(joint_names_.size());
            std::fill(joint_position_.begin(), joint_position_.end(), 0);
        }
    }
    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = joint_names_;
        states->position = joint_position_;
        states->header.stamp = t;
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        joint_position_ = joint_position_command_;
    }

}
