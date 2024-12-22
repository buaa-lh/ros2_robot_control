#include "control_node/control_manager.h"
#include <boost/numeric/odeint.hpp>
#include "lifecycle_msgs/msg/state.hpp"
using namespace boost::numeric::odeint;
namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          executor_(executor),
          param_listener_(std::make_shared<ParamListener>(this->get_node_parameters_interface())),
          is_new_cmd_available_(false)
    {
        params_ = param_listener_->get_params();
        update_rate_ = params_.update_rate;
        joint_command_topic_name_ = this->get_parameter_or<std::string>("cmd_name", "gui/joint_state");
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        is_sim_real_time_ = this->get_parameter_or<bool>("sim_real_time", true);
        is_publish_joint_state_ = this->get_parameter_or<bool>("publish_joint_state", true);
        if (is_publish_joint_state_ || is_simulation_)
        {
            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
            real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);
        }
        command_receiver_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_command_topic_name_, rclcpp::SensorDataQoS(),
                                                                                    std::bind(&ControlManager::robot_joint_command_callback, this, std::placeholders::_1));
        // joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        //  description_sub_ = this->create_subscription<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local(),
        //  std::bind(&ControlManager::robot_description_callback, this, std::placeholders::_1));
        // real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);

        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
        if (robot_description_.empty())
            throw std::runtime_error("robot description file is empty!");

        std::string hardware_class = this->get_parameter_or<std::string>("hardware", "");
        std::vector<std::string> controller_class = this->get_parameter_or<std::vector<std::string>>("controllers", std::vector<std::string>());
        hardware_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::RobotInterface>>("hardware_interface", "hardware_interface::RobotInterface");
        controller_loader_ = std::make_unique<pluginlib::ClassLoader<controller_interface::ControllerInterface>>("controller_interface", "controller_interface::ControllerInterface");
        try
        {
            robot_ = hardware_loader_->createSharedInstance(hardware_class);
            int pos = hardware_class.rfind(":");
            hardware_class = hardware_class.substr(pos + 1);
            robot_->initialize(hardware_class, robot_description_);
            auto nodes = robot_->get_all_nodes();
            for (auto &no : nodes)
                executor_->add_node(no);

            for (auto name : controller_class)
            {
                auto controller = controller_loader_->createSharedInstance(name);
                pos = name.rfind(":");
                name = name.substr(pos + 1);
                controller->initialize(name, robot_description_);
                controller->loarn_interface(&robot_->get_command_interface(), &robot_->get_state_interface());
                controllers_.push_back(controller);
                executor_->add_node(controller->get_node()->get_node_base_interface());
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            throw ex;
        }
        int active = this->get_parameter_or<int>("active_controller", 0);
        active_controller_ = controllers_[active];
    }

    ControlManager::~ControlManager()
    {
    }

    int ControlManager::get_update_rate()
    {
        return update_rate_;
    }

    void ControlManager::wait_for_active_controller()
    {
        while (!active_controller_)
        {
            for (auto &controller : controllers_)
            {
                if (controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
                {
                    active_controller_ = controller;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1000000));
            RCLCPP_INFO(this->get_logger(), "waiting for controller");
        }
        RCLCPP_INFO(get_logger(), "%s controller activated", active_controller_->get_node()->get_name());
    }

    void ControlManager::shutdown_robot()
    {
        RCLCPP_INFO(this->get_logger(), "shutting donw");
        robot_->finalize();
    }

    // void ControlManager::robot_description_callback(std_msgs::msg::String::SharedPtr desp)
    // {
    //     std::lock_guard<std::mutex> guard(robot_desp_mutex_);
    //     robot_description_ = desp->data;
    // }

    void ControlManager::robot_joint_command_callback(sensor_msgs::msg::JointState::SharedPtr js)
    {
        real_time_buffer_.writeFromNonRT(js);
        is_new_cmd_available_ = true;
    }

    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->read(t, period);
        if (is_publish_joint_state_)
        {
            auto states = std::make_shared<sensor_msgs::msg::JointState>();
            states->name = robot_->get_joint_names();
            states->position = robot_->get_state_interface().at("position");
            states->velocity = robot_->get_state_interface().at("velocity");
            states->effort = robot_->get_state_interface().at("effort");
            states->header.stamp = t;
            if (real_time_publisher_->trylock())
            {
                real_time_publisher_->msg_ = *states;
                real_time_publisher_->unlockAndPublish();
            }
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        if (active_controller_)
            active_controller_->update(t, period);
        // for (auto &controller : controllers_)
        // {
        //     if (controller->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        //     {
        //         controller->update(t, period);
        //     }
        // }
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        robot_->write(t, period);
    }

    Eigen::MatrixXd ControlManager::simulation_external_force(double t)
    {
        return Eigen::MatrixXd::Zero(6, robot_->get_dof());
    }
    void ControlManager::simulation_observer(const std::vector<double> &x, double t)
    {
        // std::cerr << t << " : ";
        // for (int i = 0; i < dof_; i++)
        //     std::cerr << x[i] << " ";
        // std::cerr << "\n";
        // std::copy(x.begin(), x.begin() + dof_, joint_position_.begin());
        // std::copy(x.begin() + dof_, x.begin() + 2 * dof_, joint_velocity_.begin());
        int n = robot_->get_dof();
        if (t == 0)
        {
            Eigen::MatrixXd f_ext = simulation_external_force(t);
            simulation_controller(t, x, f_ext);
        }
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = robot_->get_joint_names();
        std::copy(x.begin(), x.begin() + n, std::back_inserter(states->position));
        std::copy(x.begin() + n, x.begin() + 2 * n, std::back_inserter(states->velocity));
        states->effort = robot_->get_command_interface().at("torque");
        auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        states->header.stamp = rclcpp::Time(time.count());
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
        // wait for real time elapse
        if (is_sim_real_time_)
        {
            auto until = sim_start_time_ + std::chrono::duration<double>(t);
            std::this_thread::sleep_until(until);
        }
    }

    bool ControlManager::is_simulation()
    {
        return is_simulation_;
    }

    void ControlManager::start_simulation(double time)
    {
        typedef std::vector<double> state_type;

        auto f_external = std::bind(&ControlManager::simulation_external_force, this,
                                    std::placeholders::_1);

        auto controller = std::bind(&ControlManager::simulation_controller, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3);

        auto dynamics = std::bind(&hardware_interface::RobotInterface::robot_dynamics, robot_.get(),
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3,
                                  std::cref(f_external), std::cref(controller));

        auto observer = std::bind(&ControlManager::simulation_observer, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

        // Error stepper, used to create the controlled stepper
        typedef runge_kutta_cash_karp54<state_type> error_stepper_type;
        typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;
        state_type x0{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        sim_start_time_ = std::chrono::steady_clock::now();
        auto dx = std::vector<double>();
        integrate_adaptive(make_controlled(1.0e-10, 1.0e-6, error_stepper_type()), dynamics, x0, 0.0, 10.0, 0.001, observer);
        // size_t steps = integrate_adaptive(runge_kutta4<std::vector<double>>(), dynamics, x0, 0.0, time, 0.01, observer);
    }

    std::vector<double> ControlManager::simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext)
    {
        int n = robot_->get_dof();
        std::vector<double> f{fext(0, n - 1), fext(1, n - 1), fext(2, n - 1),
                              fext(3, n - 1), fext(4, n - 1), fext(5, n - 1)};
        robot_->write_state(x, f);
        auto std_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        auto time = rclcpp::Time(std_time.count());
        auto period = rclcpp::Duration(std::chrono::duration<double>(0.0));
        active_controller_->write_state(x.begin() + 2 * n, x.end());
        active_controller_->update(time, period);
        auto cmd = robot_->get_command_interface().at("torque");
        cmd.insert(cmd.end(), active_controller_->get_internal_state().begin(), active_controller_->get_internal_state().end());
        return cmd;
    }

}
