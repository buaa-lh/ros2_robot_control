#include "control_node/control_manager.h"
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;
namespace control_node
{
    ControlManager::ControlManager(std::shared_ptr<rclcpp::Executor> executor,
                                   const std::string &node_name, const std::string &name_space, const rclcpp::NodeOptions &option)
        : rclcpp::Node(node_name, name_space, option),
          is_new_cmd_available_(false),
          executor_(executor),
          dof_(0),
          is_simulation_(true),
          param_listener_(std::make_shared<ParamListener>(this->get_node_parameters_interface()))
    {

        params_ = param_listener_->get_params();
        update_rate_ = params_.update_rate;
        joint_command_topic_name_ = this->get_parameter_or<std::string>("cmd_name", "gui/joint_state");
        is_simulation_ = this->get_parameter_or<bool>("simulation", true);
        command_receiver_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_command_topic_name_, rclcpp::SensorDataQoS(), std::bind(&ControlManager::robot_joint_command_callback, this, std::placeholders::_1));

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

        description_sub_ = this->create_subscription<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).transient_local(), std::bind(&ControlManager::robot_description_callback, this, std::placeholders::_1));

        real_time_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_state_publisher_);

        robot_description_ = this->get_parameter_or<std::string>("robot_description", "");
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

    void ControlManager::robot_joint_command_callback(sensor_msgs::msg::JointState::SharedPtr js)
    {
        real_time_buffer_.writeFromNonRT(js);
        is_new_cmd_available_ = true;
    }

    void ControlManager::config_robot()
    {
        if (!robot_description_.empty())
        {
            robot_ = robot_math::urdf_to_robot(robot_description_);
            robot_math::print_robot(robot_);
            robot_model_.initString(robot_description_);
            for (auto j : robot_model_.joints_)
            {
                if (j.second->type != urdf::Joint::FIXED)
                {
                    joint_names_.push_back(j.first);
                    RCLCPP_INFO(get_logger(), j.first.c_str());
                }
            }
            dof_ = joint_names_.size();
            joint_position_command_.resize(dof_);
            std::fill(joint_position_command_.begin(), joint_position_command_.end(), 0);

            joint_velocity_command_.resize(dof_);
            std::fill(joint_velocity_command_.begin(), joint_velocity_command_.end(), 0);

            joint_torque_command_.resize(dof_);
            std::fill(joint_torque_command_.begin(), joint_torque_command_.end(), 0);

            joint_position_.resize(dof_);
            std::fill(joint_position_.begin(), joint_position_.end(), 0);
            joint_velocity_.resize(dof_);
            std::fill(joint_velocity_.begin(), joint_velocity_.end(), 0);
            joint_torque_.resize(dof_);
            std::fill(joint_torque_.begin(), joint_torque_.end(), 0);
        }
    }
    void control_node::ControlManager::read(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = joint_names_;
        states->position = joint_position_;
        states->velocity = joint_velocity_;
        states->effort = joint_torque_;
        states->header.stamp = t;
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
    }

    void ControlManager::update(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        if (is_new_cmd_available_)
        {
            auto js = *real_time_buffer_.readFromRT();
            joint_position_command_ = js->position;
            is_new_cmd_available_ = false;
        }
    }

    void ControlManager::write(const rclcpp::Time &t, const rclcpp::Duration &period)
    {
        joint_position_ = joint_position_command_;
    }
    Eigen::MatrixXd ControlManager::simulation_external_force(double t)
    {
        return Eigen::MatrixXd::Zero(6, dof_);
    }
    void ControlManager::robot_dynamics(const std::vector<double> &x, std::vector<double> &dx, double t)
    {
        int n = dof_;
        std::vector<double> q(n), dq(n), ddq(n);
        std::fill(ddq.begin(), ddq.end(), 0.0);
        std::copy(x.begin(), x.begin() + n, q.begin());
        std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());
        Eigen::MatrixXd f_ext = simulation_external_force(t);
        Eigen::VectorXd gvtao = robot_math::inverse_dynamics(&robot_, q, dq, ddq, f_ext);
        std::vector<double> cmd = simulation_controller(t, x, f_ext);
        Eigen::Map<Eigen::VectorXd> tau(&cmd[0], n);
        int m = cmd.size() - n;
        dx.resize(2 * n + m);
        std::copy(dq.begin(), dq.end(), dx.begin());
        Eigen::MatrixXd M = mass_matrix(&robot_, q);
        Eigen::VectorXd damping = Eigen::VectorXd::Zero(n);
        for(int i = 0; i < dof_; i++)
            damping(i) = dq[i] * robot_model_.joints_[joint_names_[i]]->dynamics->damping;
        Eigen::Map<Eigen::VectorXd>(&dx[n], n) = M.ldlt().solve(tau - gvtao - damping);
        std::copy(cmd.begin() + n, cmd.end(), dx.begin() + 2 * n);
    }
    void ControlManager::simulation_observer(const std::vector<double> &x, double t)
    {
        std::cerr << t << " : ";
        for (int i = 0; i < dof_; i++)
            std::cerr << x[i] << " ";
        std::cerr << "\n";
        std::copy(x.begin(), x.begin() + dof_, joint_position_.begin());
        std::copy(x.begin() + dof_, x.begin() + 2 * dof_, joint_velocity_.begin());
        auto states = std::make_shared<sensor_msgs::msg::JointState>();
        states->name = joint_names_;
        states->position = joint_position_;
        states->velocity = joint_velocity_;
        states->effort = joint_torque_;
        auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(t));
        states->header.stamp = rclcpp::Time(time.count());
        if (real_time_publisher_->trylock())
        {
            real_time_publisher_->msg_ = *states;
            real_time_publisher_->unlockAndPublish();
        }
        // wait for real time elapse
        auto until = sim_start_time_ + std::chrono::duration<double>(t);
        std::this_thread::sleep_until(until);
    }

    bool ControlManager::is_simulation()
    {
        return is_simulation_;
    }

    void ControlManager::start_simulation(double time)
    {
        typedef std::vector<double> state_type;
        auto dynamics = std::bind(&ControlManager::robot_dynamics, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3);

        auto observer = std::bind(&ControlManager::simulation_observer, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2);

        typedef runge_kutta4<state_type> rk4;

        // Error stepper, used to create the controlled stepper
        typedef runge_kutta_cash_karp54<state_type> rkck54;

        // Controlled stepper:
        // it's built on an error stepper and allows us to have the output at each
        // internally defined (refined) timestep, via integrate_adaptive call
        typedef controlled_runge_kutta<rkck54> ctrl_rkck54;
        std::vector<double> x0{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        sim_start_time_ = std::chrono::steady_clock::now();
        integrate_const(ctrl_rkck54(), dynamics, x0, 0.0, 10.0, 0.01, observer);
        // size_t steps = integrate_adaptive(runge_kutta4<std::vector<double>>(), dynamics, x0, 0.0, time, 0.01, observer);
    }

    std::vector<double> ControlManager::simulation_controller(double t, const std::vector<double> &x, const Eigen::MatrixXd &fext)
    {
        int n = dof_;
        std::vector<double> q(n), dq(n), ddq(n);
        std::fill(ddq.begin(), ddq.end(), 0.0);
        std::copy(x.begin(), x.begin() + n, q.begin());
        std::copy(x.begin() + n, x.begin() + 2 * n, dq.begin());

        std::vector<double> cmd(n);
        for(int i = 0; i < n; i++)
            cmd[i] = -dq[i];
        //std::fill(cmd.begin(), cmd.end(), 0.0);
        return cmd;
    }

}
