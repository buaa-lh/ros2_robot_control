#include "hardware_interface/robot_interface.hpp"
#include <iostream>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 49152    /* Port the Net F/T always uses */
#define COMMAND 2     /* Command code 2 starts streaming */
#define NUM_SAMPLES 0 /* Will send 1 sample before stopping */

namespace hardwares
{
    typedef unsigned int uint32;
    typedef int int32;
    typedef unsigned short uint16;
    typedef short int16;
    typedef unsigned char byte;

    typedef struct response_struct
    {
        uint32 rdt_sequence;
        uint32 ft_sequence;
        uint32 status;
        int32 FTData[6];
    } RESPONSE;



    class FTATISensor : public hardware_interface::HardwareInterface
    {
    public:
        FTATISensor() : handle_(-1), is_running_(false)
        {
        }
        int start_sensing()
        {
            char request[8];                             /* The request data sent to the Net F/T. */
            *(uint16 *)&request[0] = htons(0x1234);      /* standard header. */
            *(uint16 *)&request[2] = htons(COMMAND);     /* per table 9.1 in Net F/T user manual. */
            *(uint32 *)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
            return sendto(handle_, request, 8, 0, (struct sockaddr *)&addr_, sizeof(addr_));
        }
        int stop_sensing()
        {
            char stoprequest[8] = {0x12, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* The request data sent to the Net F/T. */
            return sendto(handle_, stoprequest, 8, 0, (struct sockaddr *)&addr_, sizeof(addr_));
        }
        CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            publisher_ = node_->create_publisher<geometry_msgs::msg::Wrench>("~/wrench", rclcpp::SensorDataQoS());
            std::string sensor_ip;
            node_->get_parameter_or<std::string>("sensor_ip", sensor_ip, "");
            if (!sensor_ip.empty())
            {
                handle_ = socket(AF_INET, SOCK_DGRAM, 0); /* Handle to UDP socket used to communicate with Net F/T. */
                if (handle_ == -1)
                {
                    RCLCPP_WARN(node_->get_logger(), "Open socket failed!");
                    return CallbackReturn::FAILURE;
                }
                struct timeval read_timeout;
                read_timeout.tv_sec = 0;
                read_timeout.tv_usec = 10;
                // non-blocking
                if (setsockopt(handle_, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)))
                {
                    RCLCPP_WARN(node_->get_logger(), "set socket time out in service failed!");
                    return CallbackReturn::FAILURE;
                }
                memset(&addr_, 0, sizeof(addr_));
                addr_.sin_family = AF_INET;
                addr_.sin_port = htons(PORT);
                addr_.sin_addr.s_addr = inet_addr(sensor_ip.c_str()); // const char *ip = "192.168.124.12";
                return CallbackReturn::SUCCESS;
            }
            RCLCPP_WARN(node_->get_logger(), "IP empty! ATI sensor unconfiged!");
            return CallbackReturn::FAILURE;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
        {
            if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                if (thread_ && thread_->joinable())
                {
                    is_running_ = false;
                    thread_->join();
                }
                thread_ = nullptr;
                stop_sensing();
            }
            if (handle_ >= 0)
                close(handle_);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (start_sensing() > 0)
            {
                is_running_ = true;
                thread_ = std::make_unique<std::thread>(
                    [this]() -> void
                    {
                        socklen_t len = sizeof(addr_);
                        char readdata[36];
                        double force[6];
                        RESPONSE resp;
                        while (is_running_)
                        {
                            int resv_num = recvfrom(handle_, readdata, 36, 0, (struct sockaddr *)&addr_, &len);

                            if (resv_num > 0)
                            {
                                resp.rdt_sequence = ntohl(*(uint32 *)&readdata[0]);
                                resp.ft_sequence = ntohl(*(uint32 *)&readdata[4]);
                                resp.status = ntohl(*(uint32 *)&readdata[8]);
                                for (int i = 0; i < 6; i++)
                                {
                                    resp.FTData[i] = ntohl(*(int32 *)&readdata[12 + i * 4]);
                                    force[i] = resp.FTData[i] / 1000000.0f;
                                }
                                geometry_msgs::msg::Wrench::UniquePtr msg = std::make_unique<geometry_msgs::msg::Wrench>();
                                msg->force.x = force[0];
                                msg->force.y = force[1];
                                msg->force.z = force[2];
                                msg->torque.x = force[3];
                                msg->torque.y = force[4];
                                msg->torque.z = force[5];
                                publisher_->publish(std::move(msg));
                            }
                        }
                    });

                return CallbackReturn::SUCCESS;
            }
            else
                return CallbackReturn::FAILURE;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            if (thread_ && thread_->joinable())
            {
                is_running_ = false;
                thread_->join();
            }
            thread_ = nullptr;
            stop_sensing();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override
        {
            if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                if (thread_ && thread_->joinable())
                {
                    is_running_ = false;
                    thread_->join();
                }
                thread_ = nullptr;
                stop_sensing();
            }
            if (handle_ >= 0)
                close(handle_);
            return CallbackReturn::SUCCESS;
        }

    protected:
        int handle_;
        sockaddr_in addr_;
        std::unique_ptr<std::thread> thread_;
        volatile bool is_running_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    };

} // namespace hardwares

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hardwares::FTATISensor, hardware_interface::HardwareInterface)