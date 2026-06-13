#ifndef _COREBOT_BASE__COREBOT_HARDWARE_HPP_
#define _COREBOT_BASE__COREBOT_HARDWARE_HPP_

#include <cmath>
#include <memory>
#include <cstring>
#include <thread>
#include <vector>
#include <algorithm>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor_encoder.hpp"
#include "wheel.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>

#include "sensor_msgs/msg/imu.hpp"
#include "corebot_interfaces/msg/encoder_ticks.hpp"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

namespace corebot_base
{

class CorebotHardware : public hardware_interface::SystemInterface
{
    struct Config
    {
        ::std::string left_wheel_name  = "left_wheel_joint";
        ::std::string right_wheel_name = "right_wheel_joint";
        int    enc_ticks_per_rev = 1084;
        double loop_rate         = 30.0;
    };

public:
    CorebotHardware();

    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    ::std::vector<hardware_interface::StateInterface>  export_state_interfaces()  override;
    ::std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_configure (const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate  (const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error     (const rclcpp_lifecycle::State & previous_state) override;

    return_type read (const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    bool open_serial_port     (const ::std::string & port_name);
    bool configure_serial_port(int fd);
    void close_serial_port    ();
    bool reconnect_serial_port();
    void read_serial_data     ();
    void parse_serial_packet  (const std::vector<uint8_t> & packet);

private:
    Config config_;

    Wheel left_wheel_;
    Wheel right_wheel_;

    rclcpp::Logger logger_;

    // Serial
    int           serial_fd_       = -1;
    ::std::string serial_port_     = "/dev/serial0";
    int           serial_baudrate_ = B115200;

    std::vector<uint8_t> serial_read_buffer_;
    uint64_t serial_valid_frames_ = 0;
    uint64_t serial_crc_errors_ = 0;
    uint64_t serial_resync_bytes_ = 0;
    uint64_t serial_buffer_resets_ = 0;

    // IMU
    double imu_ax = 0.0, imu_ay = 0.0, imu_az = 0.0;
    double imu_wx = 0.0, imu_wy = 0.0, imu_wz = 0.0;
    double imu_qx = 0.0, imu_qy = 0.0, imu_qz = 0.0, imu_qw = 1.0;

    // Encoders — raw absolute tick values from the latest serial frame
    int encoder_left  = 0;
    int encoder_right = 0;

    // Set to true by parse_serial_packet() when a valid frame arrives.
    // Consumed and reset to false at the start of each read() cycle.
    // Prevents stale encoder values causing phantom velocity spikes on
    // cycles where no new serial frame was received.
    bool new_encoder_data_ = false;
    int missed_frame_cycles_ = 0;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<corebot_interfaces::msg::EncoderTicks>::SharedPtr encoder_publisher_;
};

} // namespace corebot_base

#endif  // _COREBOT_BASE__COREBOT_HARDWARE_HPP_
