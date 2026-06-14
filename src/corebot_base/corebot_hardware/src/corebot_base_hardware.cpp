#include "corebot_hardware/corebot_base_hardware.hpp"

namespace corebot_base
{

CorebotHardware::CorebotHardware()
    : logger_(rclcpp::get_logger("CorebotHardware"))
{}

bool CorebotHardware::open_serial_port(const ::std::string & port_name)
{
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1)
    {
        RCLCPP_ERROR(logger_, "Failed to open serial port: %s", port_name.c_str());
        return false;
    }
    fcntl(serial_fd_, F_SETFL, 0);
    RCLCPP_INFO(logger_, "Opened serial port: %s", port_name.c_str());
    return true;
}

bool CorebotHardware::configure_serial_port(int fd)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to get serial port attributes");
        return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, serial_baudrate_);
    cfsetospeed(&options, serial_baudrate_);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    // Binary packets must not be altered by software flow control or newline translation.
    options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                         ISTRIP | INLCR | IGNCR | ICRNL);
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;  // Non-blocking — drain whatever is in kernel buffer
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to set serial port attributes");
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    RCLCPP_INFO(logger_, "Configured serial port");
    return true;
}

void CorebotHardware::close_serial_port()
{
    if (serial_fd_ != -1)
    {
        close(serial_fd_);
        RCLCPP_INFO(logger_, "Closed serial port");
        serial_fd_ = -1;
    }
}

bool CorebotHardware::reconnect_serial_port()
{
    RCLCPP_WARN(logger_, "Reconnecting serial port %s", serial_port_.c_str());
    close_serial_port();

    if (!open_serial_port(serial_port_))
    {
        return false;
    }

    if (!configure_serial_port(serial_fd_))
    {
        close_serial_port();
        return false;
    }

    serial_read_buffer_.clear();
    missed_frame_cycles_ = 0;
    return true;
}

void CorebotHardware::read_serial_data()
{
    if (serial_fd_ < 0) return;

    if (serial_read_buffer_.size() > 2048)
    {
        serial_read_buffer_.clear();
        ++serial_buffer_resets_;
    }

    uint8_t buffer[1024];
    int n;
    while ((n = ::read(serial_fd_, buffer, sizeof(buffer))) > 0)
        serial_read_buffer_.insert(serial_read_buffer_.end(), buffer, buffer + n);

    if (serial_read_buffer_.empty()) return;

    while (serial_read_buffer_.size() >= 20)
    {
        if (serial_read_buffer_[0] != 0xAA || serial_read_buffer_[1] != 0x55)
        {
            serial_read_buffer_.erase(serial_read_buffer_.begin());
            ++serial_resync_bytes_;
            continue;
        }

        if (serial_read_buffer_[19] != 0xBB)
        {
            serial_read_buffer_.erase(serial_read_buffer_.begin());
            ++serial_resync_bytes_;
            continue;
        }

        uint8_t crc = 0;
        for (int i = 2; i <= 17; i++)
            crc ^= serial_read_buffer_[i];

        if (crc != serial_read_buffer_[18])
        {
            serial_read_buffer_.erase(serial_read_buffer_.begin());
            ++serial_crc_errors_;
            ++serial_resync_bytes_;
            continue;
        }

        std::vector<uint8_t> packet(
            serial_read_buffer_.begin(),
            serial_read_buffer_.begin() + 20);
        parse_serial_packet(packet);
        ++serial_valid_frames_;
        serial_read_buffer_.erase(
            serial_read_buffer_.begin(),
            serial_read_buffer_.begin() + 20);
    }

    if (node_)
    {
        RCLCPP_INFO_THROTTLE(
            logger_,
            *node_->get_clock(),
            5000,
            "Serial stats: valid=%llu crc=%llu resync_bytes=%llu buffer_resets=%llu buffered=%zu",
            static_cast<unsigned long long>(serial_valid_frames_),
            static_cast<unsigned long long>(serial_crc_errors_),
            static_cast<unsigned long long>(serial_resync_bytes_),
            static_cast<unsigned long long>(serial_buffer_resets_),
            serial_read_buffer_.size());
    }
}

// ── Frame layout (20 bytes, little-endian) ────────────────────────────────
//  [0]     0xAA  header
//  [1]     0x55  header
//  [2-3]   enc_l   int16 LE
//  [4-5]   enc_r   int16 LE
//  [6-7]   ax      int16 LE  (÷16384 → g)
//  [8-9]   ay      int16 LE
//  [10-11] az      int16 LE
//  [12-13] gx      int16 LE  (÷131 → °/s)
//  [14-15] gy      int16 LE
//  [16-17] gz      int16 LE
//  [18]    csum    XOR [2..17]
//  [19]    0xBB  footer
void CorebotHardware::parse_serial_packet(const std::vector<uint8_t> & packet)
{
    if (packet.size() != 20) return;

    encoder_left  = static_cast<int16_t>(packet[2] | (packet[3] << 8));
    encoder_right = static_cast<int16_t>(packet[4] | (packet[5] << 8));

    int16_t ax_raw = static_cast<int16_t>(packet[6]  | (packet[7]  << 8));
    int16_t ay_raw = static_cast<int16_t>(packet[8]  | (packet[9]  << 8));
    int16_t az_raw = static_cast<int16_t>(packet[10] | (packet[11] << 8));
    int16_t gx_raw = static_cast<int16_t>(packet[12] | (packet[13] << 8));
    int16_t gy_raw = static_cast<int16_t>(packet[14] | (packet[15] << 8));
    int16_t gz_raw = static_cast<int16_t>(packet[16] | (packet[17] << 8));

    imu_ax = ax_raw / 16384.0;
    imu_ay = ay_raw / 16384.0;
    imu_az = az_raw / 16384.0;
    imu_wx = gx_raw / 131.0;
    imu_wy = gy_raw / 131.0;
    imu_wz = gz_raw / 131.0;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = node_->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = imu_ax * 9.81;
    imu_msg.linear_acceleration.y = imu_ay * 9.81;
    imu_msg.linear_acceleration.z = imu_az * 9.81;

    imu_msg.angular_velocity.x = imu_wx * (M_PI / 180.0);
    imu_msg.angular_velocity.y = imu_wy * (M_PI / 180.0);
    imu_msg.angular_velocity.z = imu_wz * (M_PI / 180.0);

    imu_msg.orientation.x = imu_qx;
    imu_msg.orientation.y = imu_qy;
    imu_msg.orientation.z = imu_qz;
    imu_msg.orientation.w = imu_qw;

    imu_msg.orientation_covariance[0]         = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;
    imu_msg.angular_velocity_covariance[0]    = -1;

    if (imu_publisher_)
        imu_publisher_->publish(imu_msg);

    // Signal to read() that fresh encoder data is available this cycle
    new_encoder_data_ = true;
}

CallbackReturn CorebotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    RCLCPP_INFO(logger_, "Initializing...");

    config_.left_wheel_name   = info_.hardware_parameters["left_wheel_names"];
    config_.right_wheel_name  = info_.hardware_parameters["right_wheel_names"];
    config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
    config_.loop_rate         = std::stod(info_.hardware_parameters["loop_rate"]);

    left_wheel_.setup(config_.left_wheel_name,  config_.enc_ticks_per_rev);
    right_wheel_.setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

    if (!open_serial_port(serial_port_))
    {
        RCLCPP_ERROR(logger_, "Failed to open serial port during initialization");
        return CallbackReturn::ERROR;
    }
    if (!configure_serial_port(serial_fd_))
    {
        RCLCPP_ERROR(logger_, "Failed to configure serial port during initialization");
        return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("corebot_hardware_node");
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    RCLCPP_INFO(logger_, "Finished initialization");
    return CallbackReturn::SUCCESS;
}

CallbackReturn CorebotHardware::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Error occurred...");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CorebotHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(left_wheel_.name,  hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity);
    state_interfaces.emplace_back(left_wheel_.name,  hardware_interface::HW_IF_POSITION, &left_wheel_.position);
    state_interfaces.emplace_back(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity);
    state_interfaces.emplace_back(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position);
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CorebotHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(left_wheel_.name,  hardware_interface::HW_IF_VELOCITY, &left_wheel_.command);
    command_interfaces.emplace_back(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command);
    return command_interfaces;
}

CallbackReturn CorebotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn CorebotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Starting controller...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn CorebotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Stopping controller...");
    set_motor_speeds(0.0, 0.0, serial_fd_);
    return CallbackReturn::SUCCESS;
}

return_type CorebotHardware::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    double delta_seconds = period.seconds();
    if (delta_seconds <= 0.0) return return_type::OK;

    // Reset flag — parse_serial_packet() will set it true if a valid frame arrives
    new_encoder_data_ = false;
    read_serial_data();

    if (new_encoder_data_)
    {
        missed_frame_cycles_ = 0;
        // compute_encoder_delta() updates total_ticks internally.
        // Do NOT accumulate its return value — that is the double-accumulation bug.
        // Physical wiring: rotating right wheel → encoder_left channel, rotating left wheel → encoder_right channel.
        // controllers.yaml uses swapped joint names:
        //   left_wheel_  = right_wheel_joint (physical right) → must receive physical LEFT encoder signal for correct angular odometry
        //   right_wheel_ = left_wheel_joint  (physical left)  → must receive physical RIGHT encoder signal
        // Swapping here makes diff_drive_controller's angular = (v_right - v_left) = (phys_right - phys_left) ✓
        left_wheel_.compute_encoder_delta(encoder_left);   // encoder_right = physical left wheel
        right_wheel_.compute_encoder_delta(encoder_right);   // encoder_left  = physical right wheel
    }
    else
    {
        ++missed_frame_cycles_;
    }
    // If no new frame arrived this cycle, position/velocity hold their last
    // computed values — no phantom delta is introduced.

    double prev_left_pos  = left_wheel_.position;
    double prev_right_pos = right_wheel_.position;

    left_wheel_.position  = left_wheel_.calculate_encoder_angle();
    right_wheel_.position = right_wheel_.calculate_encoder_angle();

    left_wheel_.velocity  = (left_wheel_.position - prev_left_pos)  / delta_seconds;
    right_wheel_.velocity = (right_wheel_.position - prev_right_pos) / delta_seconds;

    if (node_)
    {
        if (new_encoder_data_)
        {
            RCLCPP_INFO_THROTTLE(
                logger_,
                *node_->get_clock(),
                2000,
                "ticks L=%d R=%d | vel L=%.2f R=%.2f rad/s",
                left_wheel_.total_ticks,
                right_wheel_.total_ticks,
                left_wheel_.velocity,
                right_wheel_.velocity);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *node_->get_clock(),
                3000,
                "No fresh serial frame received from the base controller");
        }
    }

    if (missed_frame_cycles_ >= 300)
    {
        RCLCPP_ERROR(
            logger_,
            "No serial frames for %.1f seconds, attempting port recovery",
            missed_frame_cycles_ * delta_seconds);
        reconnect_serial_port();
    }

    return return_type::OK;
}

return_type CorebotHardware::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    // Commands from diff_drive_controller are in rad/s.
    // set_motor_speeds() maps rad/s → PWM (0–999) internally.
    set_motor_speeds(left_wheel_.command, right_wheel_.command, serial_fd_);
    
    return return_type::OK;
}

} // namespace corebot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    corebot_base::CorebotHardware,
    hardware_interface::SystemInterface)
