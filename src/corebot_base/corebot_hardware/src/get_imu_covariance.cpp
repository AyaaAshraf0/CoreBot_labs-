#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ──────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────
static constexpr int TARGET_SAMPLES   = 3000;
static constexpr int WARMUP_SAMPLES   = 100;
static constexpr double PROGRESS_EVERY_S = 10.0;

static const std::string OUTPUT_DIR      = "/home/pi/CoreBot/";
static const std::string OUTPUT_FILENAME = "imu_covariance_after_filter.yaml";
// ──────────────────────────────────────────────

struct Sample {
  double gx, gy, gz;
  double ax, ay, az;
  double roll, pitch, yaw;
};

class IMUCalibrationNode : public rclcpp::Node
{
public:
  IMUCalibrationNode()
  : Node("imu_calibration_node"),
    collecting_(false),
    warmup_done_(false),
    warmup_count_(0),
    last_progress_time_(this->now())
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&IMUCalibrationNode::imu_callback, this, std::placeholders::_1));

    samples_.reserve(TARGET_SAMPLES);

    RCLCPP_INFO(this->get_logger(),"IMU Calibration Node started.");
    RCLCPP_INFO(this->get_logger(),
      "Warming up — ignoring first %d samples (~%d s)...",
      WARMUP_SAMPLES, WARMUP_SAMPLES / 100);

    RCLCPP_INFO(this->get_logger(),
      "Keep the robot COMPLETELY STILL and motors OFF.");
  }

private:

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!warmup_done_) {
      warmup_count_++;

      if (warmup_count_ >= WARMUP_SAMPLES) {
        warmup_done_ = true;
        collecting_  = true;

        RCLCPP_INFO(this->get_logger(),
          "Warmup done. Collecting %d samples...", TARGET_SAMPLES);
      }
      return;
    }

    if (!collecting_) return;

    Sample s;

    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;

    s.ax = msg->linear_acceleration.x;
    s.ay = msg->linear_acceleration.y;
    s.az = msg->linear_acceleration.z;

    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(s.roll, s.pitch, s.yaw);

    samples_.push_back(s);

    auto now = this->now();

    if ((now - last_progress_time_).seconds() >= PROGRESS_EVERY_S) {
      last_progress_time_ = now;

      int pct = static_cast<int>(100.0 * samples_.size() / TARGET_SAMPLES);

      RCLCPP_INFO(this->get_logger(),
        "Progress: %d / %d samples (%d%%)",
        static_cast<int>(samples_.size()), TARGET_SAMPLES, pct);
    }

    if (static_cast<int>(samples_.size()) >= TARGET_SAMPLES) {
      collecting_ = false;
      compute_and_save();
      rclcpp::shutdown();
    }
  }

  static double mean(const std::vector<double>& v)
  {
    double s = 0.0;
    for (double x : v) s += x;
    return s / static_cast<double>(v.size());
  }

  static double variance(const std::vector<double>& v)
  {
    double m = mean(v);
    double s = 0.0;

    for (double x : v)
      s += (x - m) * (x - m);

    return s / static_cast<double>(v.size());
  }

  void compute_and_save()
  {
    std::vector<double> gx, gy, gz;
    std::vector<double> ax, ay, az;
    std::vector<double> roll, pitch, yaw;

    gx.reserve(samples_.size());
    gy.reserve(samples_.size());
    gz.reserve(samples_.size());

    ax.reserve(samples_.size());
    ay.reserve(samples_.size());
    az.reserve(samples_.size());

    roll.reserve(samples_.size());
    pitch.reserve(samples_.size());
    yaw.reserve(samples_.size());

    for (const auto& s : samples_) {
      gx.push_back(s.gx);
      gy.push_back(s.gy);
      gz.push_back(s.gz);

      ax.push_back(s.ax);
      ay.push_back(s.ay);
      az.push_back(s.az);

      roll.push_back(s.roll);
      pitch.push_back(s.pitch);
      yaw.push_back(s.yaw);
    }

    double var_gx = variance(gx);
    double var_gy = variance(gy);
    double var_gz = variance(gz);

    double var_ax = variance(ax);
    double var_ay = variance(ay);
    double var_az = variance(az);

    double var_roll  = variance(roll);
    double var_pitch = variance(pitch);
    double var_yaw   = variance(yaw);

    double bias_gx = mean(gx);
    double bias_gy = mean(gy);
    double bias_gz = mean(gz);

    double bias_ax = mean(ax);
    double bias_ay = mean(ay);
    double bias_az = mean(az);

    RCLCPP_INFO(this->get_logger(),"Calibration complete");

    std::ostringstream yaml;
    yaml << std::scientific << std::setprecision(6);

    yaml << "imu_calibration:\n\n";

    yaml << "  gyro_bias:\n";
    yaml << "    x: " << bias_gx << "\n";
    yaml << "    y: " << bias_gy << "\n";
    yaml << "    z: " << bias_gz << "\n\n";

    yaml << "  accel_bias:\n";
    yaml << "    x: " << bias_ax << "\n";
    yaml << "    y: " << bias_ay << "\n";
    yaml << "    z: " << bias_az << "\n\n";

    yaml << "  angular_velocity_covariance: [";
    yaml << var_gx << ", 0.0, 0.0, "
         << "0.0, " << var_gy << ", 0.0, "
         << "0.0, 0.0, " << var_gz << "]\n\n";

    yaml << "  linear_acceleration_covariance: [";
    yaml << var_ax << ", 0.0, 0.0, "
         << "0.0, " << var_ay << ", 0.0, "
         << "0.0, 0.0, " << var_az << "]\n\n";

    yaml << "  orientation_covariance: [";
    yaml << var_roll << ", 0.0, 0.0, "
         << "0.0, " << var_pitch << ", 0.0, "
         << "0.0, 0.0, " << var_yaw << "]\n";

    std::string full_path = OUTPUT_DIR + OUTPUT_FILENAME;

    std::filesystem::create_directories(OUTPUT_DIR);

    std::ofstream file(full_path);

    if (file.is_open()) {
      file << yaml.str();
      file.close();
      RCLCPP_INFO(this->get_logger(),"Saved to: %s", full_path.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  std::vector<Sample> samples_;

  bool collecting_;
  bool warmup_done_;
  int warmup_count_;

  rclcpp::Time last_progress_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<IMUCalibrationNode>());

  rclcpp::shutdown();

  return 0;
}