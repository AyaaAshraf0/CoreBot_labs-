#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class ImuComplementaryFilter : public rclcpp::Node
{
public:
ImuComplementaryFilter()
: Node("imu_complementary_filter")
{
alpha_ = 0.96;


    roll_ = pitch_ = yaw_ = 0.0;

    gx_bias_ = gy_bias_ = gz_bias_ = 0.0;
    calibrating_ = true;
    calib_samples_ = 0;

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 50,
        std::bind(&ImuComplementaryFilter::imu_callback, this, std::placeholders::_1));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(), "IMU complementary filter started");
}


private:


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    double ax = msg->linear_acceleration.x / 9.81;
    double ay = msg->linear_acceleration.y / 9.81;
    double az = msg->linear_acceleration.z / 9.81;

    msg->header.frame_id = "imu_link";

    rclcpp::Time now(msg->header.stamp);

    if (last_time_.nanoseconds() == 0)
    {
        last_time_ = now;
        return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // -------- Gyro Calibration --------
    if (calibrating_)
    {
        // RCLCPP_INFO(get_logger(), "Calibrating gyro bias... sample %d", calib_samples_ + 1);

        gx_bias_ += gx;
        gy_bias_ += gy;
        gz_bias_ += gz;

        calib_samples_++;

        if (calib_samples_ >= 300)
        {
            gx_bias_ /= calib_samples_;
            gy_bias_ /= calib_samples_;
            gz_bias_ /= calib_samples_;

            calibrating_ = false;

            // RCLCPP_INFO(get_logger(),
            //     "Gyro bias calibrated gx=%.6f gy=%.6f gz=%.6f",
            //     gx_bias_, gy_bias_, gz_bias_);
        }
        return;
    }
    gx -= gx_bias_;
    gy -= gy_bias_;
    gz -= gz_bias_;

    gz = -gz;  // MPU6050 Z-axis mounted inverted on CoreBot

    double accel_roll  = atan2(ay, az);
    double accel_pitch = atan2(-ax, sqrt(ay*ay + az*az));

    roll_  = alpha_ * (roll_  + gx * dt) + (1 - alpha_) * accel_roll;
    pitch_ = alpha_ * (pitch_ + gy * dt) + (1 - alpha_) * accel_pitch;
    yaw_  += gz * dt;
    

    publish_orientation(msg);
}

void publish_orientation(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    sensor_msgs::msg::Imu out = *msg;
    
    // Invert the Z-axis angular velocity to match the complementary filter and physical upside-down mount
    out.angular_velocity.z = -out.angular_velocity.z;

    // Euler → Quaternion
    double cy = cos(yaw_ * 0.5);
    double sy = sin(yaw_ * 0.5);
    double cp = cos(pitch_ * 0.5);
    double sp = sin(pitch_ * 0.5);
    double cr = cos(roll_ * 0.5);
    double sr = sin(roll_ * 0.5);

    out.orientation.w = cr * cp * cy + sr * sp * sy;
    out.orientation.x = sr * cp * cy - cr * sp * sy;
    out.orientation.y = cr * sp * cy + sr * cp * sy;
    out.orientation.z = cr * cp * sy - sr * sp * cy;

    // -------- Covariances from calibration --------

    // Angular velocity covariance
    out.angular_velocity_covariance[0] = 3.207450e-06;
    out.angular_velocity_covariance[1] = 0.0;
    out.angular_velocity_covariance[2] = 0.0;
    out.angular_velocity_covariance[3] = 0.0;
    out.angular_velocity_covariance[4] = 5.263556e-06;
    out.angular_velocity_covariance[5] = 0.0;
    out.angular_velocity_covariance[6] = 0.0;
    out.angular_velocity_covariance[7] = 0.0;
    // Be conservative on yaw-rate trust. The MPU6050 is useful for heading
    // dynamics, but this value was unrealistically tight and could make the
    // EKF outrun the real robot during turns.
    out.angular_velocity_covariance[8] = 8.0e-03;

    // Linear acceleration covariance
    out.linear_acceleration_covariance[0] = 9.868048e-04;
    out.linear_acceleration_covariance[1] = 0.0;
    out.linear_acceleration_covariance[2] = 0.0;
    out.linear_acceleration_covariance[3] = 0.0;
    out.linear_acceleration_covariance[4] = 9.385624e-04;
    out.linear_acceleration_covariance[5] = 0.0;
    out.linear_acceleration_covariance[6] = 0.0;
    out.linear_acceleration_covariance[7] = 0.0;
    out.linear_acceleration_covariance[8] = 2.533106e-03;

  
    // Orentation covariance 
    out.orientation_covariance[0] = 9.887772e-07;
    out.orientation_covariance[1] = 0.0;
    out.orientation_covariance[2] = 0.0;
    out.orientation_covariance[3] = 0.0;
    out.orientation_covariance[4] = 1.065441e-06;
    out.orientation_covariance[5] = 0.0;
    out.orientation_covariance[6] = 0.0;
    out.orientation_covariance[7] = 0.0;
    // Yaw from the complementary filter is informative, but not ground truth.
    // A looser covariance reduces heading overshoot in the EKF.
    out.orientation_covariance[8] = 1.0;

   

    // -------- TF Broadcast --------
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = out.header.stamp;
    t.header.frame_id = "base_link";
    t.child_frame_id = "imu_link";

    t.transform.translation.x = 0.015;
    t.transform.translation.y = 0.002;
    t.transform.translation.z = 0.098;

    t.transform.rotation = out.orientation;

    // tf_broadcaster_->sendTransform(t);

    imu_pub_->publish(out);
}

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

rclcpp::Time last_time_;

double roll_;
double pitch_;
double yaw_;

double gx_bias_;
double gy_bias_;
double gz_bias_;

bool calibrating_;
int calib_samples_;

double alpha_;


};

int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<ImuComplementaryFilter>());
rclcpp::shutdown();
return 0;
}
