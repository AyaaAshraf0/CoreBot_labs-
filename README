# Corebot AMR — API Reference

> **Platform:** ROS2 Humble · Raspberry Pi 4 (Ubuntu) · STM32
> **Drive type:** Differential drive · **Sensors:** IMU, wheel encoders (STM32), LiDAR (RPi)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Quick Start](#2-quick-start)
3. [Topics](#3-topics)
   - [3.1 Hardware Layer](#31-hardware-layer-raw)
   - [3.2 Sensor Layer](#32-sensor-layer)
   - [3.3 Control Layer](#33-control-layer)
   - [3.4 System Topics](#34-system-topics)
4. [Teleop Node](#4-teleop-node)
5. [ros2_control Interface](#5-ros2_control-interface)
6. [Serial Protocol Reference](#6-serial-protocol-reference)
7. [Configuration Reference](#7-configuration-reference)
8. [Launch Files](#8-launch-files)

---

## 1. System Overview

Nodes are shown as **ellipses**, topics as **rectangles** — matching rqt_graph conventions.

```mermaid
flowchart LR

    %% ── External hardware ─────────────────────────────────────
    STM32(["STM32"])

    %% ── Nodes ────────────────────────────────────────────────
    HW(["corebot_hardware_node\n― ros2_control plugin ―"])
    IMU_CAL(["imu_calibration_node\n― complementary filter ―"])
    LIDAR(["ldlidar_published"])
    TELEOP(["corebot_teleop_key"])
    DDC(["diff_drive_controller"])
    JSB(["joint_state_broadcaster"])
    RSP(["robot_state_publisher"])
    USER(["Your Node / Nav Stack"])

    %% ── Topics ───────────────────────────────────────────────
    T_ENC["/encoder_ticks"]
    T_IMUR["/imu/data"]
    T_IMU["/imu"]
    T_SCAN["/scan"]
    T_PC["/pointcloud2d"]
    T_DCMDVEL["/diff_controller/cmd_vel"]
    T_DODOM["/diff_controller/odom"]
    T_JS["/joint_states"]
    T_DJS["/dynamic_joint_states"]
    T_RD["/robot_description"]
    T_TF["/tf  /tf_static"]

    %% ── STM32 ↔ hardware node ────────────────────────────────
    STM32 -- "UART RX\n20-byte binary" --> HW
    HW -- "UART TX\nM<l>,<r>" --> STM32

    %% ── Hardware node outputs ────────────────────────────────
    HW --> T_ENC
    HW --> T_IMUR
    HW <--> DDC

    %% ── IMU pipeline ─────────────────────────────────────────
    T_IMUR --> IMU_CAL
    IMU_CAL --> T_IMU

    %% ── LiDAR ────────────────────────────────────────────────
    LIDAR --> T_SCAN
    LIDAR --> T_PC

    %% ── Control pipeline ─────────────────────────────────────
    TELEOP --> T_DCMDVEL
    USER --> T_DCMDVEL
    T_DCMDVEL --> DDC
    DDC --> T_DODOM

    %% ── Joint states ─────────────────────────────────────────
    JSB --> T_JS
    JSB --> T_DJS

    %% ── Robot state publisher ────────────────────────────────
    RSP --> T_RD
    RSP --> T_TF
```

---

## 2. Quick Start

### Full robot bringup
```bash
ros2 launch corebot_bringup corebot_bringup.launch.py
```
Brings up: hardware interface, IMU calibration, LiDAR (LD06), and controller manager.

To launch without the LiDAR:
```bash
ros2 launch corebot_bringup corebot_bringup.launch.py use_lidar:=false
```

### Keyboard teleoperation
```bash
ros2 run corebot_bringup teleop_node
```
See [Section 4](#4-teleop-node) for key bindings and parameters.

### EKF sensor fusion (optional)

The EKF is a separate package (`sensor_fusion`). Install it first:

```bash
cd ~/ros2_ws/src
git clone <SENSOR_FUSION_REPO_URL>
cd ~/ros2_ws && colcon build --packages-select sensor_fusion
source install/setup.bash
ros2 launch sensor_fusion odom_imu_ekf.launch.py
```

The EKF fuses `/diff_controller/odom` and `/imu` (calibrated).

---

## 3. Topics

### 3.1 Hardware Layer (raw)

These topics are published directly by the hardware plugin from raw serial data. They carry **uncalibrated, unfiltered** data. Use the processed topics in [Section 3.2](#32-sensor-layer) for any real application.

---

#### `/imu/data`

| Field | Value |
|---|---|
| **Type** | `sensor_msgs/msg/Imu` |
| **Publisher** | `corebot_hardware_node` |
| **QoS depth** | 10 |
| **Rate** | Tied to serial frame rate from STM32 |

Raw IMU data from the MPU-6050. All three covariance matrices are set to `-1` (REP-145 convention) — none of the fields are valid for direct use:

```
linear_acceleration   →  m/s²   raw, uncalibrated  (covariance[0] = -1)
angular_velocity      →  rad/s  raw, uncalibrated  (covariance[0] = -1)
orientation           →  NOT PROVIDED               (covariance[0] = -1)
```

> ⚠️ Do not subscribe to `/imu/data` in your application. Use `/imu` (Section 3.2).

---

#### `/encoder_ticks`

| Field | Value |
|---|---|
| **Type** | `corebot_interfaces/msg/EncoderTicks` |
| **Publisher** | `corebot_hardware_node` |
| **QoS depth** | 10 |
| **Rate** | Every valid serial frame |

Raw encoder data for developers implementing their own odometry or controller without the `diff_drive_controller`.

```
std_msgs/Header header
int32  total_ticks_left    # cumulative since startup
int32  total_ticks_right
int16  delta_left          # per-cycle delta computed on RPi (can be negative)
int16  delta_right
```

> **Note:** The STM32 sends 16-bit **absolute** encoder counts. The per-cycle `delta` fields are computed on the RPi inside the hardware plugin (`compute_encoder_delta()`), which also handles counter wrap-around. You do not need to handle wrap-around yourself when using this topic.

**Example — wheel displacement from delta ticks (C++):**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <corebot_interfaces/msg/encoder_ticks.hpp>
#include <cmath>

constexpr double TICKS_PER_REV = 1440.0;  // match enc_ticks_per_rev param
constexpr double WHEEL_RADIUS  = 0.0325;  // metres

class MyOdom : public rclcpp::Node
{
public:
  MyOdom() : Node("my_odom")
  {
    sub_ = create_subscription<corebot_interfaces::msg::EncoderTicks>(
      "/encoder_ticks", 10,
      [this](const corebot_interfaces::msg::EncoderTicks::SharedPtr msg) {
        double dist_left  = (msg->delta_left  / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;
        double dist_right = (msg->delta_right / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;
        // your odometry logic here
        (void)dist_left; (void)dist_right;
      });
  }
private:
  rclcpp::Subscription<corebot_interfaces::msg::EncoderTicks>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyOdom>());
  rclcpp::shutdown();
}
```

> If you are using the `diff_drive_controller`, odometry is handled automatically — you do not need this topic.

---

### 3.2 Sensor Layer

Processed, application-ready topics.

---

#### `/imu`

| Field | Value |
|---|---|
| **Type** | `sensor_msgs/msg/Imu` |
| **Publisher** | `imu_calibration_node` (complementary filter) |
| **Source** | Processed from `/imu/data` |

Calibrated IMU with valid covariance matrices. Use this in your application, EKF, or sensor fusion pipeline.

```
linear_acceleration  →  m/s²        calibrated, covariance set
angular_velocity     →  rad/s       calibrated, covariance set
orientation          →  quaternion  provided by complementary filter, covariance set
```

---

#### `/scan`

| Field | Value |
|---|---|
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Publisher** | `ldlidar_published` |
| **Frame** | Configurable via `frame_id` param (default: `base_laser`) |
| **Topic name** | Configurable via `laser_scan_topic_name` param |

2D laser scan from the LDLiDAR LD06. Primary sensor for obstacle detection, SLAM, and nav2.

---

#### `/pointcloud2d`

| Field | Value |
|---|---|
| **Type** | `sensor_msgs/msg/PointCloud` |
| **Publisher** | `ldlidar_published` |
| **Topic name** | Configurable via `point_cloud_2d_topic_name` param |

Same LiDAR data as `/scan` in point cloud format.

---

### 3.3 Control Layer

---

#### `/diff_controller/cmd_vel`

| Field | Value |
|---|---|
| **Type** | `geometry_msgs/msg/TwistStamped` |
| **Subscriber** | `diff_drive_controller` |

**The primary way to drive the robot.** Publish `twist.linear.x` (m/s) and `twist.angular.z` (rad/s) with a valid header stamp.

```bash
ros2 topic pub /diff_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{ header: { stamp: { sec: 0 } }, twist: { linear: { x: 0.2 }, angular: { z: 0.0 } } }"
```

For keyboard control use the teleop node (Section 4). For navigation, the nav stack publishes here directly.

---

#### `/diff_controller/odom`

| Field | Value |
|---|---|
| **Type** | `nav_msgs/msg/Odometry` |
| **Publisher** | `diff_drive_controller` |

Raw wheel odometry. If the `sensor_fusion` package is running, the EKF consumes this and produces fused odometry. Use this topic directly only for low-level odometry work or debugging.

---

### 3.4 System Topics

Standard ROS2 infrastructure topics. You generally do not need to interact with these.

| Topic | Type | Notes |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Published by `joint_state_broadcaster` |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | Published by `joint_state_broadcaster` |
| `/robot_description` | `std_msgs/msg/String` | URDF from `robot_state_publisher` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Live transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms (URDF frames) |
| `/diff_controller/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | Controller lifecycle |
| `/joint_broadcaster/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | Broadcaster lifecycle |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ROS2 parameter system |
| `/rosout` | `rcl_interfaces/msg/Log` | Aggregated node logs |
| `/clock` | `rosgraph_msgs/msg/Clock` | System clock |

---

## 4. Teleop Node

A keyboard teleoperation node that publishes `TwistStamped` directly to `/diff_controller/cmd_vel`.

### Run

```bash
ros2 run corebot_bringup teleop_node
```

With custom speeds:
```bash
ros2 run corebot_bringup teleop_node --ros-args \
  -p linear_vel:=0.15 -p angular_vel:=0.8
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_vel` | double | `0.30` | Full linear speed (m/s) |
| `angular_vel` | double | `0.50` | Full angular speed (rad/s) |
| `publish_rate` | double | `30.0` | Command publish rate (Hz) |

Uppercase keys (e.g. `W`, `A`) run at half the configured speed.

### Key Bindings

| Key | Action |
|---|---|
| `w` / `↑` | Forward (full speed) |
| `s` / `↓` | Backward (full speed) |
| `a` / `←` | Turn left (full speed) |
| `d` / `→` | Turn right (full speed) |
| `W` / `S` / `A` / `D` | Same directions at half speed |
| `q` | Forward-left diagonal |
| `e` | Forward-right diagonal |
| `z` | Backward-left diagonal |
| `c` | Backward-right diagonal |
| `Space` / any other key | Full stop |
| `x` / `Ctrl-C` | Stop robot and quit |

### Behaviour

The node uses a **direct-velocity, hold-to-move** model:
- Holding a key continuously publishes the velocity command.
- Releasing any key causes a 100 ms read timeout, after which a zero command is published and the robot stops.
- There is no velocity accumulation — each keypress sets a fixed target velocity directly.

---

## 5. ros2_control Interface

For developers writing a **custom controller** or replacing `diff_drive_controller`.

### Hardware Plugin

```
plugin:   corebot_base/CorebotHardware
type:     SystemInterface
```

### Exported State Interfaces

| Joint | Interface | Unit |
|---|---|---|
| `left_wheel_joint` | `position` | rad |
| `left_wheel_joint` | `velocity` | rad/s |
| `right_wheel_joint` | `position` | rad |
| `right_wheel_joint` | `velocity` | rad/s |

### Exported Command Interfaces

| Joint | Interface | Unit |
|---|---|---|
| `left_wheel_joint` | `velocity` | rad/s |
| `right_wheel_joint` | `velocity` | rad/s |

Commands are received as rad/s and converted to PWM (0–999) internally.

### Hardware Parameters (URDF / ros2_control XML)

| Parameter | Type | Description |
|---|---|---|
| `left_wheel_names` | string | Joint name for the left wheel |
| `right_wheel_names` | string | Joint name for the right wheel |
| `enc_ticks_per_rev` | int | Encoder resolution (ticks per full revolution) |
| `loop_rate` | double | Control loop rate in Hz |

> **Physical wiring note:** Left and right encoder channels are physically swapped on the PCB. This is corrected inside the hardware plugin and is transparent to any controller using the standard interfaces. Do not compensate externally.

---

## 6. Serial Protocol Reference

Full-duplex UART between RPi and STM32.

### Connection Parameters

| Parameter | Value |
|---|---|
| Interface | UART (USB-serial adapter) |
| Baud rate | Set via `serial_baudrate_` in hardware plugin |
| Format | 8N1, no flow control |

---

### RX — STM32 → RPi (binary, 20 bytes, little-endian)

```
Byte(s)   Field       Type    Scale             Unit
────────  ─────────   ─────   ────────────────  ──────
0         Header      —       0xAA              —
1         Header      —       0x55              —
2–3       enc_left    int16   absolute count    ticks
4–5       enc_right   int16   absolute count    ticks
6–7       accel_x     int16   ÷ 16384           g
8–9       accel_y     int16   ÷ 16384           g
10–11     accel_z     int16   ÷ 16384           g
12–13     gyro_x      int16   ÷ 131             °/s
14–15     gyro_y      int16   ÷ 131             °/s
16–17     gyro_z      int16   ÷ 131             °/s
18        Checksum    uint8   XOR bytes 2–17    —
19        Footer      —       0xBB              —
```

MPU-6050 default ranges: ±2 g accelerometer, ±250 °/s gyroscope.

Auto port-recovery triggers after 300 consecutive missed frames.

---

### TX — RPi → STM32 (ASCII)

```
Format:   M<left>,<right>\n
Example:  M512,487\n
```

| Field | Range | Description |
|---|---|---|
| `left` | −999 – +999 | Left motor PWM (sign = direction) |
| `right` | −999 – +999 | Right motor PWM (sign = direction) |

rad/s → PWM mapping is handled internally by `set_motor_speeds()`.

---

## 7. Configuration Reference

### Teleop (`corebot_teleop_key`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_vel` | double | `0.30` | Full linear speed (m/s) |
| `angular_vel` | double | `0.50` | Full angular speed (rad/s) |
| `publish_rate` | double | `30.0` | Publish rate (Hz) |

### LiDAR (`ldlidar_published`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `product_name` | string | — | Model (e.g. `LD06`) |
| `laser_scan_topic_name` | string | `scan` | Output scan topic |
| `point_cloud_2d_topic_name` | string | `pointcloud2d` | Output cloud topic |
| `frame_id` | string | `base_laser` | TF frame |
| `port_name` | string | — | Serial port (e.g. `/dev/ttyUSB1`) |
| `serial_baudrate` | int | — | Baud rate |
| `laser_scan_dir` | bool | — | Clockwise if true |
| `angle_crop_min` | double | — | Min publish angle (°) |
| `angle_crop_max` | double | — | Max publish angle (°) |
| `range_min` | double | — | Min range (m) |
| `range_max` | double | — | Max range (m) |

### EKF (`ekf_odom_imu.yaml` — `sensor_fusion` package)

| Input | Source |
|---|---|
| `/diff_controller/odom` | Wheel odometry |
| `/imu` | Calibrated IMU |

See `ekf_odom_imu.yaml` for covariance tuning.

---

## 8. Launch Files

| Launch File | Package | Args | What it starts |
|---|---|---|---|
| `corebot_bringup.launch.py` | `corebot_bringup` | `use_lidar` (default: `true`) | Hardware interface, IMU calibration, controller manager, LiDAR (LD06) |
| `odom_imu_ekf.launch.py` | `sensor_fusion` | — | EKF only — install separately (see Section 2) |

---

*Last updated: June 2026 · Corebot Connected Lab AMR*