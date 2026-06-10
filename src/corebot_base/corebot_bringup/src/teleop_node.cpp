// corebot_teleop_key.cpp
// ----------------------
// Keyboard teleoperation node for a small educational differential-drive robot.
// Publishes geometry_msgs/TwistStamped to /diff_drive_controller/cmd_vel.
//
// DIRECT-VELOCITY MODE — no accumulation:
//   Hold a key  → robot moves at a fixed velocity
//   Release key → robot stops (zero published after 100 ms timeout)
//
// Velocity limits tuned for a small classroom robot (CoreBot-class):
//   Linear  : 0.20 m/s   (half-speed: 0.10 m/s  via uppercase)
//   Angular : 1.00 rad/s (half-speed: 0.50 rad/s via uppercase)
//
// Key bindings:
//   w / ↑        : forward  at full linear speed
//   s / ↓        : backward at full linear speed
//   a / ←        : turn left  at full angular speed
//   d / →        : turn right at full angular speed
//   W (Shift+w)  : forward  at half speed
//   S (Shift+s)  : backward at half speed
//   A (Shift+a)  : turn left  at half angular speed
//   D (Shift+d)  : turn right at half angular speed
//   q            : forward-left  diagonal (full speed)
//   e            : forward-right diagonal (full speed)
//   z            : backward-left diagonal (full speed)
//   c            : backward-right diagonal (full speed)
//   Space / any other key : immediate FULL STOP
//   x  / Ctrl-C           : stop robot and quit
//
// Build (add to CMakeLists.txt):
//   add_executable(corebot_teleop_key src/corebot_teleop_key.cpp)
//   ament_target_dependencies(corebot_teleop_key rclcpp geometry_msgs)
//   install(TARGETS corebot_teleop_key DESTINATION lib/${PROJECT_NAME})
//
// Run:
//   ros2 run <your_package> corebot_teleop_key
//   ros2 run <your_package> corebot_teleop_key --ros-args \
//            -p linear_vel:=0.15 -p angular_vel:=0.8

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>

// ── terminal helpers ──────────────────────────────────────────────────────────

static termios g_orig_termios;

static void restore_terminal()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void set_raw_terminal()
{
  tcgetattr(STDIN_FILENO, &g_orig_termios);
  atexit(restore_terminal);

  termios raw  = g_orig_termios;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  raw.c_cc[VMIN]  = 0;   // non-blocking
  raw.c_cc[VTIME] = 1;   // 100 ms read timeout — used as key-release detector
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Returns the pressed key code, or -1 on timeout (key released / nothing held).
// Arrow keys → synthetic codes 1000-1003.
static int read_key()
{
  unsigned char buf[3] = {0, 0, 0};
  int n = static_cast<int>(read(STDIN_FILENO, buf, sizeof(buf)));
  if (n <= 0) return -1;   // timeout = no key held

  // Arrow keys: ESC [ A/B/C/D
  if (n == 3 && buf[0] == 0x1B && buf[1] == '[') {
    switch (buf[2]) {
      case 'A': return 1000;  // ↑
      case 'B': return 1001;  // ↓
      case 'C': return 1002;  // →
      case 'D': return 1003;  // ←
    }
  }
  return static_cast<int>(buf[0]);
}

// ── display ───────────────────────────────────────────────────────────────────

static void print_banner(double lin, double ang)
{
  printf("\033[2J\033[H");
  printf("╔════════════════════════════════════════════════════════╗\n");
  printf("║     CoreBot Keyboard Teleoperation — Direct Mode      ║\n");
  printf("╠════════════════════════════════════════════════════════╣\n");
  printf("║  HOLD key to move  ·  RELEASE to stop                 ║\n");
  printf("╠══════════════════════════╦═════════════════════════════╣\n");
  printf("║  w / ↑  forward (full)   ║  W   forward (half)        ║\n");
  printf("║  s / ↓  backward (full)  ║  S   backward (half)       ║\n");
  printf("║  a / ←  left (full)      ║  A   left (half)           ║\n");
  printf("║  d / →  right (full)     ║  D   right (half)          ║\n");
  printf("║  q  fwd-left             ║  e   fwd-right             ║\n");
  printf("║  z  bwd-left             ║  c   bwd-right             ║\n");
  printf("╠══════════════════════════╩═════════════════════════════╣\n");
  printf("║  Space / any other key : FULL STOP                    ║\n");
  printf("║  x  / Ctrl-C           : STOP & QUIT                 ║\n");
  printf("╠════════════════════════════════════════════════════════╣\n");
  printf("║  Topic  : /diff_drive_controller/cmd_vel              ║\n");
  printf("║  Linear : %.2f m/s (half: %.2f m/s)                ║\n", lin, lin * 0.5);
  printf("║  Angular: %.2f rad/s (half: %.2f rad/s)           ║\n", ang, ang * 0.5);
  printf("╚════════════════════════════════════════════════════════╝\n");
}

static void print_status(double lin, double ang, const char * label)
{
  printf("\033[15;0H  %-18s  lin: %+.3f m/s   ang: %+.3f rad/s   \n",
         label, lin, ang);
  fflush(stdout);
}

// ── node ──────────────────────────────────────────────────────────────────────

class CoreBotTeleop : public rclcpp::Node
{
public:
  CoreBotTeleop()
  : Node("corebot_teleop_key"),
    cmd_lin_(0.0),
    cmd_ang_(0.0)
  {
    // ── parameters ──────────────────────────────────────────────────────────
    this->declare_parameter("linear_vel",   0.30);  // m/s   full speed
    this->declare_parameter("angular_vel",  0.50);  // rad/s full speed
    this->declare_parameter("publish_rate", 30.0);  // Hz

    lin_vel_ = this->get_parameter("linear_vel").as_double();
    ang_vel_ = this->get_parameter("angular_vel").as_double();
    double hz = this->get_parameter("publish_rate").as_double();

    // ── publisher ────────────────────────────────────────────────────────────
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_controller/cmd_vel", 10);

    // ── single loop timer ────────────────────────────────────────────────────
    // read() inside loop() blocks up to VTIME=100 ms waiting for a byte.
    // If a key is held, the OS delivers repeated bytes → motion continues.
    // If released, read() times out → returns -1 → zero command published.
    // Timer period matches VTIME so we don't stack blocking calls.
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
    timer_ = this->create_wall_timer(period, [this]() { loop(); });

    set_raw_terminal();
    print_banner(lin_vel_, ang_vel_);
    print_status(0.0, 0.0, "STOPPED");
  }

  ~CoreBotTeleop()
  {
    cmd_lin_ = 0.0;
    cmd_ang_ = 0.0;
    publish_cmd();
    printf("\033[16;0H\n[teleop] Robot stopped. Goodbye!\n");
  }

private:
  void publish_cmd()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x  = cmd_lin_;
    msg.twist.angular.z = cmd_ang_;
    pub_->publish(msg);
  }

  void loop()
  {
    int key = read_key();

    double new_lin = 0.0;
    double new_ang = 0.0;
    const char * label = "STOPPED";

    switch (key) {
      // ── full-speed cardinal ───────────────────────────────────────────────
      case 'w': case 1000:
        new_lin =  lin_vel_;  label = "FORWARD";       break;
      case 's': case 1001:
        new_lin = -lin_vel_;  label = "BACKWARD";      break;
      case 'a': case 1003:
        new_ang =  ang_vel_;  label = "TURN LEFT";     break;
      case 'd': case 1002:
        new_ang = -ang_vel_;  label = "TURN RIGHT";    break;

      // ── half-speed cardinal ───────────────────────────────────────────────
      case 'W':
        new_lin =  lin_vel_ * 0.5;  label = "FWD slow";    break;
      case 'S':
        new_lin = -lin_vel_ * 0.5;  label = "BWD slow";    break;
      case 'A':
        new_ang =  ang_vel_ * 0.5;  label = "LEFT slow";   break;
      case 'D':
        new_ang = -ang_vel_ * 0.5;  label = "RIGHT slow";  break;

      // ── diagonal combos ───────────────────────────────────────────────────
      case 'q':
        new_lin =  lin_vel_; new_ang =  ang_vel_;  label = "FWD-LEFT";   break;
      case 'e':
        new_lin =  lin_vel_; new_ang = -ang_vel_;  label = "FWD-RIGHT";  break;
      case 'z':
        new_lin = -lin_vel_; new_ang =  ang_vel_;  label = "BWD-LEFT";   break;
      case 'c':
        new_lin = -lin_vel_; new_ang = -ang_vel_;  label = "BWD-RIGHT";  break;

      // ── quit ──────────────────────────────────────────────────────────────
      case 'x': case 3:   // 'x' or Ctrl-C
        cmd_lin_ = 0.0;
        cmd_ang_ = 0.0;
        publish_cmd();
        rclcpp::shutdown();
        return;

      // ── no key held / Space / anything else → stop ────────────────────────
      default:
        new_lin = 0.0;
        new_ang = 0.0;
        label   = "STOPPED";
        break;
    }

    cmd_lin_ = new_lin;
    cmd_ang_ = new_ang;
    publish_cmd();
    print_status(cmd_lin_, cmd_ang_, label);
  }

  // ── members ───────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double cmd_lin_;
  double cmd_ang_;
  double lin_vel_;
  double ang_vel_;
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoreBotTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}