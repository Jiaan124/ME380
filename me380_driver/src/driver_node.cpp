/*
 * ROS2 Humble driver node for ME380 robot.
 * Subscribes to sensor_msgs/msg/JointState and sends commands to the
 * motor controller (ESP32/Arduino with driver_firmware) over serial.
 *
 * Joint mapping (by name):
 *   joint1, j1 -> stepper J1 (delta degrees)
 *   joint2, j2 -> stepper J2 (delta degrees)
 *   joint3, j3 -> stepper J3 (delta degrees)
 *   joint4, j4 -> differential servo A/B angle (degrees)
 *   joint5, j5 -> differential servo A/B angle (degrees)
 *   joint6, gripper -> end effector (0 = open, 1 = closed)
 *
 * Position units: JointState uses radians; we convert to degrees for the firmware.
 */

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace me380_driver {

constexpr double RAD_TO_DEG = 180.0 / M_PI;

class DriverNode : public rclcpp::Node {
 public:
  DriverNode() : Node("driver_node"), serial_fd_(-1) {
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 9600);
    declare_parameter<std::string>("joint_state_topic", "joint_states");
    declare_parameter<double>("max_stepper_delta_per_msg", 360.0);

    serial_port_ = get_parameter("serial_port").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    max_stepper_delta_ = get_parameter("max_stepper_delta_per_msg").as_double();

    if (!openSerial()) {
      RCLCPP_WARN(get_logger(), "Serial port not open. Commands will be logged only.");
    }

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10,
        std::bind(&DriverNode::onJointState, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to %s; sending to %s @ %d",
                joint_state_topic_.c_str(), serial_port_.c_str(), baud_rate_);
  }

  ~DriverNode() { closeSerial(); }

 private:
  bool openSerial() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", serial_port_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    speed_t speed = B9600;
    switch (baud_rate_) {
      case 1200:   speed = B1200;   break;
      case 2400:   speed = B2400;   break;
      case 4800:   speed = B4800;   break;
      case 9600:   speed = B9600;   break;
      case 19200:  speed = B19200;  break;
      case 38400:  speed = B38400;  break;
      case 57600:  speed = B57600;  break;
      case 115200: speed = B115200; break;
      default:
        RCLCPP_WARN(get_logger(), "Unsupported baud %d, using 9600", baud_rate_);
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    RCLCPP_INFO(get_logger(), "Serial %s opened at %d baud", serial_port_.c_str(), baud_rate_);
    return true;
  }

  void closeSerial() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  static int indexOf(const std::vector<std::string>& names, const std::string& joint_name) {
    for (size_t i = 0; i < names.size(); ++i)
      if (names[i] == joint_name) return static_cast<int>(i);
    return -1;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto& names = msg->name;
    const auto& position = msg->position;

    if (names.size() != position.size() || names.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "JointState name/position size mismatch or empty");
      return;
    }

    int i1 = indexOf(names, "joint1");
    if (i1 < 0) i1 = indexOf(names, "j1");
    int i2 = indexOf(names, "joint2");
    if (i2 < 0) i2 = indexOf(names, "j2");
    int i3 = indexOf(names, "joint3");
    if (i3 < 0) i3 = indexOf(names, "j3");
    int i4 = indexOf(names, "joint4");
    if (i4 < 0) i4 = indexOf(names, "j4");
    int i5 = indexOf(names, "joint5");
    if (i5 < 0) i5 = indexOf(names, "j5");
    int i6 = indexOf(names, "joint6");
    if (i6 < 0) i6 = indexOf(names, "gripper");

    double j1_rad = (i1 >= 0) ? position[static_cast<size_t>(i1)] : 0.0;
    double j2_rad = (i2 >= 0) ? position[static_cast<size_t>(i2)] : 0.0;
    double j3_rad = (i3 >= 0) ? position[static_cast<size_t>(i3)] : 0.0;
    double j4_rad = (i4 >= 0) ? position[static_cast<size_t>(i4)] : 0.0;
    double j5_rad = (i5 >= 0) ? position[static_cast<size_t>(i5)] : 0.0;
    double j6_rad = (i6 >= 0) ? position[static_cast<size_t>(i6)] : 0.0;

    double j1_deg = j1_rad * RAD_TO_DEG;
    double j2_deg = j2_rad * RAD_TO_DEG;
    double j3_deg = j3_rad * RAD_TO_DEG;
    double j4_deg = j4_rad * RAD_TO_DEG;
    double j5_deg = j5_rad * RAD_TO_DEG;

    double d1 = j1_deg - last_j1_deg_;
    double d2 = j2_deg - last_j2_deg_;
    double d3 = j3_deg - last_j3_deg_;
    last_j1_deg_ = j1_deg;
    last_j2_deg_ = j2_deg;
    last_j3_deg_ = j3_deg;

    d1 = std::max(-max_stepper_delta_, std::min(max_stepper_delta_, d1));
    d2 = std::max(-max_stepper_delta_, std::min(max_stepper_delta_, d2));
    d3 = std::max(-max_stepper_delta_, std::min(max_stepper_delta_, d3));

    int gripper = (j6_rad > 0.5) ? 1 : 0;

    char buf[128];
    int len = snprintf(buf, sizeof(buf), "%.4f %.4f %.4f %.4f %.4f %d\n",
                       d1, d2, d3, j4_deg, j5_deg, gripper);

    if (serial_fd_ >= 0) {
      ssize_t n = write(serial_fd_, buf, static_cast<size_t>(len));
      if (n != len)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Serial write incomplete: %zd / %d", n, len);
    } else {
      RCLCPP_DEBUG(get_logger(), "Would send: %s", std::string(buf, static_cast<size_t>(len)).c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::string serial_port_;
  int baud_rate_;
  std::string joint_state_topic_;
  double max_stepper_delta_;
  int serial_fd_;

  double last_j1_deg_ = 0.0;
  double last_j2_deg_ = 0.0;
  double last_j3_deg_ = 0.0;
};

}  // namespace me380_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<me380_driver::DriverNode>());
  rclcpp::shutdown();
  return 0;
}
