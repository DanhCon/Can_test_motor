#include "can_test_motor/legacy/zlac_udp_odom_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>

// POSIX UDP
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint16_t calculate_crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

double ZlacUdpOdomNode::move_towards(double curr, double target, double max_step, double min_kick)
{
  if (min_kick > 0.0 && std::abs(curr) < 0.001 && std::abs(target) > 0.001) {
    if (target > 0.0) {
      return std::min(min_kick, target);
    } else {
      return std::max(-min_kick, target);
    }
  } else if (curr < target) {
    return std::min(curr + max_step, target);
  } else if (curr > target) {
    return std::max(curr - max_step, target);
  }
  return curr;
}

ZlacUdpOdomNode::ZlacUdpOdomNode(const rclcpp::NodeOptions & options)
: Node("zlac_udp_odom_node", options),
  stm32_port_(8888),
  local_port_(8888),
  wheel_radius_(0.0535),
  wheel_base_(0.45),
  cpr_(4096),
  motor_b_reverse_(true),
  publish_tf_(true),
  control_rate_(50.0),
  cmd_vel_timeout_(0.25),
  max_linear_velocity_(0.3),
  max_angular_velocity_(0.8),
  enable_smoother_(false),
  linear_accel_(0.8),
  angular_accel_(1.2),
  min_breakaway_vel_(0.04),
  target_linear_(0.0),
  target_angular_(0.0),
  current_linear_(0.0),
  current_angular_(0.0),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  v_x_(0.0),
  w_z_(0.0),
  last_pos_a_(0),
  last_pos_b_(0),
  has_last_pos_(false),
  ethernet_connected_(true),
  current_a_(0.0),
  current_b_(0.0),
  is_running_(true),
  sock_fd_(-1)
{
  // ---- Khai bao parameters (giong het ban Python) ----
  this->declare_parameter("stm32_ip", "192.168.1.100");
  this->declare_parameter("stm32_port", 8888);
  this->declare_parameter("local_port", 8888);

  this->declare_parameter("wheel_radius", 0.0535);
  this->declare_parameter("wheel_base", 0.45);
  this->declare_parameter("cpr", 4096);
  this->declare_parameter("motor_b_reverse", true);

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("control_rate", 50.0);
  this->declare_parameter("cmd_vel_timeout", 0.25);

  this->declare_parameter("max_linear_velocity", 0.3);
  this->declare_parameter("max_angular_velocity", 0.8);

  this->declare_parameter("enable_smoother", false);
  this->declare_parameter("linear_accel", 0.8);
  this->declare_parameter("angular_accel", 1.2);
  this->declare_parameter("min_breakaway_velocity", 0.04);

  this->get_parameter("stm32_ip", stm32_ip_);
  this->get_parameter("stm32_port", stm32_port_);
  this->get_parameter("local_port", local_port_);
  this->get_parameter("wheel_radius", wheel_radius_);
  this->get_parameter("wheel_base", wheel_base_);
  this->get_parameter("cpr", cpr_);
  this->get_parameter("motor_b_reverse", motor_b_reverse_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("control_rate", control_rate_);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("enable_smoother", enable_smoother_);
  this->get_parameter("linear_accel", linear_accel_);
  this->get_parameter("angular_accel", angular_accel_);
  this->get_parameter("min_breakaway_velocity", min_breakaway_vel_);

  last_odom_time_ = this->now();
  last_cmd_vel_time_ = this->now();
  last_udp_rx_time_ = this->now();

  // ---- Socket UDP ----
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Khong the tao Socket UDP!");
  } else {
    int reuse = 1;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(static_cast<uint16_t>(local_port_));
    if (::bind(
        sock_fd_, reinterpret_cast<struct sockaddr *>(&local_addr),
        sizeof(local_addr)) < 0)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Khong the bind Socket UDP tai port %d",
        local_port_);
      ::close(sock_fd_);
      sock_fd_ = -1;
    } else {
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 50000;  // timeout 50ms cho recvfrom
      ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      RCLCPP_INFO(
        this->get_logger(),
        "Da mo Socket UDP tai port %d, gui toi STM32 %s:%d",
        local_port_, stm32_ip_.c_str(), stm32_port_);
    }
  }
  std::memset(&dest_addr_, 0, sizeof(dest_addr_));
  dest_addr_.sin_family = AF_INET;
  dest_addr_.sin_port = htons(static_cast<uint16_t>(stm32_port_));
  ::inet_pton(AF_INET, stm32_ip_.c_str(), &dest_addr_.sin_addr);

  // ---- ROS interfaces ----
  rclcpp::QoS qos(10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos,
    std::bind(&ZlacUdpOdomNode::cmd_vel_callback, this, std::placeholders::_1));

  reset_odom_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_odom",
    std::bind(
      &ZlacUdpOdomNode::reset_odom_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ZlacUdpOdomNode::control_timer_callback, this));

  rx_thread_ = std::thread(&ZlacUdpOdomNode::udp_receive_loop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "[*] Node ZLAC8015D UDP Odom khoi dong (R=%.1fmm, L=%.1fmm).",
    wheel_radius_ * 1000.0, wheel_base_ * 1000.0);
}

ZlacUdpOdomNode::~ZlacUdpOdomNode()
{
  is_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  RCLCPP_INFO(this->get_logger(), "Da dong Socket va tat an toan.");
}

void ZlacUdpOdomNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!is_running_) {
    return;
  }
  last_cmd_vel_time_ = this->now();

  double lin_x = msg->linear.x;
  double ang_z = msg->angular.z;

  if (std::abs(lin_x) < 0.005) {
    lin_x = 0.0;
  }
  if (std::abs(ang_z) < 0.005) {
    ang_z = 0.0;
  }

  target_linear_ = std::clamp(lin_x, -max_linear_velocity_, max_linear_velocity_);
  target_angular_ = std::clamp(ang_z, -max_angular_velocity_, max_angular_velocity_);
}

void ZlacUdpOdomNode::reset_odom_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Nhan yeu cau reset Odometry ve 0...");
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    v_x_ = 0.0;
    w_z_ = 0.0;
    has_last_pos_ = false;
  }
  res->success = true;
  res->message = "Da reset toa do Odometry ve 0 thanh cong.";
}

void ZlacUdpOdomNode::control_timer_callback()
{
  if (!is_running_ || sock_fd_ < 0) {
    return;
  }

  const rclcpp::Time now = this->now();
  const double dt = 1.0 / control_rate_;

  // 1. Watchdog cmd_vel timeout -> giam toc ve 0
  const double since_cmd = (now - last_cmd_vel_time_).seconds();
  if (since_cmd > cmd_vel_timeout_) {
    target_linear_ = 0.0;
    target_angular_ = 0.0;
  }

  // 2. Watchdog mat ket noi Ethernet (qua 1.0s khong nhan telemetry)
  double since_rx = 0.0;
  {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    since_rx = (now - last_udp_rx_time_).seconds();
  }
  if (since_rx > 1.0) {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "CANH BAO: MAT KET NOI ETHERNET TOI STM32! (%.1fs khong nhan UDP)", since_rx);
    ethernet_connected_ = false;
  }

  // 3. Velocity smoother + breakaway kick
  if (enable_smoother_) {
    const double step_lin = linear_accel_ * dt;
    const double step_ang = angular_accel_ * dt;
    current_linear_ = move_towards(current_linear_, target_linear_, step_lin, min_breakaway_vel_);
    current_angular_ = move_towards(current_angular_, target_angular_, step_ang, 0.0);
  } else {
    current_linear_ = target_linear_;
    current_angular_ = target_angular_;
  }

  // 4. Dong goi 12B: AA 55 + float v + float omega + CRC16 (little-endian)
  uint8_t packet[12];
  packet[0] = 0xAA;
  packet[1] = 0x55;
  const float v = static_cast<float>(current_linear_);
  const float w = static_cast<float>(current_angular_);
  std::memcpy(&packet[2], &v, sizeof(float));
  std::memcpy(&packet[6], &w, sizeof(float));
  const uint16_t crc = calculate_crc16(packet, 10);
  std::memcpy(&packet[10], &crc, sizeof(uint16_t));

  const ssize_t sent = ::sendto(
    sock_fd_, packet, sizeof(packet), 0,
    reinterpret_cast<struct sockaddr *>(&dest_addr_), sizeof(dest_addr_));
  if (sent < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Loi khi gui goi UDP toi STM32");
  }
}

void ZlacUdpOdomNode::udp_receive_loop()
{
  const double meter_per_count =
    (2.0 * M_PI * wheel_radius_) / static_cast<double>(cpr_);

  uint8_t buf[1024];
  while (is_running_) {
    if (sock_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    const ssize_t n = ::recvfrom(sock_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
    if (n < 22) {
      continue;  // timeout hoac goi rac
    }
    if (buf[0] != 0x55 || buf[1] != 0xAA) {
      continue;
    }

    int16_t vel_a_raw, vel_b_raw, cur_a_raw, cur_b_raw;
    int32_t pos_a, pos_b;
    uint16_t err_code, v_bus_raw;
    std::memcpy(&vel_a_raw, &buf[2], sizeof(int16_t));
    std::memcpy(&vel_b_raw, &buf[4], sizeof(int16_t));
    std::memcpy(&pos_a, &buf[6], sizeof(int32_t));
    std::memcpy(&pos_b, &buf[10], sizeof(int32_t));
    std::memcpy(&err_code, &buf[14], sizeof(uint16_t));
    std::memcpy(&cur_a_raw, &buf[16], sizeof(int16_t));
    std::memcpy(&cur_b_raw, &buf[18], sizeof(int16_t));
    std::memcpy(&v_bus_raw, &buf[20], sizeof(uint16_t));
    (void)vel_a_raw;
    (void)vel_b_raw;

    const rclcpp::Time now = this->now();
    {
      std::lock_guard<std::mutex> lock(comm_mutex_);
      if (!ethernet_connected_) {
        RCLCPP_INFO(this->get_logger(), "DA KHOI PHUC KET NOI ETHERNET TOI STM32.");
        ethernet_connected_ = true;
      }
      last_udp_rx_time_ = now;

      current_a_ = static_cast<double>(cur_a_raw) / 10.0;
      current_b_ = static_cast<double>(cur_b_raw) / 10.0;
    }

    if (err_code != 0) {
      if (err_code == 0xEEEE) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "CANH BAO: KICH HOAT BAO VE KET TAI / QUA DONG (0xEEEE)!");
      } else if (err_code == 0xEE01) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "CANH BAO: MAT KET NOI CAN GIUA STM32 VA ZLAC (0xEE01)!");
      } else {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "CANH BAO LOI PHAN CUNG DRIVER: 0x%04X", err_code);
      }
    }

    // Dien ap DC Bus
    const double v_bus = (v_bus_raw < 1000) ?
      (static_cast<double>(v_bus_raw) / 10.0) :
      (static_cast<double>(v_bus_raw) / 100.0);
    std_msgs::msg::Float32 batt_msg;
    batt_msg.data = static_cast<float>(v_bus);
    battery_pub_->publish(batt_msg);

    // ---- Odometry tu encoder (khoa mutex) ----
    double cur_x, cur_y, cur_theta, cur_vx, cur_wz;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_last_pos_) {
        last_pos_a_ = pos_a;
        last_pos_b_ = pos_b;
        has_last_pos_ = true;
        last_odom_time_ = now;
        continue;
      }
      const double dt = (now - last_odom_time_).seconds();
      if (dt <= 0.0001) {
        continue;
      }
      last_odom_time_ = now;

      const int64_t delta_a = static_cast<int64_t>(pos_a) - last_pos_a_;
      int64_t delta_b = static_cast<int64_t>(pos_b) - last_pos_b_;
      last_pos_a_ = pos_a;
      last_pos_b_ = pos_b;
      if (motor_b_reverse_) {
        delta_b = -delta_b;
      }

      const double dist_l = static_cast<double>(delta_a) * meter_per_count;
      const double dist_r = static_cast<double>(delta_b) * meter_per_count;
      const double dist_center = (dist_l + dist_r) / 2.0;
      const double d_theta = (dist_r - dist_l) / wheel_base_;

      const double theta_mid = theta_ + d_theta / 2.0;
      x_ += dist_center * std::cos(theta_mid);
      y_ += dist_center * std::sin(theta_mid);
      theta_ += d_theta;
      theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

      v_x_ = dist_center / dt;
      w_z_ = d_theta / dt;

      cur_x = x_;
      cur_y = y_;
      cur_theta = theta_;
      cur_vx = v_x_;
      cur_wz = w_z_;
    }

    // ---- Publish /odom + TF (ngoai mutex) ----
    const double cy = std::cos(cur_theta * 0.5);
    const double sy = std::sin(cur_theta * 0.5);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = cur_x;
    odom_msg.pose.pose.position.y = cur_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;
    odom_msg.pose.covariance = {
      1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
    odom_msg.twist.twist.linear.x = cur_vx;
    odom_msg.twist.twist.angular.z = cur_wz;
    odom_msg.twist.covariance = {
      1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = cur_x;
      tf.transform.translation.y = cur_y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = 0.0;
      tf.transform.rotation.y = 0.0;
      tf.transform.rotation.z = sy;
      tf.transform.rotation.w = cy;
      tf_broadcaster_->sendTransform(tf);
    }
  }
}

void ZlacUdpOdomNode::stop()
{
  is_running_ = false;
  RCLCPP_INFO(this->get_logger(), "Dang dung Node, gui lenh phanh khan cap toi STM32...");
  for (int i = 0; i < 3; ++i) {
    uint8_t packet[12];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    const float zero = 0.0F;
    std::memcpy(&packet[2], &zero, sizeof(float));
    std::memcpy(&packet[6], &zero, sizeof(float));
    const uint16_t crc = calculate_crc16(packet, 10);
    std::memcpy(&packet[10], &crc, sizeof(uint16_t));
    if (sock_fd_ >= 0) {
      ::sendto(
        sock_fd_, packet, sizeof(packet), 0,
        reinterpret_cast<struct sockaddr *>(&dest_addr_), sizeof(dest_addr_));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZlacUdpOdomNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Runtime error: %s", e.what());
  }
  node->stop();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
