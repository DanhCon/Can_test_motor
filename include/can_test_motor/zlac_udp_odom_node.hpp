#ifndef CAN_TEST_MOTOR__ZLAC_UDP_ODOM_NODE_HPP_
#define CAN_TEST_MOTOR__ZLAC_UDP_ODOM_NODE_HPP_

// Node ROS 2 (C++): ZLAC8015D UDP CANopen Odometry Node
// Chuyen doi 1-1 tu scripts/zlac_udp_odom_node.py (501 dong).
// - Nhan /cmd_vel (Twist), loc smoother + breakaway kick, gui UDP 12B xuong STM32.
// - Nhan telemetry 22B tu STM32, tinh odometry Runge-Kutta bac 2,
//   publish /odom + broadcast TF odom -> base_link + /battery_voltage.
// Chuan tham khao: differential_drive (EIU-FABLAB-AMR) + rclcpp Humble.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

// CRC-16/Modbus (poly 0xA001, init 0xFFFF) - dong bo 100% voi STM32 + ban Python.
uint16_t calculate_crc16(const uint8_t * data, size_t len);

class ZlacUdpOdomNode : public rclcpp::Node
{
public:
  explicit ZlacUdpOdomNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ZlacUdpOdomNode();

  // Gui 3 goi stop + dong socket (goi truoc khi huy node).
  void stop();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void control_timer_callback();
  void reset_odom_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void udp_receive_loop();

  static double move_towards(double curr, double target, double max_step, double min_kick);

  // ---- Parameters ----
  std::string stm32_ip_;
  int stm32_port_;
  int local_port_;

  double wheel_radius_;
  double wheel_base_;
  int cpr_;
  bool motor_b_reverse_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  double control_rate_;
  double cmd_vel_timeout_;

  double max_linear_velocity_;
  double max_angular_velocity_;
  bool enable_smoother_;
  double linear_accel_;
  double angular_accel_;
  double min_breakaway_vel_;

  // ---- Trang thai van hanh ----
  double target_linear_;
  double target_angular_;
  double current_linear_;
  double current_angular_;

  double x_;
  double y_;
  double theta_;
  double v_x_;
  double w_z_;

  int32_t last_pos_a_;
  int32_t last_pos_b_;
  bool has_last_pos_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_cmd_vel_time_;

  rclcpp::Time last_udp_rx_time_;
  bool ethernet_connected_;
  double current_a_;
  double current_b_;

  std::mutex odom_mutex_;
  std::mutex comm_mutex_;  // bao last_udp_rx_time_, ethernet_connected_, current_a/b
  std::atomic<bool> is_running_;

  // ---- UDP socket (POSIX) ----
  int sock_fd_;
  struct sockaddr_in dest_addr_;

  // ---- ROS interfaces ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::thread rx_thread_;
};

#endif  // CAN_TEST_MOTOR__ZLAC_UDP_ODOM_NODE_HPP_
