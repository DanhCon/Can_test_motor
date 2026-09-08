#ifndef CAN_TEST_MOTOR__TELEOP_JOY_HPP_
#define CAN_TEST_MOTOR__TELEOP_JOY_HPP_

// Node ROS 2 (C++): Gamepad Teleop - chuyen doi 1-1 tu scripts/teleop_joy.py.
// - Sub /joy, anh xa truc + nut (deadman/turbo/estop/reset) -> pub /cmd_vel.
// - Client /reset_odom + pub /set_pose cho EKF.

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

class GamepadTeleopNode : public rclcpp::Node
{
public:
  explicit GamepadTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GamepadTeleopNode();

  // Gui lenh dung truoc khi tat (giong finally ban Python).
  void publish_stop();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback();
  void call_reset_odom_service();
  void service_response_callback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  // ---- Parameters ----
  int axis_linear_;
  int axis_angular_;
  double deadzone_;

  double scale_linear_normal_;
  double scale_angular_normal_;
  double scale_linear_turbo_;
  double scale_angular_turbo_;

  bool enable_deadman_;
  int btn_deadman_;
  int btn_turbo_;
  int btn_estop_;
  int btn_reset_odom_;

  double publish_rate_;

  // ---- Trang thai ----
  double v_out_;
  double omega_out_;
  bool estop_active_;
  int last_reset_btn_state_;
  int last_estop_btn_state_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_odom_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // CAN_TEST_MOTOR__TELEOP_JOY_HPP_
