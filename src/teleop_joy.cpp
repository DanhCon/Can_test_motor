#include "can_test_motor/teleop_joy.hpp"

#include <chrono>

GamepadTeleopNode::GamepadTeleopNode(const rclcpp::NodeOptions & options)
: Node("gamepad_teleop_node", options),
  axis_linear_(1),
  axis_angular_(3),
  deadzone_(0.08),
  scale_linear_normal_(0.3),
  scale_angular_normal_(0.5),
  scale_linear_turbo_(0.3),
  scale_angular_turbo_(0.5),
  enable_deadman_(true),
  btn_deadman_(4),
  btn_turbo_(5),
  btn_estop_(1),
  btn_reset_odom_(3),
  publish_rate_(20.0),
  v_out_(0.0),
  omega_out_(0.0),
  estop_active_(false),
  deadman_pressed_(false),
  stop_sent_count_(3),
  last_reset_btn_state_(0),
  last_estop_btn_state_(0)
{
  this->declare_parameter("axis_linear", 1);
  this->declare_parameter("axis_angular", 2);
  this->declare_parameter("deadzone", 0.08);

  this->declare_parameter("scale_linear_normal", 0.3);
  this->declare_parameter("scale_angular_normal", 0.5);
  this->declare_parameter("scale_linear_turbo", 0.3);
  this->declare_parameter("scale_angular_turbo", 0.5);

  this->declare_parameter("enable_deadman", true);
  this->declare_parameter("btn_deadman", 10);
  this->declare_parameter("btn_turbo", 11);
  this->declare_parameter("btn_estop", 1);
  this->declare_parameter("btn_reset_odom", 3);

  this->declare_parameter("publish_rate", 20.0);

  this->get_parameter("axis_linear", axis_linear_);
  this->get_parameter("axis_angular", axis_angular_);
  this->get_parameter("deadzone", deadzone_);
  this->get_parameter("scale_linear_normal", scale_linear_normal_);
  this->get_parameter("scale_angular_normal", scale_angular_normal_);
  this->get_parameter("scale_linear_turbo", scale_linear_turbo_);
  this->get_parameter("scale_angular_turbo", scale_angular_turbo_);
  this->get_parameter("enable_deadman", enable_deadman_);
  this->get_parameter("btn_deadman", btn_deadman_);
  this->get_parameter("btn_turbo", btn_turbo_);
  this->get_parameter("btn_estop", btn_estop_);
  this->get_parameter("btn_reset_odom", btn_reset_odom_);
  this->get_parameter("publish_rate", publish_rate_);

  rclcpp::QoS qos(10);
  cmd_joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/input_joy/cmd_vel", qos);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&GamepadTeleopNode::joy_callback, this, std::placeholders::_1));
  reset_odom_client_ = this->create_client<std_srvs::srv::Trigger>("reset_odom");
  set_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "set_pose", qos);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GamepadTeleopNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "[*] Node Gamepad Teleop da khoi dong san sang!");
}

GamepadTeleopNode::~GamepadTeleopNode()
{
  RCLCPP_INFO(this->get_logger(), "Gamepad Teleop node is stopping...");
}

void GamepadTeleopNode::publish_stop()
{
  geometry_msgs::msg::Twist stop;
  stop.linear.x = 0.0;
  stop.angular.z = 0.0;
  cmd_joy_pub_->publish(stop);
}

void GamepadTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const auto & btns = msg->buttons;
  const auto btn = [&btns](int idx) -> int {
      if (idx < 0 || idx >= static_cast<int>(btns.size())) {
        return 0;
      }
      return btns[static_cast<size_t>(idx)];
    };

  // 1. E-Stop (bat suon duong, toggle)
  if (btn_estop_ < static_cast<int>(btns.size())) {
    const int e_state = btn(btn_estop_);
    if (e_state == 1 && last_estop_btn_state_ == 0) {
      estop_active_ = !estop_active_;
      if (estop_active_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[E-STOP] PHANH KHAN CAP DA DUOC KICH HOAT! (Bam lai nut B de mo khoa)");
      } else {
        RCLCPP_INFO(this->get_logger(), "[E-STOP] Da mo khoa phanh khan cap.");
      }
    }
    last_estop_btn_state_ = e_state;
  }
  if (estop_active_) {
    v_out_ = 0.0;
    omega_out_ = 0.0;
    return;
  }

  // 2. Reset odometry (bat suon duong)
  const int r_state = btn(btn_reset_odom_);
  if (r_state == 1 && last_reset_btn_state_ == 0) {
    call_reset_odom_service();
  }
  last_reset_btn_state_ = r_state;

  // 3. Deadman: ho tro ca USB L1 (4), L2 (6), Bluetooth (9), vi tri 10, 11 va tham so btn_deadman_
  bool is_deadman = true;
  if (enable_deadman_) {
    is_deadman = (btn(btn_deadman_) == 1) || (btn(10) == 1) || (btn(9) == 1) || (btn(4) == 1) || (btn(6) == 1);
  }
  deadman_pressed_ = is_deadman;

  if (!is_deadman) {
    v_out_ = 0.0;
    omega_out_ = 0.0;
    return;
  }

  // 4. Turbo: ho tro index 5, 7, 11 va tham so btn_turbo_
  const bool is_turbo = (btn(btn_turbo_) == 1) || (btn(11) == 1) || (btn(5) == 1) || (btn(7) == 1);
  const double scale_lin = is_turbo ? scale_linear_turbo_ : scale_linear_normal_;
  const double scale_ang = is_turbo ? scale_angular_turbo_ : scale_angular_normal_;

  // 5. Doc truc + deadzone
  double raw_lin = 0.0;
  double raw_ang = 0.0;
  if (axis_linear_ >= 0 && axis_linear_ < static_cast<int>(msg->axes.size())) {
    raw_lin = msg->axes[static_cast<size_t>(axis_linear_)];
  }
  if (axis_angular_ >= 0 && axis_angular_ < static_cast<int>(msg->axes.size())) {
    raw_ang = msg->axes[static_cast<size_t>(axis_angular_)];
  }
  if (std::abs(raw_lin) < deadzone_) {
    raw_lin = 0.0;
  }
  if (std::abs(raw_ang) < deadzone_) {
    raw_ang = 0.0;
  }

  v_out_ = raw_lin * scale_lin;
  omega_out_ = raw_ang * scale_ang;

  if (std::abs(v_out_) > 0.001 || std::abs(omega_out_) > 0.001) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[Teleop] Dang phat lenh chay: v_linear=%.2f m/s, w_angular=%.2f rad/s",
      v_out_, omega_out_);
  }
}

void GamepadTeleopNode::timer_callback()
{
  bool should_publish = false;
  if (estop_active_) {
    v_out_ = 0.0;
    omega_out_ = 0.0;
  }

  const bool is_active = (enable_deadman_ ? deadman_pressed_ : true) &&
                         (std::abs(v_out_) > 0.001 || std::abs(omega_out_) > 0.001);

  if (is_active) {
    stop_sent_count_ = 0;
    should_publish = true;
  } else {
    // Gui 3 lan goi tin van toc 0 de dung han, sau do ngung publish
    // de twist_mux tu dong nha kenh joystick (timeout 0.25s) cho Nav2/cmd_vel
    if (stop_sent_count_ < 3) {
      v_out_ = 0.0;
      omega_out_ = 0.0;
      should_publish = true;
      stop_sent_count_++;
    }
  }

  if (should_publish) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_out_;
    cmd.angular.z = omega_out_;
    cmd_joy_pub_->publish(cmd);
  }
}

void GamepadTeleopNode::call_reset_odom_service()
{
  if (reset_odom_client_->service_is_ready()) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = reset_odom_client_->async_send_request(
      req,
      std::bind(
        &GamepadTeleopNode::service_response_callback, this,
        std::placeholders::_1));
    (void)future;
  } else {
    RCLCPP_WARN(this->get_logger(), "Service /reset_odom chua san sang!");
  }

  geometry_msgs::msg::PoseWithCovarianceStamped reset_pose;
  reset_pose.header.stamp = this->now();
  reset_pose.header.frame_id = "odom";
  reset_pose.pose.pose.orientation.w = 1.0;
  reset_pose.pose.covariance = {
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
  set_pose_pub_->publish(reset_pose);
  RCLCPP_INFO(this->get_logger(), "Da phat lenh reset /set_pose toi bo loc EKF.");
}

void GamepadTeleopNode::service_response_callback(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  try {
    auto res = future.get();
    if (res->success) {
      RCLCPP_INFO(this->get_logger(), "Reset Odometry thanh cong: %s", res->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Reset Odometry that bai: %s", res->message.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Loi goi Service: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadTeleopNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Runtime error: %s", e.what());
  }
  node->publish_stop();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
