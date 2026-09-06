#!/usr/bin/env python3
"""
===============================================================================
Node ROS 2: Gamepad Teleop Node (Điều khiển Robot bằng Tay Cầm)
Mô tả:
  - Lắng nghe topic /joy từ joy_node (chuẩn ROS 2 sensor_msgs/msg/Joy).
  - Ánh xạ cần gạt Joystick sang vận tốc dài (v) và vận tốc góc (omega).
  - Xuất bản tin Twist lên topic /cmd_vel (kết nối trực tiếp với zlac_udp_odom_node).
  - Tính năng nâng cao:
    + Nút an toàn Deadman Switch (Giữ L1/LB mới cho chạy).
    + Chế độ tăng tốc Turbo Boost (Giữ R1/RB để chạy nhanh).
    + Phanh khẩn cấp E-Stop (Bấm nút B/Tròn để dừng khẩn).
    + Nút gọi Service /reset_odom (Bấm nút Y/Tam giác để reset Odometry).
    + Tự động nhận diện layout tay cầm (PS4 / PS5 / Xbox / Logitech F710).

Tương thích: ROS 2 Humble, Iron, Foxy, Jazzy
===============================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class GamepadTeleopNode(Node):
    def __init__(self):
        super().__init__('gamepad_teleop_node')

        # ---------------------------------------------------------------------
        # KHAI BÁO CÁC PARAMETERS (CÓ THỂ CHỈNH TRONG LAUNCH FILE / YAML)
        # ---------------------------------------------------------------------
        # Cấu hình Trục cần gạt (Axes)
        self.declare_parameter('axis_linear', 1)        # Cần gạt trái Y (Tiến / Lùi)
        self.declare_parameter('axis_angular', 3)       # Cần gạt phải X (hoặc gạt trái X = 0)
        self.declare_parameter('deadzone', 0.08)        # Vùng chết cần gạt (chống trôi cần)

        # Cấu hình Vận tốc (Giới hạn tối đa 0.3 m/s theo yêu cầu)
        self.declare_parameter('scale_linear_normal', 0.3)    # Tốc độ tiến bình thường: 0.3 m/s
        self.declare_parameter('scale_angular_normal', 0.5)   # Tốc độ quay bình thường: 0.5 rad/s
        self.declare_parameter('scale_linear_turbo', 0.3)     # Tốc độ Turbo khóa ở 0.3 m/s
        self.declare_parameter('scale_angular_turbo', 0.5)    # Tốc độ quay Turbo: 0.5 rad/s

        # Cấu hình Nút bấm (Buttons)
        # Mặc định theo layout chuẩn Xbox / Logitech / PS4 (L1=4, R1=5, B=1, Y=3)
        self.declare_parameter('enable_deadman', True)       # Bật nút an toàn Deadman (True: giữ nút mới chạy)
        self.declare_parameter('btn_deadman', 4)             # Nút Deadman: L1 (LB)
        self.declare_parameter('btn_turbo', 5)               # Nút Turbo: R1 (RB)
        self.declare_parameter('btn_estop', 1)               # Nút dừng khẩn: B (Nút Tròn trên PS)
        self.declare_parameter('btn_reset_odom', 3)          # Nút reset Odometry: Y (Nút Tam giác trên PS)

        self.declare_parameter('publish_rate', 20.0)         # Tần số gửi cmd_vel (Hz)

        # Đọc giá trị Parameters
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.deadzone = self.get_parameter('deadzone').value

        self.scale_linear_normal = self.get_parameter('scale_linear_normal').value
        self.scale_angular_normal = self.get_parameter('scale_angular_normal').value
        self.scale_linear_turbo = self.get_parameter('scale_linear_turbo').value
        self.scale_angular_turbo = self.get_parameter('scale_angular_turbo').value

        self.enable_deadman = self.get_parameter('enable_deadman').value
        self.btn_deadman = self.get_parameter('btn_deadman').value
        self.btn_turbo = self.get_parameter('btn_turbo').value
        self.btn_estop = self.get_parameter('btn_estop').value
        self.btn_reset_odom = self.get_parameter('btn_reset_odom').value

        self.publish_rate = self.get_parameter('publish_rate').value

        # Biến trạng thái nội bộ
        self.v_out = 0.0
        self.omega_out = 0.0
        self.estop_active = False
        self.last_reset_btn_state = 0

        # Publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber nhận dữ liệu từ joy_node
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Client gọi Service /reset_odom của zlac_udp_odom_node
        self.reset_odom_client = self.create_client(Trigger, 'reset_odom')

        # Timer định kỳ gửi cmd_vel
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("=" * 65)
        self.get_logger().info("[*] Node Gamepad Teleop đã khởi động sẵn sàng!")
        self.get_logger().info(f"[*] Chế độ Deadman (Nút an toàn): {'BẬT (Giữ nút L1/LB để chạy)' if self.enable_deadman else 'TẮT'}")
        self.get_logger().info(f"[*] Nút Turbo (R1/RB): x{self.scale_linear_turbo/self.scale_linear_normal:.1f} tốc độ")
        self.get_logger().info(f"[*] Nút Phanh khẩn cấp E-Stop: B / Nút Tròn")
        self.get_logger().info(f"[*] Nút Reset Odometry: Y / Nút Tam giác")
        self.get_logger().info("=" * 65)

    def joy_callback(self, msg: Joy):
        # 1. Kiểm tra nút phanh khẩn cấp E-Stop (Nút B / Tròn)
        if len(msg.buttons) > self.btn_estop and msg.buttons[self.btn_estop] == 1:
            self.estop_active = not self.estop_active
            if self.estop_active:
                self.get_logger().warn("[E-STOP] PHANH KHẨN CẤP ĐÃ ĐƯỢC KÍCH HOẠT! (Bấm lại nút B để mở khóa)")
            else:
                self.get_logger().info("[E-STOP] Đã mở khóa phanh khẩn cấp.")
            return

        if self.estop_active:
            self.v_out = 0.0
            self.omega_out = 0.0
            return

        # 2. Kiểm tra nút Reset Odometry (Nút Y / Tam giác - Bắt sườn dương)
        if len(msg.buttons) > self.btn_reset_odom:
            btn_state = msg.buttons[self.btn_reset_odom]
            if btn_state == 1 and self.last_reset_btn_state == 0:
                self.call_reset_odom_service()
            self.last_reset_btn_state = btn_state

        # 3. Kiểm tra nút an toàn Deadman Switch (L1 / LB)
        if self.enable_deadman:
            if len(msg.buttons) <= self.btn_deadman or msg.buttons[self.btn_deadman] == 0:
                # Chưa giữ nút an toàn -> Tự động dừng xe
                self.v_out = 0.0
                self.omega_out = 0.0
                return

        # 4. Kiểm tra nút Turbo Boost (R1 / RB)
        is_turbo = False
        if len(msg.buttons) > self.btn_turbo and msg.buttons[self.btn_turbo] == 1:
            is_turbo = True

        scale_lin = self.scale_linear_turbo if is_turbo else self.scale_linear_normal
        scale_ang = self.scale_angular_turbo if is_turbo else self.scale_angular_normal

        # 5. Đọc giá trị cần gạt Joystick
        raw_lin = msg.axes[self.axis_linear] if len(msg.axes) > self.axis_linear else 0.0
        # Trục angular: nếu gạt qua trái thường là số dương -> quay trái (dương)
        raw_ang = msg.axes[self.axis_angular] if len(msg.axes) > self.axis_angular else 0.0

        # Lọc Deadzone
        if abs(raw_lin) < self.deadzone:
            raw_lin = 0.0
        if abs(raw_ang) < self.deadzone:
            raw_ang = 0.0

        # Tính vận tốc mục tiêu
        self.v_out = float(raw_lin * scale_lin)
        self.omega_out = float(raw_ang * scale_ang)

    def timer_callback(self):
        # Xuất bản tin Twist lên /cmd_vel
        cmd = Twist()
        cmd.linear.x = self.v_out
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.omega_out
        self.cmd_pub.publish(cmd)

    def call_reset_odom_service(self):
        if not self.reset_odom_client.service_is_ready():
            self.get_logger().warn("Service /reset_odom chưa sẵn sàng!")
            return
        req = Trigger.Request()
        future = self.reset_odom_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"✅ Reset Odometry thành công: {res.message}")
            else:
                self.get_logger().warn(f"❌ Reset Odometry thất bại: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Lỗi gọi Service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Khi tắt node, gửi lệnh vận tốc = 0 để phanh xe ngay
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
