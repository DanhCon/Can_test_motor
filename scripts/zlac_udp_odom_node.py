#!/usr/bin/env python3
"""
===============================================================================
Node ROS 2: ZLAC8015D UDP CANopen Odometry Node
Mô tả:
  - Giao tiếp với STM32F103 + W5500 qua giao thức Ethernet UDP nhị phân (Binary).
  - Tần số truyền nhận: 50 Hz (20ms/chu kỳ).
  - Điều khiển: Nhận /cmd_vel (Twist), làm mượt (Velocity Smoother), gửi gói UDP (12 bytes, CRC16).
  - Phản hồi: Nhận Telemetry 22 bytes từ STM32, tính toán Odometry (Runge-Kutta 2nd order),
              publish /odom (nav_msgs/Odometry) và broadcast TF (odom -> base_link).
  - Hỗ trợ thêm: Publish điện áp DC Bus (/battery_voltage), dòng điện 2 bánh, Service /reset_odom.

Tương thích: ROS 2 Humble, Iron, Foxy, Jazzy
===============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import tf2_ros

import socket
import struct
import math
import time
import threading
import numpy as np


# =============================================================================
# HÀM TÍNH TOÁN TOÀN VẸN DỮ LIỆU CRC-16/MODBUS (ĐỒNG BỘ 100% VỚI CODE C STM32)
# =============================================================================
def calculate_crc16(data: bytes) -> int:
    """Tính toán CRC-16 Modbus (Đa thức 0xA001, Giá trị khởi tạo 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class ZLAC8015DUDPOdomNode(Node):
    def __init__(self):
        super().__init__('zlac_udp_odom_node')

        # ---------------------------------------------------------------------
        # KHAI BÁO CÁC PARAMETERS (HỖ TRỢ TÙY BIẾN QUA FILE YAML / LAUNCH FILE)
        # ---------------------------------------------------------------------
        # Thông số mạng UDP
        self.declare_parameter('stm32_ip', '192.168.1.100')
        self.declare_parameter('stm32_port', 8888)
        self.declare_parameter('local_port', 8888)

        # Thông số cơ khí robot (Cập nhật chuẩn theo xe thực tế)
        self.declare_parameter('wheel_radius', 0.0535)    # Bán kính bánh xe: 0.0535m (53.5mm)
        self.declare_parameter('wheel_base', 0.45)        # Khoảng cách 2 bánh: 0.45m (450mm)
        self.declare_parameter('cpr', 4096)               # Độ phân giải Encoder: 4096 xung/vòng
        self.declare_parameter('motor_b_reverse', True)   # Đảo dấu motor B nếu lắp đối xứng

        # Khung tọa độ và tần số
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('control_rate', 50.0)      # Chu kỳ điều khiển gửi UDP: 50 Hz
        self.declare_parameter('cmd_vel_timeout', 0.25)   # Timeout ngắt lệnh an toàn: 250ms

        # Giới hạn vận tốc
        self.declare_parameter('max_linear_velocity', 1.5)   # m/s
        self.declare_parameter('max_angular_velocity', 1.5)  # rad/s

        # Bộ lọc làm mượt gia tốc (Velocity Smoother)
        self.declare_parameter('linear_accel', 0.8)   # m/s^2 (gia tốc tăng/giảm tốc dài)
        self.declare_parameter('angular_accel', 1.2)  # rad/s^2 (gia tốc quay)
        self.declare_parameter('min_breakaway_velocity', 0.04)  # m/s (~7.1 RPM: Vận tốc bứt phá ma sát tĩnh)

        # Đọc giá trị Parameters
        self.stm32_ip = self.get_parameter('stm32_ip').value
        self.stm32_port = self.get_parameter('stm32_port').value
        self.local_port = self.get_parameter('local_port').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.cpr = self.get_parameter('cpr').value
        self.motor_b_reverse = self.get_parameter('motor_b_reverse').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.control_rate = self.get_parameter('control_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.linear_accel = self.get_parameter('linear_accel').value
        self.angular_accel = self.get_parameter('angular_accel').value
        self.min_breakaway_vel = self.get_parameter('min_breakaway_velocity').value

        # ---------------------------------------------------------------------
        # BIẾN NỘI BỘ VẬN HÀNH & ODOMETRY
        # ---------------------------------------------------------------------
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Tọa độ Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_x = 0.0
        self.w_z = 0.0

        # Mốc xung Encoder
        self.last_pos_a = None
        self.last_pos_b = None
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()

        # Giám sát Watchdog Ethernet (mất tín hiệu UDP từ STM32)
        self.last_udp_rx_time = self.get_clock().now()
        self.ethernet_connected = True
        self.current_a = 0.0
        self.current_b = 0.0

        # Khóa an toàn đa luồng (Thread-safe)
        self.odom_lock = threading.Lock()
        self.is_running = True

        # ---------------------------------------------------------------------
        # KHỞI TẠO SOCKET UDP GIAO TIẾP VỚI W5500
        # ---------------------------------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(('0.0.0.0', self.local_port))
            self.sock.settimeout(0.05)  # Timeout 50ms cho mỗi lần đọc
            self.get_logger().info(f"Đã mở Socket UDP lắng nghe tại port {self.local_port}, gửi tới STM32 {self.stm32_ip}:{self.stm32_port}")
        except Exception as e:
            self.get_logger().error(f"Không thể bind Socket UDP tại port {self.local_port}: {e}")

        # ---------------------------------------------------------------------
        # ROS 2 PUBLISHERS, SUBSCRIBERS, TF, SERVICES
        # ---------------------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber nhận lệnh vận tốc từ Navigation / Teleop
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # Service reset Odometry về 0
        self.reset_odom_srv = self.create_service(
            Trigger, 'reset_odom', self.reset_odom_callback
        )

        # Timer gửi lệnh điều khiển UDP xuống STM32 (50 Hz)
        timer_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)

        # Luồng ngầm (Background Thread) liên tục nhận gói Telemetry 22 bytes từ STM32
        self.rx_thread = threading.Thread(target=self.udp_receive_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"[*] Node ZLAC8015D UDP Odom khởi động thành công (Bán kính: {self.wheel_radius*1000:.1f}mm, Khoảng cách bánh: {self.wheel_base*1000:.1f}mm).")

    # =========================================================================
    # CALLBACK NHẬN LỆNH /cmd_vel TỪ ROS 2
    # =========================================================================
    def cmd_vel_callback(self, msg: Twist):
        if not self.is_running:
            return
        self.last_cmd_vel_time = self.get_clock().now()

        lin_x = msg.linear.x
        ang_z = msg.angular.z

        # Lọc vùng chết nhỏ (Deadzone 0.005)
        if abs(lin_x) < 0.005:
            lin_x = 0.0
        if abs(ang_z) < 0.005:
            ang_z = 0.0

        # Giới hạn dải vận tốc cho phép
        self.target_linear = float(np.clip(lin_x, -self.max_linear_velocity, self.max_linear_velocity))
        self.target_angular = float(np.clip(ang_z, -self.max_angular_velocity, self.max_angular_velocity))

    # =========================================================================
    # SERVICE RESET ODOMETRY VỀ GỐC (0, 0, 0)
    # =========================================================================
    def reset_odom_callback(self, req, res):
        self.get_logger().info("Nhận yêu cầu reset Odometry về 0...")
        with self.odom_lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.v_x = 0.0
            self.w_z = 0.0
            self.last_pos_a = None
            self.last_pos_b = None
        res.success = True
        res.message = "Đã reset tọa độ Odometry về 0 thành công."
        return res

    # =========================================================================
    # TIMER ĐIỀU KHIỂN ĐỊNH KỲ 50 HZ: LÀM MƯỢT VẬN TỐC & BẮN GÓI UDP
    # =========================================================================
    def control_timer_callback(self):
        if not self.is_running:
            return

        now = self.get_clock().now()
        dt = 1.0 / self.control_rate

        # 1. Watchdog an toàn: Nếu mất tín hiệu cmd_vel quá timeout -> tự giảm tốc về 0
        time_since_last_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if time_since_last_cmd > self.cmd_vel_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # 2. Watchdog giám sát mất kết nối Ethernet tới STM32 (quá 1.0s không nhận được Telemetry)
        time_since_last_rx = (now - self.last_udp_rx_time).nanoseconds / 1e9
        if time_since_last_rx > 1.0:
            if self.ethernet_connected:
                self.get_logger().error(
                    f"CẢNH BÁO: MẤT KẾT NỐI ETHERNET TỚI STM32! (Không nhận được phản hồi UDP trong {time_since_last_rx:.1f}s)",
                    throttle_duration_sec=2.0
                )
                self.ethernet_connected = False
            else:
                self.get_logger().error(
                    f"CẢNH BÁO: MẤT KẾT NỐI ETHERNET TỚI STM32! (Không nhận được phản hồi UDP trong {time_since_last_rx:.1f}s)",
                    throttle_duration_sec=2.0
                )

        # 3. Bộ lọc gia tốc làm mượt (Velocity Smoother)
        step_lin = self.linear_accel * dt
        step_ang = self.angular_accel * dt

        def move_towards(curr, target, max_step, min_kick=0.0):
            # Bứt phá ma sát tĩnh: Chỉ kích hoạt nếu có đặt min_kick > 0 và bắt đầu từ 0
            if min_kick > 0.0 and abs(curr) < 0.001 and abs(target) > 0.001:
                if target > 0:
                    return min(min_kick, target)
                else:
                    return max(-min_kick, target)
            elif curr < target:
                return min(curr + max_step, target)
            elif curr > target:
                return max(curr - max_step, target)
            return curr

        self.current_linear = move_towards(self.current_linear, self.target_linear, step_lin, self.min_breakaway_vel)
        self.current_angular = move_towards(self.current_angular, self.target_angular, step_ang, 0.0)

        # 4. Đóng gói 12 bytes nhị phân gửi xuống STM32:
        # Format: Header [0xAA, 0x55] (2B) + float v (4B) + float omega (4B) + uint16 CRC (2B)
        payload = struct.pack('<BBff', 0xAA, 0x55, float(self.current_linear), float(self.current_angular))
        crc = calculate_crc16(payload)
        packet = payload + struct.pack('<H', crc)

        # 5. Gửi UDP tới STM32 Gateway
        try:
            self.sock.sendto(packet, (self.stm32_ip, self.stm32_port))
        except Exception as e:
            self.get_logger().error(f"Lỗi khi gửi gói UDP tới STM32: {e}", throttle_duration_sec=2.0)

    # =========================================================================
    # LUỒNG NGẦM NHẬN TELEMETRY 22 BYTES TỪ STM32 & CẬP NHẬT ODOMETRY
    # =========================================================================
    def udp_receive_loop(self):
        """Hứng gói tin phản hồi 22 bytes từ STM32 và tính toán Odometry liên tục"""
        meter_per_count = (2.0 * math.pi * self.wheel_radius) / float(self.cpr)

        while self.is_running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) < 22:
                    continue

                # Cấu trúc 22 bytes:
                # Header (0x55 0xAA) + vel_a(h) + vel_b(h) + pos_a(l) + pos_b(l) + error(H) + cur_a(h) + cur_b(h) + v_bus(H)
                h1, h2, vel_a_raw, vel_b_raw, pos_a, pos_b, err_code, cur_a_raw, cur_b_raw, v_bus_raw = struct.unpack(
                    '<BBhhllHhhH', data[:22]
                )

                if h1 != 0x55 or h2 != 0xAA:
                    continue

                now = self.get_clock().now()

                # Cập nhật thời gian nhận gói UDP thành công & thông báo phục hồi nếu vừa mất mạng
                if not self.ethernet_connected:
                    self.get_logger().info("ĐÃ KHÔI PHỤC KẾT NỐI ETHERNET TỚI STM32.")
                    self.ethernet_connected = True
                self.last_udp_rx_time = now

                # Cập nhật giá trị dòng điện thực tế nội bộ (đơn vị: Ampe)
                self.current_a = cur_a_raw / 10.0
                self.current_b = cur_b_raw / 10.0

                # Cảnh báo lỗi driver nếu có
                if err_code != 0:
                    if err_code == 0xEEEE:
                        self.get_logger().error("CẢNH BÁO: KÍCH HOẠT BẢO VỆ KẸT TẢI / QUÁ DÒNG TRÊN DRIVER (0xEEEE)!", throttle_duration_sec=2.0)
                    elif err_code == 0xEE01:
                        self.get_logger().warn("CẢNH BÁO: MẤT KẾT NỐI CAN GIỮA STM32 VÀ ZLAC (0xEE01)!", throttle_duration_sec=2.0)
                    else:
                        self.get_logger().error(f"CẢNH BÁO LỖI PHẦN CỨNG DRIVER: 0x{err_code:04X}", throttle_duration_sec=2.0)

                # Publish điện áp DC Bus (Pin)
                v_bus = v_bus_raw / 10.0 if v_bus_raw < 1000 else v_bus_raw / 100.0
                batt_msg = Float32()
                batt_msg.data = float(v_bus)
                self.battery_pub.publish(batt_msg)

                # -------------------------------------------------------------
                # TÍNH TOÁN ODOMETRY TỪ XUNG ENCODER (CHÍNH XÁC CAO)
                # -------------------------------------------------------------
                with self.odom_lock:
                    if self.last_pos_a is None:
                        self.last_pos_a = pos_a
                        self.last_pos_b = pos_b
                        self.last_odom_time = now
                        continue

                    dt = (now - self.last_odom_time).nanoseconds / 1e9
                    if dt <= 0.0001:
                        continue
                    self.last_odom_time = now

                    # Tính độ chênh lệch xung
                    delta_a = pos_a - self.last_pos_a
                    delta_b = pos_b - self.last_pos_b
                    self.last_pos_a = pos_a
                    self.last_pos_b = pos_b

                    # Đảo chiều motor B nếu lắp đối xứng ngược chiều
                    if self.motor_b_reverse:
                        delta_b = -delta_b

                    # Quãng đường lăn bánh trái và bánh phải (m)
                    dist_L = delta_a * meter_per_count
                    dist_R = delta_b * meter_per_count

                    # Động học vi sai (Differential Drive Forward Kinematics)
                    dist_center = (dist_L + dist_R) / 2.0
                    d_theta = (dist_R - dist_L) / self.wheel_base

                    # Tích phân điểm giữa Runge-Kutta bậc 2 (Midpoint integration)
                    theta_mid = self.theta + (d_theta / 2.0)
                    self.x += dist_center * math.cos(theta_mid)
                    self.y += dist_center * math.sin(theta_mid)
                    self.theta += d_theta

                    # Chuẩn hóa góc quay về đoạn [-PI, PI]
                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

                    # Vận tốc thực tế tức thời
                    self.v_x = dist_center / dt
                    self.w_z = d_theta / dt

                    cur_x = self.x
                    cur_y = self.y
                    cur_theta = self.theta
                    cur_vx = self.v_x
                    cur_wz = self.w_z

                # -------------------------------------------------------------
                # XUẤT BẢN TIN ODOMETRY VÀ BROADCAST TF
                # -------------------------------------------------------------
                # Tính Quaternion từ góc Euler Yaw (theta)
                cy = math.cos(cur_theta * 0.5)
                sy = math.sin(cur_theta * 0.5)
                qx, qy, qz, qw = 0.0, 0.0, sy, cy

                stamp = now.to_msg()

                # Gửi Odometry Message
                odom_msg = Odometry()
                odom_msg.header.stamp = stamp
                odom_msg.header.frame_id = self.odom_frame
                odom_msg.child_frame_id = self.base_frame

                odom_msg.pose.pose.position.x = cur_x
                odom_msg.pose.pose.position.y = cur_y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw

                # Ma trận hiệp phương sai sai số vị trí (Pose Covariance)
                odom_msg.pose.covariance = [
                    1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  1e-3
                ]

                odom_msg.twist.twist.linear.x = cur_vx
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.angular.z = cur_wz

                # Ma trận hiệp phương sai sai số vận tốc (Twist Covariance)
                odom_msg.twist.covariance = [
                    1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  1e-3
                ]

                self.odom_pub.publish(odom_msg)

                # Broadcast TF (odom -> base_link)
                if self.publish_tf:
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id = self.odom_frame
                    tf.child_frame_id = self.base_frame

                    tf.transform.translation.x = cur_x
                    tf.transform.translation.y = cur_y
                    tf.transform.translation.z = 0.0
                    tf.transform.rotation.x = qx
                    tf.transform.rotation.y = qy
                    tf.transform.rotation.z = qz
                    tf.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(tf)

            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"Lỗi luồng đọc UDP: {e}", throttle_duration_sec=2.0)

    # =========================================================================
    # DỪNG AN TOÀN KHI TẮT NODE
    # =========================================================================
    def stop(self):
        self.is_running = False
        self.get_logger().info("Đang dừng Node, gửi lệnh phanh xe khẩn cấp tới STM32...")

        # Gửi liên tiếp 3 gói lệnh dừng v=0, omega=0 để đảm bảo STM32 nhận được
        for _ in range(3):
            payload = struct.pack('<BBff', 0xAA, 0x55, 0.0, 0.0)
            crc = calculate_crc16(payload)
            packet = payload + struct.pack('<H', crc)
            try:
                self.sock.sendto(packet, (self.stm32_ip, self.stm32_port))
                time.sleep(0.02)
            except Exception:
                pass

        if hasattr(self, 'sock'):
            self.sock.close()
        self.get_logger().info("Đã đóng Socket và tắt an toàn.")


def main(args=None):
    rclpy.init(args=args)
    node = ZLAC8015DUDPOdomNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
