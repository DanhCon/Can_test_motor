# SỔ TAY KỸ THUẬT: CHẨN ĐOÁN & KHẮC PHỤC SỰ CỐ ROBOT AMR
**Hệ thống:** Jetson TX2 (Ubuntu 22.04, ROS 2 Humble) + STM32 W5500 + 2x ZLAC8015D + OLE LiDAR + BNO055

---

## 1. SƠ ĐỒ ĐỊA CHỈ IP & CỔNG MẠNG CHUẨN
| Thiết bị | Địa chỉ IP | Giao thức / Port | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Jetson TX2 (PC)** | `192.168.1.10` | - | Cổng mạng `eth0` |
| **STM32 + W5500** | `192.168.1.100` | UDP `8888` | Điều khiển 2 động cơ ZLAC8015D qua CANopen |
| **OLE LiDAR 2D** | `192.168.1.101` | UDP `2368` | Cảm biến Laser 2D quét môi trường (15 Hz) |

---

## 2. TỔNG HỢP CÁC LỖI THỰC TẾ & CÁCH KHẮC PHỤC

### 🚨 Lỗi 1: Cổng UDP 8888 bị chiếm dụng (`Address already in use`)
* **Hiện tượng:** Khi chạy script hoặc node phần cứng, báo lỗi:
  ```text
  OSError: [Errno 98] Address already in use
  ```
* **Nguyên nhân:** Khi nhấn `Ctrl + C` tắt robot, tiến trình `ros2_control_node` hoặc script cũ chưa giải phóng socket UDP `8888`.
* **Cách khắc phục:** Chạy lệnh Python đọc trực tiếp nhân Linux để tìm đúng PID và buộc đóng cổng:
  ```bash
  python3 -c "
  import os
  port_hex = f'{8888:04X}'
  inodes = set()
  with open('/proc/net/udp') as f:
      for line in f.readlines()[1:]:
          parts = line.strip().split()
          if parts[1].endswith(':' + port_hex):
              inodes.add(parts[9])
  for pid in [p for p in os.listdir('/proc') if p.isdigit()]:
      try:
          for fd in os.listdir(f'/proc/{pid}/fd'):
              target = os.readlink(f'/proc/{pid}/fd/{fd}')
              for inode in inodes:
                  if f'[{inode}]' in target:
                      os.system(f'kill -9 {pid}')
                      print(f'Da giai phong PID {pid} dang giu port 8888')
      except Exception:
          pass
  "
  ```

---

### 🚨 Lỗi 2: Xung đột kiểu dữ liệu giữa `twist_mux` và `diff_drive_controller` trên ROS 2 Humble
* **Hiện tượng:**
  ```text
  Cannot echo topic '/diff_drive_controller/cmd_vel', as it contains more than one type: [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]
  ```
* **Nguyên nhân:**
  * Trên ROS 2 Humble, `twist_mux` chỉ phát ra `geometry_msgs/msg/Twist` (Unstamped).
  * `diff_drive_controller` mặc định lại dùng `TwistStamped`.
* **Cách khắc phục:**
  1. Trong `config/diff_drive_controller.yaml`, cấu hình:
     ```yaml
     diff_drive_controller:
       ros__parameters:
         use_stamped_vel: false
     ```
  2. Khi `use_stamped_vel: false`, controller sẽ lắng nghe topic `/diff_drive_controller/cmd_vel_unstamped`.
  3. Trong `launch/robot.launch.py`, remap đầu ra của `twist_mux`:
     ```python
     remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
     ```

---

### 🚨 Lỗi 3: Robot không lăn bánh khi để dưới sàn (Ma sát tĩnh)
* **Hiện tượng:** STM32 log nhận `cmd_v = 0.30 m/s`, xung encoder có nhích nhẹ nhưng bánh xe không quay dưới đất. Khi kê bổng bánh xe lên thì quay tít (18 RPM).
* **Nguyên nhân:** Khung robot nặng, khi vận tốc đặt thấp (`0.1 - 0.3 m/s`), tốc độ động cơ tương ứng chỉ khoảng 15 - 18 RPM $\to$ lực kéo ban đầu không đủ thắng ma sát tĩnh sàn nhà.
* **Quy tắc kiểm tra:**
  * **Luôn kê bổng 2 bánh chủ động** khi kiểm tra phần mềm/firmware.
  * Khi đặt xuống sàn, tăng dần vận tốc hoặc cấu hình hệ số gia tốc phù hợp.

---

### 🚨 Lỗi 4: Xung đột IP giữa LiDAR và STM32 Gateway
* **Hiện tượng:** LiDAR bật lên nhưng không có topic `/scan` (`Publisher count: 0`), hoặc lúc nhận lúc mất.
* **Nguyên nhân:**
  * STM32 dùng IP `192.168.1.100`.
  * File cấu hình gốc `ole2dv2.yaml` bị điền nhầm `lidar_ip: 192.168.1.100` $\to$ Driver kết nối nhầm vào STM32 thay vì mắt quét.
* **Cách kiểm tra & khắc phục:**
  1. Dùng lệnh `arp -a` để phát hiện MAC OUI:
     * `00:08:dc:...` (WIZnet) = `192.168.1.100` (STM32)
     * MAC còn lại = `192.168.1.101` (LiDAR OLE)
  2. Sửa file cấu hình về đúng `192.168.1.101`:
     ```bash
     sed -i 's/lidar_ip: 192.168.1.100/lidar_ip: 192.168.1.101/g' /home/nhatbot_ws/src/nhatbot_drivers/oleros2/src/ros2_lidar/params/ole2dv2.yaml
     sed -i 's/lidar_ip: 192.168.1.100/lidar_ip: 192.168.1.101/g' /home/nhatbot_ws/install/ros2_lidar/share/ros2_lidar/params/ole2dv2.yaml
     ```

---

### 🚨 Lỗi 5: Tiến trình LiDAR cũ chạy ngầm chiếm cổng UDP 2368
* **Hiện tượng:** Chạy launch mới nhưng không nhận được tia quét dù IP đã sửa đúng.
* **Nguyên nhân:** Tiến trình cũ từ các lần chạy trước vẫn còn sống ngầm và giữ port UDP `2368`.
* **Cách khắc phục:**
  ```bash
  kill -9 $(ps -ef | grep -E 'ros2_lidar|laser_filters|lidar_driver' | grep -v grep | awk '{print $2}') 2>/dev/null
  ```

---

### 🚨 Lỗi 6: LiDAR dừng ở `out1.. out2..` khi chạy `ros2 run`
* **Hiện tượng:** Chạy `ros2 run ros2_lidar lidar_driver` thì in ra `out1.. out2..` rồi đứng yên.
* **Nguyên nhân:** `lidar_driver` là **ROS 2 LifecycleNode**. Nó kết thúc Constructor ở `out2..` và đứng chờ sự kiện kích hoạt (`configure` $\to$ `activate`).
* **Cách khắc phục:** Phải khởi động thông qua file launch chuyên dụng:
  ```bash
  ros2 launch ros2_lidar ole2dv2_launch.py
  ```
  *(File launch này chứa bộ phát sự kiện `EmitEvent` tự động kích hoạt node).*

---

### 🚨 Lỗi 7: Kẹt bộ nhớ chia sẻ FastRTPS (`RTPS_TRANSPORT_SHM Error`)
* **Hiện tượng:** Xuất hiện log đỏ khi chạy node IMU BNO055 hoặc các node khác:
  ```text
  [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX: open_and_lock_file failed
  ```
* **Nguyên nhân:** FastDDS để lại các file lock rác trong thư mục chia sẻ bộ nhớ khi bị tắt đột ngột.
* **Cách khắc phục:** Dọn sạch thư mục `/dev/shm` trước khi launch:
  ```bash
  rm -rf /dev/shm/fastrtps*
  ```

---

## 3. BẢNG LỆNH "CỨU HỘ & KHỞI ĐỘNG NHANH" (QUICK CHEATSHEET)

Mỗi khi hệ thống bị treo, đơ cổng hoặc chuẩn bị khởi động lại, chạy cụm lệnh 1 dòng này:

```bash
# 1. Dọn dẹp tiến trình treo & file rác bộ nhớ
rm -rf /dev/shm/fastrtps*
kill -9 $(ps -ef | grep -E 'ros2_control_node|zlac|lidar|laser_filters|twist_mux' | grep -v grep | awk '{print $2}') 2>/dev/null

# 2. LỰA CHỌN KHỞI CHẠY:
# Cách A: Khởi động TOÀN BỘ TỰ HÀNH (Hardware + AMCL + Nav2) trong 1 lệnh duy nhất:
source /home/nhatbot_ws/install/setup.bash
ros2 launch can_test_motor bringup_all.launch.py enable_deadman:=false

# Cách B: Chỉ khởi động phần cứng để test tay cầm / cảm biến (không nạp bản đồ):
source /home/nhatbot_ws/install/setup.bash
ros2 launch can_test_motor robot.launch.py enable_deadman:=false
```

Kiểm tra sức khỏe hệ thống ở Terminal 2:
```bash
# Kiểm tra tần số quét LiDAR (đạt chuẩn ~15 Hz từ laser_filters)
ros2 topic hz /scan

# Kiểm tra tần số Odometry đã dung hợp EKF (đạt chuẩn ~20 Hz mượt mà, nhẹ tải CPU Jetson TX2)
ros2 topic hz /odometry/filtered

# Kiểm tra phản hồi vận tốc động cơ
ros2 topic echo /diff_drive_controller/odom
```
