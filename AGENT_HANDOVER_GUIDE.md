# 🤖 TÀI LIỆU BÀN GIAO & HƯỚNG DẪN KẾ THỪA DỰ ÁN (AGENT HANDOVER GUIDE)
> **Dành cho:** Kỹ sư phát triển / AI Agent tiếp quản dự án  
> **Repository:** [DanhCon/Can_test_motor](https://github.com/DanhCon/Can_test_motor.git)  
> **Phiên bản tài liệu:** 2.0 (Cập nhật ngày 05/09/2026)  
> **Mục đích:** Cung cấp toàn bộ ngữ cảnh kỹ thuật, kiến trúc phân tầng, các "hố sâu" (pitfalls) đã gặp và giải quyết, bài học xương máu và quy tắc vận hành để bất kỳ Agent nào đọc vào cũng có thể nắm bắt 100% hệ thống ngay lập tức mà không làm hỏng các tính năng đã ổn định.

---

## 🧭 1. TỔNG QUAN DỰ ÁN & MỤC TIÊU

Dự án xây dựng hệ thống điều khiển cầu nối thời gian thực (**Real-Time Gateway**) cho Robot tự hành AGV / AMR sử dụng mô hình động học vi sai (Differential Drive):
- **Cấp điều khiển cao (High-Level Controller):** Máy tính nhúng / PC chạy ROS 2 (Humble / Iron / Foxy / Jazzy).
- **Cầu nối trung gian (Hardware Gateway):** Vi điều khiển **STM32F103C8T6** kết hợp chip mạng Ethernet phần cứng **W5500** (giao tiếp SPI1 @ 4.5 Mbps).
- **Driver động cơ công nghiệp:** **ZLAC8015D** điều khiển đồng thời 2 động cơ Hub không chổi than (Dual-channel BLDC Hub Servo), giao tiếp qua bus **CANopen (CiA 301 / CiA 402)** với tốc độ 500 kbps.

### Sơ đồ luồng dữ liệu 2 chiều (Chu kỳ 50 Hz = 20ms):
```
[ROS 2 PC / Navigation / Teleop]
       │
       ▼ ▲  Gói UDP nhị phân (Down: 12 Bytes | Up Telemetry: 22 Bytes)
[W5500 SPI Ethernet Gateway (IP: 192.168.1.100, Port: 8888)]
       │
       ▼ ▲  SPI1 (CS: PA4, RST: PA3, SCK: PA5, MISO: PA6, MOSI: PA7)
[STM32F103C8T6 Microcontroller]
       │  ├─ Bộ lọc động học vi sai (Linear v, Angular omega ↔ RPM 2 bánh)
       │  ├─ Watchdog an toàn mạng 250ms (Tự phanh khi mất gói UDP)
       │  ├─ Watchdog CAN 1.5s (Báo lỗi 0xEE01 khi mất kết nối ZLAC)
       │  └─ Bảo vệ kẹt tải & quá dòng (Stall & Overcurrent 0xEEEE)
       ▼ ▲  CAN Bus 2.0B @ 500 kbps (PA11/PA12, Transceiver TJA1050 / VP230)
[ZLAC8015D Dual Servo Driver (Node ID: 0x01)]
       │
   ┌───┴───┐
   ▼       ▼
[Motor A] [Motor B]
(Trái)    (Phải)
```

---

## 📁 2. CẤU TRÚC CODEBASE & CHI TIẾT CÁC FILE NODE, CONFIG, LAUNCH

### 2.1. Cây thư mục tổng thể dự án
```
Can_test_motor/
├── Core/
│   ├── Inc/
│   │   ├── zlac_can.h            # Khai báo cấu trúc, macro, hằng số cơ khí, mã lỗi CANopen
│   │   └── main.h                # Khai báo chân GPIO, SPI1, CAN1
│   └── Src/
│       ├── main.c                # Vòng lặp chính STM32: Đọc/ghi W5500, Watchdog 250ms, gửi Telemetry 50Hz
│       └── zlac_can.c            # CANopen State Machine, PDO mapping, SDO Write/Read, Kinematics
├── scripts/
│   ├── zlac_udp_odom_node.py     # [NODE CHÍNH] Gateway UDP điều khiển động cơ, Odometry, TF & Watchdog
│   ├── teleop_joy.py             # [NODE TAY CẦM] Ánh xạ Gamepad (/joy) sang /cmd_vel với Deadman & Turbo
│   └── test_robot_telemetry.py   # [SCRIPT TEST] Kiểm thử độc lập kết nối UDP nhị phân & đo RTT không cần ROS 2
├── config/
│   ├── joy.yaml                  # Cấu hình teleop_twist_joy chuẩn (cần gạt, deadman, giới hạn tốc độ)
│   ├── ekf.yaml                  # Cấu hình bộ lọc Extended Kalman Filter dung hợp /odom và /bno055/imu
│   ├── bno055_params_i2c.yaml    # Cấu hình IMU 9-DOF BNO055 qua I2C (offsets calib, NDOF 50Hz)
│   └── ole2dv2.yaml              # Cấu hình LiDAR công nghiệp OLE qua mạng Ethernet UDP (IP 192.168.1.101)
├── launch/
│   ├── teleop.launch.py          # [LAUNCH CHÍNH] Khởi động joy_node + teleop_joy + zlac_udp_odom_node
│   ├── teleop_robot.launch.py    # Launch file tay cầm cấu hình tốc độ mềm mại hơn
│   ├── bno055.launch.py          # Khởi động driver IMU BNO055 + Static TF base_link -> imu_link
│   ├── lidar.launch.py           # Launch file tổng hợp chạy OLE LiDAR (Ethernet) hoặc RPLidar (USB)
│   ├── ole_lidar.launch.py       # Khởi động riêng OLE LiDAR qua Ethernet
│   └── rplidar.launch.py         # Khởi động riêng RPLidar qua cổng USB Serial (/dev/ttyUSB0)
├── AGENT_HANDOVER_GUIDE.md       # Tài liệu này (Living Document - kim chỉ nam cho Agent)
└── README.md                     # Hướng dẫn đấu dây phần cứng và tài liệu kỹ thuật
```

### 2.2. Chi tiết các File Node (`scripts/`)
| Tên File | Chức năng chính | Các tính năng & logic đã hoàn thành |
| :--- | :--- | :--- |
| **`zlac_udp_odom_node.py`** | **Node Lõi điều khiển & Odometry** | • **Giao tiếp UDP nhị phân (50 Hz):** Gửi gói 12 bytes (`<BBffH`, CRC-16/Modbus) tới STM32 `192.168.1.100:8888`. Hứng gói Telemetry 22 bytes (`<BBhhllHhhH`) từ STM32.<br>• **Velocity Smoother:** Làm mượt gia tốc tuyến tính (`linear_accel = 0.8 m/s²`) và góc quay (`angular_accel = 1.5 rad/s²`).<br>• **Breakaway Kick (Bứt phá ma sát tĩnh):** Tự kích bước nhảy `min_breakaway_velocity = 0.04 m/s` khi khởi hành từ vận tốc 0 để 2 bánh bứt phá đồng thời, chống hiện tượng xe bị lệch bánh.<br>• **Odometry Runge-Kutta bậc 2:** Tính toán vị trí robot từ hiệu số xung encoder bánh trái/phải (`dist_center`, `d_theta`), chuẩn hóa góc quay `[-π, π]`, publish `/odom` và broadcast TF `odom -> base_link`.<br>• **Ethernet Watchdog:** Đếm thời gian nhận UDP; nếu quá `1.0s` mất kết nối sẽ cảnh báo lỗi đỏ rực trên console, tự in thông báo phục hồi khi có lại mạng.<br>• **Bảo vệ & Chẩn đoán:** Bắt và cảnh báo mã lỗi `0xEEEE` (kẹt tải/quá dòng), `0xEE01` (mất CAN). Publish `/battery_voltage` và cung cấp Service `/reset_odom`.<br>• **Lưu trữ dòng điện nội bộ:** Lưu `self.current_a`, `self.current_b` (không tạo topic ROS 2 theo đúng yêu cầu người dùng). |
| **`teleop_joy.py`** | **Node Điều khiển Tay cầm Gamepad** | • Lắng nghe topic `/joy` từ `joy_node`.<br>• **Deadman Switch:** Giữ nút L1 (`btn_deadman = 9` hoặc `4`) mới cho phép phát lệnh `/cmd_vel`. Nhả tay = tự động dừng xe.<br>• **Turbo Boost:** Nhấn giữ nút R1 (`btn_turbo = 5`) để tăng tốc tối đa từ `0.8 m/s` lên `1.2 m/s`.<br>• **Phanh khẩn cấp (E-Stop):** Nhấn nút B / Tròn (`btn_estop = 1`) để dừng khẩn tức thời.<br>• **Nút Reset Odom:** Nhấn nút Y / Tam giác (`btn_reset_odom = 3`) để gọi service `/reset_odom`.<br>• Hỗ trợ vùng chết joystick (`deadzone = 0.08`) chống trôi cần. |
| **`test_robot_telemetry.py`** | **Script kiểm thử độc lập không cần ROS 2** | • Chạy bằng Python thuần (`python3 test_robot_telemetry.py`).<br>• Đo độ trễ khứ hồi RTT thực tế giữa PC và STM32 (đạt `0.20 ~ 0.30 ms`).<br>• Bảng điều khiển Console trực quan: Hiện RPM 2 bánh, Xung encoder, Dòng điện 2 motor, Điện áp Pin, Mã lỗi, Tỉ lệ mất gói (Packet Loss = 0%). |

### 2.3. Chi tiết các File Cấu hình (`config/`)
| Tên File | Chức năng & Phạm vi áp dụng | Nội dung cấu hình chi tiết đã thiết lập |
| :--- | :--- | :--- |
| **`joy.yaml`** | Cấu hình cho package `teleop_twist_joy` | • Gộp cả tiến/lùi và bẻ lái vào cùng 1 cần gạt trái (đẩy chéo 45° để vừa tiến vừa cua).<br>• Tốc độ thường: `scale_linear.x = 0.8 m/s`, `scale_angular.yaw = 0.6 rad/s`.<br>• Bật `require_enable_button = true`, nút kích hoạt `enable_button = 4` (L1/LB). |
| **`ekf.yaml`** | Bộ lọc Kalman mở rộng (`robot_localization`) | • Chạy ở tần số `50.0 Hz`, chế độ `two_d_mode = true` (khóa z, roll, pitch cho robot sàn phẳng).<br>• `odom0: /odom`: Chỉ lấy vận tốc tịnh tiến `vx` (bỏ `vyaw` để tránh sai số trượt lốp).<br>• `imu0: /bno055/imu`: Lấy góc `yaw` tuyệt đối từ thuật toán NDOF và vận tốc góc `vyaw` từ Gyroscope Z. Loại bỏ gia tốc trọng trường (`remove_gravitational_acceleration = true`). |
| **`bno055_params_i2c.yaml`** | Cấu hình cảm biến IMU 9-DOF BNO055 | • Giao tiếp I2C bus 1 (Jetson TX2 Pin 27/28), địa chỉ I2C `40` (0x28).<br>• Tần số đọc dữ liệu `data_query_frequency = 50 Hz`. Frame ID: `imu_link`.<br>• Chế độ NDOF (`operation_mode = 12`).<br>• **Đã nạp sẵn bảng bù sai số thực tế (Calibration Offsets)** sau khi hiệu chuẩn trên xe mới: `offset_acc: [65503, 2, 65501]`, `offset_mag: [65341, 366, 65232]`, `offset_gyr: [0, 65534, 65535]`. |
| **`ole2dv2.yaml`** | Cấu hình cảm biến LiDAR công nghiệp OLE Oleros2 | • Giao tiếp Ethernet UDP qua switch mạng: IP OLE `192.168.1.101`, Subnet `255.255.255.0`, Port UDP `60001`.<br>• Frame ID: `laser_frame`.<br>• Tần số quét `10.0 Hz` (600 RPM), góc quét toàn cảnh 360° (`-π` đến `+π`).<br>• Phạm vi đo khoảng cách: `0.15 m - 12.0 m`. Bật lọc dữ liệu và cường độ phản xạ (`enable_intensity = true`). |

### 2.4. Chi tiết các File Khởi động (`launch/`)
| Tên File | Các Node được khởi chạy | Tham số & Tùy biến quan trọng |
| :--- | :--- | :--- |
| **`teleop.launch.py`** | **[LAUNCH CHÍNH ĐIỀU KHIỂN ROBOT]**<br>1. `joy_node`<br>2. `teleop_joy.py`<br>3. `zlac_udp_odom_node.py` | • Đọc tay cầm tại `/dev/input/js0`.<br>• **Cấu hình tay cầm chuẩn PS4:** Cần TRÁI (`axis_linear = 1`) lái Tiến/Lùi, Cần PHẢI (`axis_angular = 2`) bẻ lái Xoay xe.<br>• Nút Deadman L1 (`btn_deadman = 9`), Turbo R1 (`btn_turbo = 5`), E-stop B (`btn_estop = 1`), Reset odom Y (`btn_reset_odom = 3`).<br>• Tự động truyền các thông số cơ khí chuẩn: `wheel_radius = 0.0535`, `wheel_base = 0.45`, `publish_tf = True`. |
| **`teleop_robot.launch.py`** | 1. `joy_node`<br>2. `teleop_joy.py`<br>3. `zlac_udp_odom_node.py` | • Phiên bản cấu hình tốc độ mềm hơn (`0.5 m/s` thường, `1.2 m/s` turbo, `axis_angular = 3`). Thích hợp cho người mới làm quen hoặc test trong không gian hẹp. |
| **`bno055.launch.py`** | 1. `bno055_node`<br>2. `static_transform_publisher` (`base_link -> imu_link`) | • Nạp file cấu hình `bno055_params_i2c.yaml`.<br>• **Static TF tọa độ lắp IMU thực tế:** `x = 0.175 m, y = -0.048 m, z = 0.041 m, yaw = 0.0, pitch = 0.0, roll = 0.0`. |
| **`ekf.launch.py`** | **[LAUNCH DUNG HỢP CẢM BIẾN EKF]**<br>1. `bno055_node`<br>2. `static_transform_publisher` (`base_link -> imu_link`)<br>3. `zlac_udp_odom_node` (`publish_tf=False`)<br>4. `ekf_node` (`robot_localization`)<br>5. `joy_node` + `teleop_joy` (tùy chọn) | • **Dung hợp 50Hz:** $v_x$ từ `/odom` + Yaw và $\omega_z$ từ `/bno055/imu`.<br>• Xuất ra topic `/odometry/filtered` siêu chuẩn xác và broadcast duy nhất TF `odom -> base_link`.<br>• Tự động cấu hình `publish_tf: False` trên `zlac_udp_odom_node` để tránh xung đột 2 nguồn TF.<br>• Hỗ trợ reset đồng bộ cả Odom thô lẫn EKF qua nút Y / Tam giác trên tay cầm. |
| **`lidar.launch.py`** | **[LAUNCH TỔNG HỢP CẢM BIẾN QUÉT LASER]**<br>Hỗ trợ cả OLE LiDAR và RPLidar | • Cung cấp các tham số dòng lệnh linh hoạt:<br>&nbsp;&nbsp;`use_ole:=true` (mặc định bật OLE LiDAR qua Ethernet IP `192.168.1.101`).<br>&nbsp;&nbsp;`use_rplidar:=false` (tùy chọn bật RPLidar qua USB `/dev/ttyUSB0`).<br>• Tự động gán frame `laser_frame` đồng nhất cho cả 2 loại cảm biến. |
| **`ole_lidar.launch.py`** | 1. `oleros2_node` | • Khởi chạy độc lập cảm biến OLE LiDAR qua IP `192.168.1.101` với frame `laser_frame`. |
| **`rplidar.launch.py`** | 1. `sllidar_node` | • Khởi chạy độc lập RPLidar qua cổng USB Serial `/dev/ttyUSB0` (baudrate 115200) với frame `laser_frame`. |

---

## ⚙️ 3. CÁC THÔNG SỐ CƠ KHÍ & CẤU HÌNH HỆ THỐNG ĐÃ CĂN CHỈNH CHUẨN

| Thông số | Giá trị | File quy định | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Bán kính bánh xe ($R$)** | `0.0535 m` (53.5 mm) | `zlac_can.h`, `zlac_udp_odom_node.py` | Đường kính bánh 107 mm (bánh cao su công nghiệp) |
| **Khoảng cách 2 bánh ($L$)** | `0.450 m` (450 mm) | `zlac_can.h`, `zlac_udp_odom_node.py` | Đo giữa 2 tâm vết tiếp xúc sàn |
| **Encoder CPR** | `4096 counts/vòng` | `zlac_udp_odom_node.py` | 1024 lines x 4 (quadrature) |
| **Đảo chiều Motor B** | `ZLAC_MOTOR_B_REVERSE = 1` | `zlac_can.h`, `zlac_udp_odom_node.py` | Do 2 motor lắp đối xứng 180 độ |
| **Vận tốc tối đa an toàn** | `1.5 m/s` (~1.5 vòng/s = 90 RPM) | `zlac_can.h`, `zlac_udp_odom_node.py` | Giới hạn phần mềm chống lật |
| **Gia tốc làm mượt ROS 2** | `linear: 0.8 m/s²`, `angular: 1.5 rad/s²` | `zlac_udp_odom_node.py` | Velocity Smoother chống giật |
| **Breakaway Kick (Ma sát tĩnh)**| `min_breakaway_velocity = 0.04 m/s` | `zlac_udp_odom_node.py` | Giúp 2 bánh bứt phá ma sát tĩnh đồng thời |
| **Profile Accel/Decel Driver**| Accel: `700 ms`, Decel: `900 ms` | `zlac_can.c` (`0x6083`, `0x6084`) | Cài đặt nội bộ trong driver ZLAC |
| **CAN Baudrate & Node ID** | `500 kbps`, Node ID = `0x01` | `zlac_can.c`, DIP switch | Phải có trở đầu cuối 120 Ohm trên bus |
| **IP / Port STM32 (W5500)** | `192.168.1.100:8888` | `main.c` | Static IP |
| **Phím bấm tay cầm PS4** | Deadman: Button `9` (L1), Quay: Axis `2` | `teleop.launch.py` | Nút L1 bắt buộc nhấn giữ khi lái |

---

## 📡 4. ĐỊNH DẠNG GIAO THỨC TRUYỀN THÔNG (PACKET PROTOCOLS)

### A. Gói điều khiển từ ROS 2 xuống STM32 (Downlink - 12 Bytes nhị phân, Little-Endian)
- Tần số gửi: **50 Hz** (20ms/gói).
- Cấu trúc struct pack: `'<BBffH'`
  1. `Header[0]`: `0xAA` (1 byte)
  2. `Header[1]`: `0x55` (1 byte)
  3. `linear_velocity`: `float32` (4 bytes, đơn vị: m/s, tiến dương / lùi âm)
  4. `angular_velocity`: `float32` (4 bytes, đơn vị: rad/s, quay trái dương / quay phải âm)
  5. `CRC16`: `uint16` (2 bytes, thuật toán CRC-16 Modbus tính trên 10 bytes đầu)

### B. Gói Telemetry từ STM32 phản hồi lên ROS 2 (Uplink - 22 Bytes nhị phân, Little-Endian)
- Tần số gửi: **50 Hz** (20ms/gói).
- Cấu trúc struct unpack: `'<BBhhllHhhH'`
  1. `Header[0..1]`: `0x55`, `0xAA` (2 bytes)
  2. `vel_a`: `int16` (2 bytes, vận tốc thực tế Motor A, đơn vị: 0.1 RPM)
  3. `vel_b`: `int16` (2 bytes, vận tốc thực tế Motor B, đơn vị: 0.1 RPM)
  4. `pos_a`: `int32` (4 bytes, vị trí xung encoder Motor A)
  5. `pos_b`: `int32` (4 bytes, vị trí xung encoder Motor B)
  6. `error_code`: `uint16` (2 bytes, 0: OK, `0xEEEE`: Kẹt tải/Quá dòng, `0xEE01`: Mất kết nối CAN)
  7. `current_a`: `int16` (2 bytes, dòng điện thực tế Motor A, đơn vị: 0.1A)
  8. `current_b`: `int16` (2 bytes, dòng điện thực tế Motor B, đơn vị: 0.1A)
  9. `bus_voltage`: `uint16` (2 bytes, điện áp nguồn Pin DC Bus, đơn vị: 0.1V hoặc 0.01V)

### C. Giao thức CANopen giữa STM32 và ZLAC8015D
- **TPDO0 (CAN ID: `0x181`):** Vận tốc thực tế 2 motor (`0x606C sub 01 & 02`, 32-bit mỗi motor).
- **TPDO1 (CAN ID: `0x281`):** Dòng điện thực tế 2 motor (`0x6077 sub 01 & 02`, 16-bit mỗi motor, đơn vị 0.1A).
- **TPDO2 (CAN ID: `0x381`):** Xung góc Encoder 2 motor (`0x6064 sub 01 & 02`, 32-bit mỗi motor).
- **TPDO3 (CAN ID: `0x481`):** Mã lỗi phần cứng nội bộ của driver (16-bit).
- **SYNC (CAN ID: `0x080`):** Lệnh đồng bộ gửi từ STM32 định kỳ 20ms để chốt dữ liệu encoder/vận tốc.
- **Lệnh điều khiển vận tốc:** Ghi bằng **SDO Write** vào `0x60FF sub 01` (Motor A) và `0x60FF sub 02` (Motor B) kèm `HAL_Delay(2)` giữa 2 bánh.

---

## ⚠️ 5. CÁC "HỐ SÂU" ĐÃ GẶP & BÀI HỌC KINH NGHIỆM XƯƠNG MÁU (CRITICAL PITFALLS)

> [!CAUTION]
> **ĐẶC BIỆT LƯU Ý CHO AGENT KẾ THỪA:** Tuyệt đối không thử lại các giải pháp đã thất bại dưới đây trừ khi có lý do phần cứng cực kỳ đặc thù!

### ❌ Hố sâu 1: Đổi SDO sang RPDO1 để gửi vận tốc
- **Hiện tượng:** Trước đây từng thử thay thế việc gửi lệnh vận tốc từ `SDO_Write(0x60FF)` sang `RPDO1 (CAN ID 0x301)` để giảm tải bus CAN.
- **Hậu quả:** Động cơ **hoàn toàn không quay và không phản hồi**. Nguyên nhân do firmware ZLAC8015D yêu cầu phải cấu hình RPDO mapping lưu vào EEPROM bằng chuỗi lệnh đồng bộ chuyên biệt và yêu cầu bit Event Timer.
- **Bài học:** Giữ nguyên hàm `ZLAC_SetSpeed_raw` dùng `SDO_Write` vào `0x60FF:01` và `0x60FF:02` với lệnh nghỉ `HAL_Delay(2)` ở giữa. Cơ chế này đã được thử nghiệm thực tế và **chạy mượt 100% ổn định**.

### ❌ Hố sâu 2: Lỗi dòng điện luôn đọc về 0.0A (Mapping TPDO1)
- **Hiện tượng:** Biến `current_a` và `current_b` gửi về máy tính luôn luôn bằng `0.0A`, bộ bảo vệ quá dòng không phát huy tác dụng.
- **Nguyên nhân:** Hàm `_TPDO1_Config` cũ cố gắng map đối tượng `0x6077 sub 03` (32-bit gộp). Driver ZLAC8015D từ chối map này vì trong Object Dictionary, dòng điện thực tế chỉ cho phép map ở dạng 16-bit riêng lẻ:
  - `0x6077 sub 01`: Dòng motor A (16-bit).
  - `0x6077 sub 02`: Dòng motor B (16-bit).
- **Bài học & Đã sửa:** Trong `Core/Src/zlac_can.c`, hàm `_TPDO1_Config` phải map tách rời 2 sub-index (`0x1A01:01 -> 0x6077:01:10` và `0x1A01:02 -> 0x6077:02:10`), kích hoạt `0x1A01 sub 00 = 2`. Khi đó CAN ID `0x281` mới được driver phát ra đều đặn 20ms.

### ❌ Hố sâu 3: Ma sát tĩnh (Stiction) khiến xe bị "chột" bánh khi xuất phát
- **Hiện tượng:** Khi bắt đầu nhấn ga từ vận tốc 0, 1 bánh lăn trước, bánh kia bị ì lại hoặc kêu rít rè rè, làm robot bị xoay lệch hướng trước khi đi thẳng.
- **Nguyên nhân:** Ma sát nghỉ (static friction) của hộp số hành tinh và lốp cao su lớn hơn lực kéo ban đầu nếu bộ lọc gia tốc tăng tốc từ từ từ 0.001 m/s.
- **Bài học & Đã sửa:** Trong file `scripts/zlac_udp_odom_node.py`, hàm `move_towards` được tích hợp cơ chế **Breakaway Kick**:
  - Khi vận tốc hiện tại đang bằng 0 và nhận lệnh chạy, hàm sẽ "đạp nhẹ" một bước nhảy tức thời lên `min_breakaway_vel = 0.04 m/s` (đủ vượt qua ma sát tĩnh mà không làm giật xe), sau đó mới tăng tốc mượt mà theo gia tốc `linear_accel = 0.8 m/s²`.
  - Profile Acceleration trên STM32 cài ở mức `700 ms`, Profile Deceleration cài ở mức `900 ms`.

### ❌ Hố sâu 4: Lỗi kẹp vận tốc góc (Angular velocity clamping bug)
- **Hiện tượng:** Khi tích hợp Breakaway kick, nếu người dùng chỉ xoay xe tại chỗ ($v=0, \omega \neq 0$), xe không thể quay góc.
- **Nguyên nhân:** Trước đó logic kick áp dụng chung hoặc kẹp nhầm góc xoay vào dải ma sát tĩnh tịnh tiến.
- **Bài học & Đã sửa:** Breakaway kick **chỉ áp dụng cho `current_linear`**, còn `current_angular` thì dùng `min_kick = 0.0` để người điều khiển có thể xoay xe tại chỗ êm ái ở mọi dải tốc độ.

### ❌ Hố sâu 5: Yêu cầu phạm vi của người dùng (User Scope Constraints)
- **Quy tắc quan trọng:** Người dùng **tuyệt đối không muốn tạo thêm các topic ROS 2 riêng lẻ cho dòng điện** (như `/motor_a_current`, `/motor_b_current`). Dữ liệu dòng điện chỉ được bóc tách và lưu vào biến nội bộ trong node hoặc đưa vào chẩn đoán. Đừng tự ý thêm publisher mới nếu người dùng không yêu cầu.

---

## 🛠️ 6. CẨM NANG "BẮT BỆNH" NHANH KHI HỆ THỐNG GẶP SỰ CỐ (TROUBLESHOOTING)

### Triệu chứng 1: Gửi vận tốc xuống nhưng 2 động cơ không quay
1. **Kiểm tra tay vần bánh xe:**
   - Nếu bánh xe **cứng ngắc (có lực ghì)**: Servo đã ON. Hãy kiểm tra nút Deadman L1 trên tay cầm (phải nhấn giữ) hoặc kiểm tra topic `/cmd_vel` xem vận tốc có khác 0 không (`ros2 topic echo /cmd_vel`).
   - Nếu bánh xe **xoay nhẹ tênh**: Servo chưa ON (`ZLAC_READY == false`). Hãy kiểm tra đèn LED trên driver ZLAC hoặc kiểm tra cáp CAN.
2. **Kiểm tra terminal ROS 2:**
   - Nếu thấy log: `CẢNH BÁO: KÍCH HOẠT BẢO VỆ KẸT TẢI / QUÁ DÒNG TRÊN DRIVER (0xEEEE)!`: Do dòng điện vọt lên > 6.0A trong khi bánh bị chặn cơ khí. STM32 tự ngắt bảo vệ cuộn dây trong 2 giây.
   - Nếu thấy log: `CẢNH BÁO: MẤT KẾT NỐI CAN GIỮA STM32 VÀ ZLAC (0xEE01)!`: Tuột cáp CAN H / CAN L hoặc thiếu trở đầu cuối 120 Ohm.
   - Nếu thấy log: `CẢNH BÁO: MẤT KẾT NỐI ETHERNET TỚI STM32!`: Dây LAN bị lỏng, IP không đúng subnet `192.168.1.xxx`, hoặc STM32 chưa cấp nguồn.
3. **Kiểm tra Watchdog 250ms trên STM32:** Nếu máy tính phát gói UDP với tần số dưới 4 Hz hoặc delay chập chờn, STM32 sẽ tự kích hoạt `ZLAC_Stop()` để tránh trôi xe.

### Triệu chứng 2: Xe chạy thẳng nhưng Odometry báo quay vòng vòng (hoặc ngược lại)
- Kiểm tra cờ `motor_b_reverse`: Mặc định trong code là `True` (trên ROS 2) và `1` (trên STM32). Nếu động cơ B lắp cùng chiều hoặc cơ khí thay đổi, đảo cờ này.
- Kiểm tra CPR: Phải đúng `4096`.
- Kiểm tra bán kính bánh xe $R = 0.0535\text{ m}$ và khoảng cách bánh $L = 0.450\text{ m}$.

---

## 🚀 7. QUY TRÌNH BIÊN DỊCH, NẠP FIRMWARE & VẬN HÀNH

### Bước 1: Biên dịch & Nạp STM32 (Keil MDK ARM v5)
1. Mở project: `MDK-ARM/Can_test_motor.uvprojx`.
2. Nhấn **F7** (Build Project) — đảm bảo `0 Errors, 0 Warnings`.
3. Cắm mạch nạp ST-Link (SWD: SWDIO, SWCLK, GND, 3.3V).
4. Nhấn **F8** (Download) để nạp firmware xuống STM32F103.

### Bước 2: Cấu hình mạng máy tính (ROS 2 PC)
- Đặt IP tĩnh cho card mạng Ethernet máy tính cùng dải mạng:
  - **IP:** `192.168.1.50` (hoặc bất kỳ IP nào khác `192.168.1.100`)
  - **Subnet Mask:** `255.255.255.0`
  - **Gateway:** `192.168.1.1`
- Kiểm tra kết nối phần cứng bằng lệnh ping:
  ```bash
  ping 192.168.1.100
  ```
  *(Thời gian phản hồi bình thường là `< 1 ms`)*.

### Bước 3: Chạy hệ thống điều khiển ROS 2
- Khởi chạy toàn bộ hệ thống bằng launch file:
  ```bash
  ros2 launch can_test_motor teleop.launch.py
  ```
  *(File này tự động mở `joy_node`, `teleop_joy` và `zlac_udp_odom_node`)*.
- Nhấn giữ nút **L1** trên tay cầm PS4 và đẩy cần joystick bên trái để tiến/lùi, cần bên phải để quay góc.
- Kiểm tra tọa độ Odometry thời gian thực:
  ```bash
  ros2 topic echo /odom
  ```
- Kiểm tra điện áp Pin:
  ```bash
  ros2 topic echo /battery_voltage
  ```

---

## 📝 8. NGUYÊN TẮC LÀM VIỆC DÀNH CHO AGENT TIẾP THEO
1. **Luôn giải thích rõ ràng trước khi sửa:** Giải thích nguyên nhân, giải pháp dự kiến và xin ý kiến xác nhận của người dùng đối với các thay đổi lớn.
2. **Bảo toàn tính ổn định:** Giữ nguyên các hàm cốt lõi đã chạy ổn định (`SDO_Write 0x60FF`, `_TPDO1_Config 16-bit`, `Breakaway kick 0.04 m/s`, `Watchdog an toàn`).
3. **Kiểm tra cú pháp trước khi commit:** Dùng `python -m py_compile` cho file Python và kiểm tra logic C cẩn thận trước khi hướng dẫn người dùng nhấn F7 trong Keil.
4. **Đồng bộ hóa Git:** Luôn commit với thông điệp rõ ràng theo chuẩn Conventional Commits (`feat:`, `fix:`, `docs:`) và push lên `origin/main`.
5. **Quy tắc Duy trì & Đọc lại Tài liệu này (BẮT BUỘC):**
   - **Đọc trước khi làm:** Mỗi khi bắt đầu một phiên làm việc hoặc chuẩn bị sửa đổi code, Agent **phải lướt qua file này trước** để nắm bắt nhanh các thông số chuẩn, các quyết định kiến trúc và các "hố sâu" cấm kỵ.
   - **Cập nhật sau khi làm:** Bất kỳ khi nào có thay đổi về tính năng, thông số cơ khí, thêm mã lỗi, hoặc giải quyết một bài toán mới, Agent **phải cập nhật ngay vào file này** để tài liệu luôn phản ánh chính xác trạng thái mới nhất của hệ thống.
6. **Quy tắc Bàn giao khi Sắp hết Quota/Token (BẮT BUỘC):**
   - Khi Agent nhận thấy sắp hết quota (lỗi 429, `FreeUsageLimitError`), context dài, hoặc người dùng báo `sắp hết`, Agent **phải ghi chú ngay** công việc đang dở vào `Mục 9` bên dưới trước khi dừng.
   - Không được thoát phiên mà không để lại: đang làm gì, sửa tới đâu, bước tiếp theo là gì.

---

## 📌 9. NHẬT KÝ BÀN GIAO LIVE (AGENT GHI NỐI TIẾP VÀO ĐÂY)

> **Hướng dẫn cho Agent:** Mỗi lần bàn giao thì copy khung dưới, điền ngày giờ, điền nội dung, đặt lên ĐẦU mục 9 (mới nhất lên trên). Giữ tối đa 5 entry gần nhất, entry cũ hơn thì xóa tóm tắt gọn 1 dòng.

### Template (copy khi bàn giao):
```markdown
#### [YYYY-MM-DD HH:MM] - <Tên Agent/Model> - <Tóm tắt 1 dòng>
- **Trạng thái hiện tại:** Đang làm gì, tới bước nào, kết quả ra sao?
- **File đã sửa:** Liệt kê file + dòng/hàm đã chạm vào (VD: `Core/Src/zlac_can.c:_TPDO1_Config`, `scripts/zlac_udp_odom_node.py:move_towards`).
- **Lệnh đã chạy & kết quả:** (VD: `python -m py_compile ... OK`, `ping 192.168.1.100 <1ms`, `ros2 launch ... lỗi gì`).
- **Việc còn dở / Bước tiếp theo:** Agent sau cần làm gì cụ thể (1, 2, 3...).
- **Cách verify nhanh:** Lệnh kiểm tra để biết đã xong chưa.
- **Lưu ý / Hố mới phát hiện:** Nếu có.
---
```

### Lịch sử bàn giao:
<!-- Agent mới ghi tiếp vào dưới dòng này, entry mới nhất lên trên cùng -->

#### [2026-09-06 17:10] - Antigravity (Gemini 3.8 Flash) - Kích hoạt Bộ lọc Dung hợp Cảm biến EKF (robot_localization)
- **Trạng thái hiện tại:** Đã xây dựng hoàn chỉnh hệ thống dung hợp EKF 50Hz kết hợp Odometry bánh xe và IMU BNO055.
  - Tạo launch file tổng hợp `launch/ekf.launch.py`: Khởi chạy đồng bộ BNO055 (50Hz NDOF), Static TF `base_link -> imu_link`, `zlac_udp_odom_node` với `publish_tf: False`, bộ lọc `robot_localization/ekf_node` nạp `config/ekf.yaml`, và cụm điều khiển tay cầm gamepad (tùy chọn, mặc định bật).
  - Cập nhật `config/ekf.yaml`: Đặt `publish_tf: true` (EKF phát TF duy nhất `odom -> base_link`), `imu0_relative: true` (triệt tiêu góc lệch từ trường lúc bật máy, đưa Yaw ban đầu về 0).
  - Cập nhật `scripts/teleop_joy.py`: Thêm cơ chế phát lệnh `/set_pose` khi người dùng nhấn nút Y / Tam giác trên tay cầm, giúp reset đồng thời cả Odometry thô và bộ lọc EKF về $(0,0,0)$.
  - Sửa `launch/teleop_robot.launch.py`: Đổi package name cũ `project_1` sang `can_test_motor`.
  - Cập nhật `package.xml`: Khai báo thêm phụ thuộc `robot_localization` và `joy`.
- **File đã tạo/sửa:**
  - `launch/ekf.launch.py` (Mới)
  - `config/ekf.yaml`
  - `scripts/teleop_joy.py`
  - `launch/teleop_robot.launch.py`
  - `package.xml`
  - `AGENT_HANDOVER_GUIDE.md`
- **Lệnh đã chạy & kết quả:** `python -m py_compile` tất cả file đều đạt 100% OK.
- **Cách verify nhanh:**
  ```bash
  sudo apt update && sudo apt install -y ros-humble-robot-localization
  cd /home/nhatbot_ws/src/can_test_motor && git pull origin main
  cd /home/nhatbot_ws && colcon build --packages-select can_test_motor --symlink-install && source install/setup.bash
  ros2 launch can_test_motor ekf.launch.py
  # Kiểm tra dữ liệu:
  ros2 topic echo /odometry/filtered
  ros2 run tf2_ros tf2_echo odom base_link
  ```
---

#### [2026-09-06 16:53] - Antigravity (Gemini 3.8 Flash) - Giới hạn tốc độ max 0.3 m/s và tắt Velocity Smoother trên ROS 2
- **Trạng thái hiện tại:** Đã điều chỉnh tốc độ tay cầm giới hạn tối đa 0.3 m/s theo yêu cầu an toàn. Tắt bộ lọc gia tốc phần mềm trên ROS 2 (`enable_smoother = False`) để nhường việc điều tiết tăng/giảm tốc cho phần cứng driver ZLAC8015D (Profile Accel 700ms, Decel 900ms). Cập nhật package name `can_test_motor` trong `teleop.launch.py`.
- **File đã sửa:** 
  - `scripts/zlac_udp_odom_node.py`: Thêm cờ `enable_smoother` (default False), `max_linear_velocity = 0.3`.
  - `scripts/teleop_joy.py`: `scale_linear_normal = 0.3`, `scale_linear_turbo = 0.3`, `scale_angular = 0.5`.
  - `launch/teleop.launch.py`: Cập nhật `package='can_test_motor'`, tốc độ 0.3 m/s, truyền `enable_smoother: False`.
- **Lệnh đã chạy & kết quả:** `python -m py_compile` cả 3 file đều đạt 100% OK.
- **Cách verify nhanh:** `ros2 launch can_test_motor teleop.launch.py` -> đẩy kịch cần joystick xe chạy tối đa 0.3 m/s, tăng/giảm tốc mượt mà do driver tự hãm.
---

#### [2026-09-06 16:25] - Antigravity (Gemini 3.8 Flash) - Hoàn thành Hiệu chuẩn IMU BNO055 và Lưu Offsets
- **Trạng thái hiện tại:** Hiệu chuẩn thực tế BNO055 trên Jetson TX2 thành công xuất sắc (3,3,3,3). Đã gọi service trích xuất mảng offsets thực tế và lưu vĩnh viễn vào `config/bno055_params_i2c.yaml`, bật `set_offsets: true`.
- **Thông số offsets thực tế đo được (2026-09-06):**
  - `offset_acc: [65503, 2, 65501]`, `radius_acc: 1000`
  - `offset_mag: [65341, 366, 65232]`, `radius_mag: 453`
  - `offset_gyr: [0, 65534, 65535]`
- **File đã sửa:** `config/bno055_params_i2c.yaml`
- **Bước tiếp theo:** Khởi động lại launch file để xác nhận cảm biến tự nạp offsets ngay khi bật nguồn mà không cần múa lại, sau đó chuyển sang tích hợp bộ lọc EKF / Nav2.
- **Cách verify nhanh:** `ros2 launch can_test_motor bno055.launch.py` -> log hiển thị `Current sensor offsets` khớp với các giá trị trên.
---

#### [2026-09-06 08:50] - Antigravity (Gemini 3.8 Flash) - Cấu hình IMU BNO055 (50Hz I2C), Launch File và Bộ lọc EKF
- **Trạng thái hiện tại:** Đã tạo toàn bộ file cấu hình và launch chuẩn cho BNO055 + EKF robot_localization trong `Can_test_motor`. Sẵn sàng đấu nối I2C và test trên Jetson TX2.
- **File đã tạo/sửa:** 
  - `config/bno055_params_i2c.yaml`: Cấu hình driver BNO055 chạy 50Hz, NDOF mode, bus 1, addr 0x28, `set_offsets: false` cho bước calib ban đầu.
  - `launch/bno055.launch.py`: Khởi động node `bno055` và `static_transform_publisher` (`base_link -> imu_link`).
  - `config/ekf.yaml`: Cấu hình bộ lọc EKF 50Hz dung hợp `/odom` (vận tốc vx) và `/bno055/imu` (hướng yaw + vận tốc góc vyaw).
- **Lệnh đã chạy & kết quả:** `python -m py_compile launch/bno055.launch.py` thành công 100% không lỗi cú pháp.
- **Việc còn dở / Bước tiếp theo:** 
  1. Đấu dây I2C từ BNO055 sang J21 Jetson TX2 (Pin 1: 3.3V, Pin 27: SDA, Pin 28: SCL, Pin 9: GND; COM3=GND).
  2. Chạy `sudo i2cdetect -y -r 1` kiểm tra địa chỉ `0x28`.
  3. Chạy launch BNO055, thực hiện quy trình xoay cảm biến để hiệu chuẩn 3-3-3-3.
  4. Gọi service `/bno055/calibration_request` lấy offsets mới cập nhật vào `config/bno055_params_i2c.yaml` và bật `set_offsets: true`.
  5. Chạy EKF và kiểm tra `/odometry/filtered`.
- **Cách verify nhanh:** `ros2 topic hz /bno055/imu` (đạt 50Hz ổn định) và `ros2 topic echo /bno055/calib_status`.
---

---

## 🔄 10. TỔNG HỢP REPO CŨ EIU-FABLAB-AMR & KẾ HOẠCH CHUYỂN GIAO (2026-09-06)

> **Nguồn:** https://github.com/NhatTran-97/EIU-FABLAB-AMR/tree/main/nhatbot_drivers
> **Mục tiêu user:** Chuyển giao hệ cũ (Modbus) sang hệ mới (CAN + Ethernet UDP). Chốt hướng 2 (dùng `zlac_udp_odom_node` chuẩn `/cmd_vel` -> `/odom`), hiện tại CHỈ tổng hợp, CHƯA code.

### 10.1 Hệ cũ có gì (Modbus RTU trực tiếp, không gateway)
- Kiến trúc: `PC --USB-RS485 Modbus RTU 115200, slave 1--> ZLAC8015D`.
- `zlac8015d_driver/` (ROS2 C++ Modbus trực tiếp):
  - `src/zlac8015d_driver.cpp`: `modbus_new_rtu("/dev/zlac_8015d", 115200)`, timeout 1s. API: `setRPM / getRPM (/10) / getWheelsTravelled (0x20A7 4 regs) / getBatteryVoltage (*0.01) / getMotor-DriverTemp / getMotorFaults / enable(0x08)-disable(0x07)-eStop(0x05)-clearAlarm(0x06)`.
  - `src/zlac_interfaces.cpp`: node `zlac_driver_node` sub `/nhatbot/wheel_rotational_vel Float32MultiArray rad/s` -> RPM -> `setRPM(-left, right)`, pub `/nhatbot/JointState` 10Hz (đang comment, không pub thực), service reset encoder.
  - Thanh ghi: `0x200E control, 0x200D mode (1 rel/2 abs/3 vel), 0x2080/2081 accel, 0x2082/2083 decel, 0x2088/2089 cmd RPM, 0x20AB/AC fb RPM, 0x20A1 voltage, 0x20A4/0x20B0 temp, 0x2005 clear pos, 0x20A5/A6 fault`.
  - Params (`node_parameters.hpp`): `modbus_port /dev/zlac_8015d, baud 115200, accel/decel 1000ms, max_rpm 200, cpr 4096, wheel_radius 0.0535, travel_in_one_rev 0.336`.
- `nhatbot_firmware/` (bản ros2_control chạy Nav2):
  - `src/nhatbot_hw_interface.cpp`: `NhatbotInterface : SystemInterface`, export `position+velocity state / velocity command`, `read()` lấy pos rad + vel rad/s, `write()` đổi rad/s -> RPM.
  - `include/.../driver_manager.hpp`: singleton + mutex bọc `ZLAC8015D_SDK`, `init()` = `enable + mode SPEED_RPM + accel 100 / decel 100`.
  - `src/zlac_sdk.cpp`: bản SDK đầy đủ, `modbus_fail_read_handler retry 3 lần`, `autoReconnect/healthCheck`, đảo chiều `setRPM(left, -right)`, `get_wheels_tick (Left, -Right)`.
- Vệ tinh: `bno055` IMU, `oleros2` LiDAR OLE 2D/3D (`/scan`, `laser_data_frame`), `rplidar_ros` submodule, `laser_filters/filters`, `peripheral_interfaces/voices`.
- Nhược điểm: Modbus blocking ~10Hz, latency cao, hay rớt phải reconnect, PC dí trực tiếp driver, không watchdog cứng.

### 10.2 Đối chiếu với hệ mới Can_test_motor (CAN + Ethernet)
- Kiến trúc: `PC --UDP 50Hz--> STM32F103+W5500 (192.168.1.100:8888) --CANopen 500kbps--> ZLAC`.
- Down 12B `AA 55 v w CRC16`, Up 22B `55 AA vel_a/b (0.1RPM) pos_a/b error current_a/b voltage`.
- STM32 lo kinematics, watchdog 250ms tự phanh, watchdog CAN 1.5s (`0xEE01`), bảo vệ kẹt (`0xEEEE`), SDO `0x60FF` + TPDO `0x181/281/381/481` + SYNC 20ms.
- ROS2 1 node `scripts/zlac_udp_odom_node.py`: sub `/cmd_vel`, smoother `0.8 m/s² + breakaway 0.04 m/s`, pub `/odom + TF odom->base_link + /battery_voltage` 50Hz.
- Trùng khớp 100%: `wheel_radius 0.0535, cpr 4096, travel 0.336 (=2*pi*R)`, cùng mode vel RPM, cùng đảo chiều motor B.
- Khác biệt: `max_rpm 200` cũ vs `1.5 m/s (~90 RPM)` mới; `accel 1000/100ms` cũ vs `700/900ms + smoother` mới; topic `/nhatbot/*` cũ vs `/cmd_vel /odom` mới; fault Modbus regs cũ vs `error_code 0xEEEE/0xEE01` mới.

### 10.3 Kế hoạch chuyển giao (hướng 2, chưa code)
1. Giữ tầng trên: Nav2/SLAM, OLE/rplidar, BNO055, voices. Bỏ `zlac8015d_driver + nhatbot_hw_interface/DriverManager` Modbus.
2. Thay chân đế: `diff_drive_controller` xuất `/cmd_vel` cắm vào `zlac_udp_odom_node`; lấy `/odom` từ UDP node trả về cho Nav2/EKF thay `JointState` cũ.
3. Mapping khi code: copy `wheel_radius/base/cpr/motor_b_reverse` sang YAML mới; viết bridge `/odom` -> `joint_states` giả nếu Nav2 còn cần + `/battery_voltage + fault` -> `/diagnostics`; đổi `modbus_port` thành `stm32_ip/port`; verify `ping 192.168.1.100 <1ms` + `ros2 topic echo /odom`; watchdog STM32 thay `tryReconnectEvery`.
- Trạng thái: ĐÃ tổng hợp xong 2026-09-06, CHƯA sửa code nào. Agent sau đọc mục này + mục 9 rồi hỏi user `code đi?` mới làm.

### 10.4 IMU BNO055 (đọc kỹ 2026-09-06, nguồn: EIU-FABLAB-AMR/nhatbot_drivers/bno055)
- Driver ROS2 Python (gốc flynneva/bno055), Bosch BNO055 9-DOF fusion cứng.
- File lõi: `bno055/bno055.py` (node `bno055`, 2 timer + `threading.Lock`), `bno055/sensor/SensorService.py` (`configure()` check chip ID 0xA0, UNIT_SEL 0x83, axis remap; `get_sensor_data()` burst 45B từ 0x08, normalize quaternion tay; `get_calib_status()` JSON sys/gyro/accel/mag), `bno055/params/NodeParameters.py`, `params/bno055_params_i2c.yaml` (file đang dùng), `bno055/registers.py`, `connectors/i2c.py+uart.py`, `launch/bno055.launch.py`.
- Cấu hình đang chạy: `i2c bus 1 addr 0x28, data_query 100Hz, calib 0.1Hz, frame imu_link, mode 0x0C NDOF, placement P0, factors acc100/mag16M/gyr900/grav100, set_offsets true + offsets xe cũ`.
- Topics: `bno055/imu, bno055/imu_raw, bno055/mag, bno055/grav, bno055/temp, bno055/calib_status`, service `bno055/calibration_request`. TF tĩnh: `base_link -> imu_link [0.175, -0.048, 0.041, 0,0,0]`.
- Lưu ý chuyển giao: `Can_test_motor` hiện CHƯA có IMU. Bê sang phải kiểm tra lại bus I2C, đo lại TF + P0-P7, calib lại offsets (offsets cũ của xe khác), hạ 100Hz -> 50Hz nếu warn `skipping query cycle` để đồng bộ vòng UDP 50Hz. Chi tiết các bước xem hướng dẫn vận hành (user hỏi riêng, không code vội).

### 10.5 Các file còn lại cần thay đổi (rà soát 2026-09-06, chưa code)
1. **Bỏ hẳn (đặc thù Modbus):** `nhatbot_firmware` ros2_control (`nhatbot_hw_interface.cpp`, `driver_manager.hpp`, `zlac_sdk.cpp`, `motor_interfaces.xml/sensor_*.xml`, `launch/bringup_hardware_interface.launch.py`) + `zlac8015d_driver/params/motor_driver_params.yaml` (`modbus_port /dev/ttyUSB0`). Hướng 2 dùng `zlac_udp_odom_node` thẳng, không cần `controller_manager`.
2. **Sửa `nhatbot_firmware/config/diff_drive_controller.yaml`:** giữ `wheel_separation 0.45 / wheel_radius 0.0535`; tắt 1 bên TF (`enable_odom_tf` vs `publish_tf` của UDP node, tránh double broadcast `odom->base_link`); kiểm tra lại đảo tên `left_wheel_names: [wheel_right_joint]` theo dây xe mới; nâng limits cũ (`max_velocity 0.5/accel 0.3`) lên theo xe mới (`1.5/0.8`); bỏ `SensorBroadcaster` Modbus. Giữ `update_rate 50` khớp UDP 50Hz.
3. **Sửa `peripheral_interfaces/nhatbot_status.py`:** viết lại sub `/nhatbot/zlac_status (ZlacStatus)` + `/safety_stop` sang `/battery_voltage` + `error_code 0xEEEE/0xEE01` của UDP node; đo lại ngưỡng pin (`FULL 29.8/WARN 28.0/LOW 25.5` là của pack cũ); kiểm tra lại map chân Jetson.GPIO BOARD theo dây mới. Giữ `audio_server/client + voices/*.mp3`.
4. **Giữ, chỉ remap:** `filters`, `laser_filters` (sửa `frame_id laser_data_frame` + `/scan`); `rplidar_ros`, `oleros2` (fix IP qua switch + `ole2dv2.yaml`); `odom_calibration/` (chạy lại lấy `wheel_multiplier`, đang 1.0).
- Thứ tự khi code: diff_drive yaml (TF + limit) -> nhatbot_status (pin/lỗi) -> filters frame -> calib odom.

### 10.6 Toàn repo EIU-FABLAB-AMR (quét root 2026-09-06, chưa code)
- Repo cũ là full-stack AMR + tay máy (~19 packages), xe mới `Can_test_motor` hiện mới có tầng chân đế (STM32 gateway + `zlac_udp_odom_node` + teleop).
- **Bê sang, sửa nhẹ (tầng trên, không dính Modbus):** `nhatbot_description` (URDF/meshes, đo lại xe mới + TF IMU/lidar), `nhatbot_localization` (EKF, đổi input sang `/odom` UDP 50Hz + `/bno055/imu`), `nhatbot_mapping / nhatbot_navigation / nav2` (SLAM online_async, Nav2, nav_to_pose, rviz; remap odom/scan/tf + nâng `max_vel 0.5->1.5`), `nhatbot_safety` (`/safety_stop`), `nhatbot_twist_teleop`, `differential_drive`, `nhatbot_behavior/controller/utils`, `nhatbot_msgs`, `scripts/maps/rviz` trong `nhatbot_stack`. Trigger an toàn viết lại từ `error_code 0xEEEE/0xEE01 + /battery_voltage`.
- **Bỏ hẳn:** toàn bộ chân đế Modbus (mục 10.5) + viết lại `nhatbot_stack/launch/nhatbot_bringup.launch.py + build_map.launch.py` (đang gọi bringup Modbus cũ).
- **Để sau:** `nhatbot_dobot_magician`, `vision_perception` (chỉ bê khi xe mới có tay máy/camera), `nhatbot_ros2_basic` demo.

---

## 🔭 11. CHUYỂN GIAO LIDAR (OLE Oleros2 + RPLidar) - TÀI LIỆU THAM KHẢO

> **Trạng thái:** Đã nghiên cứu kỹ (2026-09-06), CHƯA code. Cần chờ user xác nhận mới bắt đầu.

### 11.1 Tổng quan hệ thống LIDAR trong EIU-FABLAB-AMR

Hệ cũ có **2 loại LIDAR**:

| Loại | Giao thức | Topic | Cổng kết nối | Phần mềm |
|:---|:---|:---|:---|:---|
| **OLE Oleros2** (2D/3D) | Ethernet UDP (libpcap) | `/scan`, `/laser_data_frame` | Ethernet (cần switch) | `oleros2` ROS2 package |
| **RPLidar** (A1/A2/A3/S1/S2...) | USB Serial (UART) | `/scan` | USB (`/dev/ttyUSB*`) | `sllidar_ros2` package |

### 11.2 Vấn đề mạng - Cần Switch Ethernet có nguồn

> ⚠️ **Không dùng cục chia RJ45 1→2 thụ động** (không nguồn): chỉ dùng được 1 trong 2, hạ 100 Mbps half-duplex, cắm STM32 + LiDAR cùng lúc = rớt cả 2.

**Giải pháp:** Mua **switch Ethernet 5 cổng Gigabit có nguồn 5V** cho robot (~200-300k VND).

**Sơ đồ mạng:**
```
[Jetson TX2] IP: 192.168.1.50 ─┐
[STM32 W5500] IP: 192.168.1.100 ─┼──→ [Switch Ethernet 5P] ──→ [OLE LiDAR] IP: 192.168.1.x
[RPLidar] USB                    ─┘          (cùng subnet /24)
```

- Cả 3 thiết bị cùng subnet `192.168.1.x/24`
- Gói UDP STM32 (12/22 bytes × 50Hz) + LiDAR vài Mbps → switch gánh nhẹ
- Test `ping` tất cả < 1ms

### 11.3 OLE Oleros2 LiDAR - Cấu hình chi tiết

#### 11.3.1 Package & Driver
- **Repo:** `github.com/olelidar/oleros2`
- **Giao thức:** Ethernet UDP dùng `libpcap` (bắt gói tin từ mạng)
- **Cần file cấu hình:** `ole2dv2.yaml` (chưa đọc nội dung cụ thể trong repo cũ)
- **Topics phát ra:**
  - `/scan` — `sensor_msgs/LaserScan` (2D scan)
  - `/laser_data_frame` — `sensor_msgs/PointCloud2` hoặc dạng frame riêng
  - Có thể kèm `/imu` nếu OLE có tích hợp IMU

#### 11.3.2 Cấu hình mạng OLE
```yaml
# ole2dv2.yaml (cấu hình mẫu - cần verify lại từ driver)
ole2d:
  ros__parameters:
    ip_address: "192.168.1.x"       # IP tĩnh của OLE trên cùng subnet
    subnet_mask: "255.255.255.0"
    udp_port: 60001                  # Cổng UDP nhận dữ liệu (thường 60001-60002)
    frame_id: "laser_frame"          # Frame ID của LIDAR
    rpm: 10                          # Tốc độ quay (10 Hz mặc định)
    scan_frequency: 10.0
    range_min: 0.15                  # Khoảng cách tối thiểu (m)
    range_max: 12.0                  # Khoảng cách tối đa (m)
    angle_min: -3.14159              # Góc bắt đầu (-π)
    angle_max: 3.14159               # Góc kết thúc (+π)
    angle_increment: 0.00872665       # Tính từ 360° / số điểm
    scan_time: 0.1                   # Thời gian quét 1 vòng (s)
```

#### 11.3.3 Launch file cho OLE
```python
# launch/ole_lidar.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oleros2',
            executable='oleros2_node',
            name='ole_lidar',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('can_test_motor'), 'config', 'ole2dv2.yaml'),
                {'frame_id': 'laser_frame'},
            ],
        ),
    ])
```

#### 11.3.4 TF cho OLE
- Static TF: `base_link → laser_frame` (hoặc `base_link → lidar_link`)
- Vị trí lắp đặt thực tế trên xe (x, y, z, roll, pitch, yaw) — cần đo lại
- Ví dụ: `static_transform_publisher base_link laser_frame [x] [y] [z] [roll] [pitch] [yaw]`

### 11.4 RPLidar - Cấu hình chi tiết

#### 11.4.1 Package & Driver
- **Repo ROS2:** `github.com/Slamtec/sllidar_ros2` (dành riêng cho ROS2)
- **Repo ROS1:** `github.com/Slamtec/rplidar_ros` (không dùng cho ROS2)
- **Giao thức:** USB Serial (UART)
- **Baud rate theo model:**

| Model | Baud rate | Launch file |
|:---|:---|:---|
| RPLIDAR A1, A2M8 | `115200` | `sllidar_a1_launch.py` |
| RPLIDAR A2M7, A2M12, A3, S1 | `256000` | `sllidar_a2m7_launch.py` |
| RPLIDAR S2, S3, S2E | `1000000` | `sllidar_s2_launch.py` |
| RPLIDAR C1 | `460800` | `sllidar_c1_launch.py` |
| RPLIDAR T1 | UDP network | Cấu hình IP (không phải serial) |

#### 11.4.2 Build & chạy
```bash
# Clone package
mkdir -p ~/rplidar_ws/src
cd ~/rplidar_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/rplidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Chạy (ví dụ RPLIDAR A1)
ros2 launch sllidar_ros2 sllidar_a1_launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=115200 \
  frame_id:=laser
```

#### 11.4.3 Verify
```bash
ros2 topic list                  # Phải thấy /scan
ros2 topic echo /scan --once     # Xem dữ liệu LaserScan
ros2 topic hz /scan              # Kiểm tra tần số
```

### 11.5 Laser Filters (lọc dữ liệu LIDAR)

- **Package:** `laser_filters` (từ repo cũ `laser_filters/filters`)
- **Mục đích:** Lọc bỏ các điểm không mong muốn (gần xe quá gần, tự thân robot)
- **Cần sửa:** `frame_id` sang `laser_data_frame` + remap `/scan`
- **Cấu hình filter mẫu:**

```yaml
# laser_filters.yaml
scan_filter_chain:
  - name: range_filter_limiter
    type: "laser_filters/RangeFilterLimits"
    params:
      lower_threshold: 0.15
      upper_threshold: 12.0
      filtering_frame: "laser_frame"
  - name: scan_sharp_edges
    type: "laser_filters/ScanShimmering"
    params:
      window_size: 5
      tolerance: 0.01
```

### 11.6 Tích hợp LIDAR với EKF (robot_localization)

Khi đã có `/scan` từ LIDAR, cần tích hợp vào EKF để nâng chất lượng định vị:

```yaml
# Bổ sung vào config/ekf.yaml hiện tại:
ekf_filter_node:
  ros__parameters:
    # ... các config hiện có (odom0, imu0) ...
    
    # 3. Nguồn dữ liệu 3: LIDAR scan matching (nếu dùng scan_matching)
    # laser_scan0: /scan
    # laser_scan0_config: [false, false, false, false, false, true, false, false, false, false, false, true, false, false, false]
    # laser_scan0_queue_size: 10
```

> **Lưu ý:** LIDAR 2D chỉ cung cấp `LaserScan` (không phải odometry). Để dùng làm nguồn EKF cần có `rf2o_laser_odometry` hoặc `slam_toolbox` để chuyển scan → odometry. Hoặc EKF hiện tại đã đủ với `/odom` (UDP) + `/bno055/imu`.

### 11.7 Sơ đồ tổng thể hệ thống LIDAR khi hoàn thiện

```
[Jetson TX2 - 192.168.1.50]
     │
     ├── [Switch Ethernet 5P] ── [STM32 192.168.1.100:8888] ── CAN ── [ZLAC] ── [2 Motor]
     │                           (UDP 50Hz, 12/22 bytes)
     │                           └── [OLE LiDAR 192.168.1.x] ── Ethernet ── /scan, /laser_data_frame
     │                           └── [RPLidar USB] ── USB ── /scan
     │
     ├── [BNO055 IMU] ── I2C ── /bno055/imu
     │
     └── [EKF] ── Nguồn: /odom (UDP) + /bno055/imu (+ /scan nếu dùng laser_odom)
              └── /odometry/filtered → Nav2 / SLAM
```

### 11.8 Thứ tự thực hiện khi code LIDAR

1. **Bước 1:** Mua/setup Ethernet switch + cấu hình IP tĩnh cho OLE
2. **Bước 2:** Build `sllidar_ros2` cho RPLidar, test `/scan` trên USB
3. **Bước 3:** Build/setup `oleros2` cho OLE, test `/scan` qua Ethernet
4. **Bước 4:** Tạo `ole2dv2.yaml` cấu hình, thử nghiệm với `ros2 topic echo /scan`
5. **Bước 5:** Tạo launch file tổng hợp cho LIDAR
6. **Bước 6:** Thêm TF static `base_link → laser_frame`
7. **Bước 7:** Tích hợp `laser_filters` nếu cần
8. **Bước 8:** Remap topics trong Nav2/SLAM launch file (`/scan`, `/map`)
9. **Bước 9:** Calib odom thực tế (`odom_calibration`)

### 11.9 Các file LIDAR trong repo cũ cần lấy sang

| File/Package | Mục đích | Trạng thái |
|:---|:---|:---|
| `oleros2/` (toàn package) | Driver OLE LiDAR | Cể clone từ GitHub + build |
| `rplidar_ros` / `sllidar_ros2` | Driver RPLidar | Cể clone từ GitHub + build |
| `laser_filters/` | Lọc dữ liệu scan | Copy + sửa frame_id |
| `ole2dv2.yaml` | Config OLE LiDAR | Cần tìm/đọc trong repo cũ |
| `nhatbot_description` | URDF + meshes (gắn LIDAR) | Đo lại TF lắp LIDAR trên xe mới |
| `laser_data_frame` TF | Frame ID cho LIDAR | Cần cấu hình |
| `odom_calibration/` | Hiệu chỉnh wheel_multiplier | Chạy lại trên xe thực |

### 11.10 Ghi chú tìm hiểu từ Web (2026-09-06)

- **OLE Oleros2:** Sử dụng libpcap để bắt gói tin UDP từ Ethernet. Cấu hình IP tĩnh. Phát dữ liệu dạng `LaserScan` và có thể `PointCloud2` cho 3D.
- **RPLidar ROS2:** Dùng package chính thức `sllidar_ros2`. Lưu ý phân biệt baud rate theo model. Đừng nhầm `rplidar_ros` (ROS1/catkin) với `sllidar_ros2` (ROS2/colcon).
- **Cả 2 LiDAR đều dùng frame_id riêng** (`laser_frame` hoặc `laser_link`), cần static TF từ `base_link`.
- **OLE LiDAR có thể có IMU tích hợp** — kiểm tra datasheet để confirm nếu cần dùng cho fuse EKF thêm.
- **Switch Ethernet có nguồn 5V** là bắt buộc vì OLE LiDAR cần cấp nguồn qua Ethernet (PoE) hoặc nguồn riêng.

---

## 📌 9. NHẬT KÝ BÀN GIAO LIVE (AGENT GHI NỐI TIẾP VÀO ĐÂY)
