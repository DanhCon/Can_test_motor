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

## 📁 2. CẤU TRÚC CODEBASE & TRÁCH NHIỆM TỪNG FILE

```
Can_test_motor/
├── Core/
│   ├── Inc/
│   │   ├── zlac_can.h            # Khai báo cấu trúc, macro, hằng số cơ khí, mã lỗi CANopen
│   │   └── main.h                # Khai báo chân GPIO, SPI1, CAN1
│   └── Src/
│       ├── main.c                # Vòng lặp chính: Đọc/ghi W5500, Watchdog 250ms, gửi Telemetry 50Hz
│       └── zlac_can.c            # CANopen State Machine, PDO mapping, SDO Write/Read, Kinematics
├── scripts/
│   └── zlac_udp_odom_node.py     # Node ROS 2: UDP Client, Velocity Smoother, Breakaway Kick, Odometry, TF
├── launch/
│   └── teleop.launch.py          # Launch file chạy chung: joy_node, teleop_joy, zlac_udp_odom_node
├── AGENT_HANDOVER_GUIDE.md       # Tài liệu này (Hướng dẫn bàn giao cho AI Agent)
└── README.md                     # Hướng dẫn đấu dây phần cứng và tài liệu kỹ thuật
```

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
