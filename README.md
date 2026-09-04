# STM32F103 + W5500 + ZLAC8015D Dual-Motor CANopen Gateway

Dự án phát triển mạch điều khiển cầu nối trung gian (Hardware Gateway) thời gian thực giữa máy tính điều khiển cấp cao (Ubuntu / ROS / ROS 2) và Driver động cơ servo vi sai công nghiệp **ZLAC8015D** thông qua mạng Ethernet **W5500 (UDP)** và bus truyền thông công nghiệp **CANopen**.

---

## 📌 1. Kiến trúc hệ thống (System Architecture)

```
[Ubuntu / ROS 2 PC]
       │
       ▼ (Ethernet UDP Socket: 50 Hz, RTT < 0.3 ms)
[W5500 Network Module]
       │
       ▼ (SPI1 @ 4.5 Mbps, Full-Duplex)
[STM32F103C8T6 Gateway]
       │  ├─ Watchdog an toàn mạng (250 ms)
       │  ├─ Kiểm tra toàn vẹn dữ liệu (CRC-16 Modbus)
       │  └─ Kinematics vi sai (v, omega ↔ RPM)
       ▼ (CAN Bus 2.0B @ 500 kbps)
[ZLAC8015D Dual-Channel Servo Driver]
       │
  ┌────┴────┐
  ▼         ▼
[Bánh Trái] [Bánh Phải]
```

- **Tần số điều khiển & phản hồi:** Đồng bộ $50.0\text{ Hz}$ cả 2 chiều ($20\text{ ms}$/chu kỳ).
- **Độ trễ khứ hồi mạng (Round-Trip Latency):** $\approx 0.20 \sim 0.30\text{ ms}$ (200 - 300 micro-giây).
- **Tỉ lệ mất gói (Packet Loss):** $0.0\%$.

---

## 🔌 2. Sơ đồ kết nối phần cứng (Hardware Pinout)

### A. Module Ethernet W5500 ↔ STM32F103 (SPI1)
| Chân W5500 | Chân STM32 | Chức năng | Ghi chú |
| :--- | :--- | :--- | :--- |
| **VCC** | 3.3V | Cấp nguồn | Nguồn sạch 3.3V, tối thiểu 200mA |
| **GND** | GND | Nối đất | Chung mass với toàn hệ thống |
| **SCK** | **PA5** | SPI1 Clock | 4.5 Mbps (Prescaler 16, Mode 0) |
| **MISO** | **PA6** | SPI1 MISO | Master In Slave Out |
| **MOSI** | **PA7** | SPI1 MOSI | Master Out Slave In |
| **CS (SCSn)** | **PA4** | GPIO Output | Chip Select (Active LOW) |
| **RST (RSTn)**| **PA3** | GPIO Output | Hardware Reset (Active LOW) |

> ⚠️ **Lưu ý:** Không dùng các chân PB3, PB4 làm CS/RST vì trên dòng STM32F103 các chân này mặc định gắn với mạch nạp debug JTAG (JTDO, JNTRST), dễ gây xung đột treo chip khi nạp code nếu chưa remap SWD.

### B. Module CAN Transceiver (TJA1050 / VP230) ↔ STM32F103
| Chân Transceiver | Chân STM32 | Chức năng | Ghi chú |
| :--- | :--- | :--- | :--- |
| **VCC** | 5V | Cấp nguồn | TJA1050 yêu cầu nguồn 5V |
| **GND** | GND | Nối đất | Chung mass |
| **CAN_TX** | **PA12** | CAN1 TX | Tốc độ Baudrate: 500 kbps |
| **CAN_RX** | **PA11** | CAN1 RX | Nhận frame từ mạng CAN |
| **CAN_H / CAN_L**| Bus CAN | Cặp dây xoắn | Cần có trở đầu cuối $120\,\Omega$ ở hai đầu bus |

---

## 📦 3. Định dạng gói tin giao tiếp UDP (Packet Protocol)

### A. Gói tin điều khiển từ PC xuống STM32 (12 Bytes - UDP Port 8888)
```
Offset:  0      1      2..5          6..9          10..11
Byte:   [0xAA] [0x55] [v (float)]   [omega (fl)]  [CRC-16 (uint16)]
```
- **Header:** `0xAA 0x55`
- **v:** Vận tốc dài robot ($m/s$, kiểu `float` 32-bit Little-endian).
- **omega:** Vận tốc góc robot ($rad/s$, kiểu `float` 32-bit Little-endian).
- **CRC-16:** Tính theo thuật toán CRC-16/MODBUS cho 10 bytes đầu tiên.

### B. Gói tin Telemetry từ STM32 phản hồi lên PC (16 Bytes - UDP Port 8888)
```
Offset:  0      1      2..3          4..5          6..9         10..13       14..15
Byte:   [0x55] [0xAA] [vel_a(i16)]  [vel_b(i16)]  [pos_a(i32)] [pos_b(i32)] [error(u16)]
```
- **Header:** `0x55 0xAA`
- **vel_a, vel_b:** Vận tốc thực tế của Motor A và Motor B (đơn vị: $0.1\,\text{RPM}$, kiểu `int16_t`).
- **pos_a, pos_b:** Giá trị xung Encoder tích lũy của Motor A và Motor B ($4096\,\text{xung/vòng}$, kiểu `int32_t`).
- **error_code:** Mã lỗi từ Driver (`0` = Bình thường).

---

## 🛠️ 4. Tổng hợp các lỗi thực tế đã gặp & Giải pháp xử lý (Bug Fixes & Lessons Learned)

Trong quá trình phát triển dự án, hệ thống đã gặp và giải quyết triệt để các lỗi kỹ thuật quan trọng sau:

### Lỗi 1: Module W5500 không Ping được, mất kết nối SPI
- **Triệu chứng:** Đèn nguồn và đèn mạng W5500 sáng, nhưng lệnh `ping 192.168.1.100` từ Ubuntu luôn báo `Destination Host Unreachable`.
- **Nguyên nhân cốt lõi:**
  1. STM32CubeMX khi khởi tạo chân GPIO Output (`PA4` - CS) mặc định đặt ở mức **LOW**. Khi W5500 vừa được cấp nguồn mà chân CS đã ở mức LOW sẵn, W5500 sẽ không bao giờ nhìn thấy **sườn xuống (Falling Edge)** để đồng bộ khung truyền SPI đầu tiên.
  2. Hàm đọc ghi SPI trong callback của W5500 lúc đầu dùng `HAL_SPI_Transmit` đơn thuần mà không đọc thanh ghi dữ liệu, làm cờ **Overrun (OVR)** của bộ SPI bị kích hoạt, khóa chặt ngoại vi SPI.
- **Giải pháp:**
  - Kéo chân CS lên mức `HIGH` ngay dòng đầu tiên trước khi kích xung Reset W5500:
    ```c
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
    ```
  - Chuyển toàn bộ callback đọc/ghi SPI sang hàm song công:
    ```c
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 100);
    ```

### Lỗi 2: Biên dịch thiếu hàm thư viện WIZnet `ioLibrary_Driver`
- **Triệu chứng:** Keil C báo lỗi `undefined symbol WIZCHIP_READ / WIZCHIP_WRITE`.
- **Nguyên nhân:** File `wizchip_conf.h` bản mới nhất từ Wiznet GitHub mặc định cấu hình `#define _WIZCHIP_ W6300` thay vì `W5500`.
- **Giải pháp:** Đổi cấu hình trong `wizchip_conf.h` thành:
  ```c
  #define _WIZCHIP_ W5500
  ```

### Lỗi 3: Động cơ quay theo lệnh nhưng Feedback vận tốc và Encoder luôn bằng 0
- **Triệu chứng:** Xe chạy theo lệnh từ Python, nhưng dữ liệu trả về máy tính luôn là `Vel: 0.0 RPM | Enc: 0`.
- **Nguyên nhân:**
  - Đối chiếu tài liệu gốc `ZLAC8015D CANopen Communication Quick Start Guide` (dòng 1406-1514): Mặc định xuất xưởng từ nhà máy, Driver ZLAC **tắt toàn bộ TPDO** (`Number of mapped objects = 0`, `Event Timer = 0`).
  - Code khởi động trong `zlac_can.c` trước đây chỉ cấu hình nhận lệnh (RPDO0, RPDO1) mà **quên viết hàm cấu hình gửi dữ liệu (TPDO)**. Driver không hề biết cần gửi thông số nào và gửi khi nào, nên hoàn toàn im lặng.
- **Giải pháp:**
  - Viết 2 hàm cấu hình TPDO chuẩn theo tài liệu hãng:
    - `_TPDO0_Config(id)`: Ánh xạ Vận tốc Motor A & B (`0x606C sub 01 & 02`, 32-bit mỗi motor) về COB-ID `0x181`, chu kỳ Timer $20\,\text{ms}$ (`0x28`).
    - `_TPDO2_Config(id)`: Ánh xạ Vị trí Encoder Motor A & B (`0x6064 sub 01 & 02`, 32-bit mỗi motor) về COB-ID `0x381`, chu kỳ Timer $20\,\text{ms}$ (`0x28`).
  - Gọi 2 hàm này trong trạng thái `ZLAC_CONFIG` của máy trạng thái CANopen.

### Lỗi 4: Động cơ quay nhanh gấp 10 lần vận tốc đặt trong Python
- **Triệu chứng:** Đặt $v = 0.05\,\text{m/s}$ ($\approx 5.96\,\text{RPM}$) nhưng bánh xe thực tế quay tới $59.5\,\text{RPM}$ ($0.5\,\text{m/s}$).
- **Nguyên nhân:**
  - Trong tài liệu gốc ZLAC (dòng 2909 & 2926): Thanh ghi cài đặt tốc độ mục tiêu `0x60FF` có đơn vị là **`1 RPM`** (vòng/phút nguyên bản).
  - Code cũ lầm tưởng đơn vị là $0.1\,\text{RPM}$ nên đã nhân thừa $10.0$ (`int16_t zlac_L = (int16_t)(rpm_L * 10.0f);`).
- **Giải pháp:** Bỏ việc nhân 10 trong hàm `ZLAC_SetSpeed_mps`:
  ```c
  int16_t zlac_L = (int16_t)roundf(rpm_L);
  int16_t zlac_R = (int16_t)roundf(rpm_R);
  ```

### Lỗi 5: Xe đột ngột đứng im khi tích hợp thuật toán an toàn
- **Triệu chứng:** Mạng UDP vẫn có RX/TX, nhưng xe không chịu nhúc nhích.
- **Nguyên nhân:**
  - STM32 được nâng cấp kiểm tra **CRC-16 Modbus** (`Calculate_CRC16`).
  - Phía Python ban đầu lại gửi mã **Checksum cộng dồn đơn giản** (`sum(payload)`).
  - Khi mã kiểm tra không khớp, STM32 từ chối nhận lệnh và kích hoạt cơ chế **Watchdog an toàn 250ms**, ngắt hoàn toàn nguồn cấp cho motor.
- **Giải pháp:** Đồng bộ thuật toán CRC-16 Modbus ở cả 2 đầu:
  ```python
  def calculate_crc16(data: bytes) -> int:
      crc = 0xFFFF
      for byte in data:
          crc ^= byte
          for _ in range(8):
              crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
      return crc
  ```

### Lỗi 6: Tần số nhận phản hồi bị tụt nhẹ từ 50Hz về 47.4Hz
- **Triệu chứng:** Tần số gửi đạt 49.6Hz nhưng nhận về chỉ đạt ~47.4Hz.
- **Nguyên nhân:** Phép tính $1000\,\text{ms} / 21\,\text{ms} = 47.6\,\text{Hz}$. Do có lệnh `HAL_Delay(5)` ở đáy vòng lặp `while (1)` cộng với thời gian xử lý SDO của CAN bus, chu kỳ kiểm tra thời gian bị trễ đúng $1\,\text{ms}$ ($21\,\text{ms}$ thay vì $20\,\text{ms}$).
- **Giải pháp:** Giảm `HAL_Delay(5)` ở cuối vòng lặp `while (1)` của `main.c` xuống `HAL_Delay(1)`.

---

## 🚀 5. Hướng dẫn chạy thử nghiệm (Quickstart)

### 1. Nạp code STM32:
- Mở project bằng Keil MDK-ARM: `MDK-ARM/Can_test_motor.uvprojx`.
- Bấm **F7** (Build), sau đó bấm **F8** (Download) nạp vào STM32 qua ST-Link.
- Khởi động lại nguồn toàn bộ hệ thống (ZLAC8015D + STM32).

### 2. Chạy test trên máy tính Ubuntu:
- Cắm dây mạng LAN nối trực tiếp giữa máy tính Ubuntu và module W5500.
- Cấu hình IP tĩnh trên cổng mạng Ubuntu:
  - **IP:** `192.168.1.10`
  - **Netmask:** `255.255.255.0`
  - **Gateway:** `192.168.1.1`
- Chạy script đo đạc kiểm tra:
  ```bash
  python3 scripts/test_robot_telemetry.py
  ```
- Quan sát các thông số Vận tốc, Quãng đường Odometry và Tần số $50\,\text{Hz}$ hiển thị mượt mà trên Terminal. Nhấn `Ctrl + C` để dừng xe an toàn.


---

## ⏱️ 6. Kiến trúc định thời tối ưu: Hardware Timer Interrupt + Flag Pattern (50Hz Real-Time)

### A. Vấn đề cốt lõi: Vì sao không gọi W5500 & CAN trực tiếp trong hàm ngắt (ISR)?
Trong lập trình vi điều khiển, việc đưa toàn bộ tác vụ truyền thông vào trong hàm ngắt của Timer (`TIMx_IRQHandler` hoặc `HAL_TIM_PeriodElapsedCallback`) là một lỗi kiến trúc nghiêm trọng:
1. **Tranh chấp ngoại vi SPI (Re-entrancy / Resource Corruption):**
   - Vòng lặp chính `while(1)` có thể đang trong tiến trình giao tiếp với W5500 (`recvfrom`, kiểm tra thanh ghi SPI, kéo chân CS xuống `LOW`).
   - Nếu ngắt Timer xảy ra đột ngột và gọi `sendto()`, hàm ngắt sẽ can thiệp vào chân CS và thanh ghi SPI1, làm vỡ khung truyền SPI và gây kẹt bus SPI vĩnh viễn.
2. **Nguy cơ Deadlock với `SysTick`:**
   - Thư viện STM32 HAL và WIZnet sử dụng `HAL_GetTick()` để kiểm tra Timeout cho các tác vụ SPI/CAN.
   - `HAL_GetTick()` tăng thông qua ngắt `SysTick`. Nếu ngắt Timer có độ ưu tiên cao hơn hoặc bằng `SysTick`, `SysTick` sẽ bị chặn trong lúc hàm ngắt Timer đang chạy. Nếu hàm giao tiếp SPI gặp sự cố chờ, Timeout sẽ không bao giờ đếm được -> **Vi điều khiển bị treo cứng hoàn toàn**.
3. **Quy tắc vàng của lập trình nhúng:**
   > *"Hàm ngắt (ISR) chỉ thực hiện các tác vụ siêu ngắn (vài nano/micro-giây) rồi thoát ngay. Tuyệt đối không gọi các hàm truyền thông thời gian dài (SPI, CAN, UART, Ethernet) bên trong ISR."*

### B. Giải pháp chuẩn công nghiệp: "Timer ngắt dựng Cờ" (Interrupt-driven Flag Pattern)
Tách rời hoàn toàn **bộ định thời phần cứng chính xác tuyệt đối** khỏi **tác vụ truyền thông ngoại vi**:
- **Hardware Timer (TIM2):** Chạy độc lập ở tầng phần cứng, cứ đúng $20.00\,	ext{ms}$ phát xung ngắt cực ngắn ($< 0.05\,\mu	ext{s}$) chỉ để bật một cờ hiệu: `flag_telemetry_50hz = 1`.
- **Vòng lặp `while(1)`:** Chạy liên tục kiểm tra cờ. Khi thấy `flag_telemetry_50hz == 1`, nó xóa cờ và tiến hành gửi SYNC CAN + bắn UDP W5500.

### C. Hướng dẫn tính toán & cấu hình chi tiết (STM32F103 @ 72MHz)

#### 1. Công thức tính Prescaler và Period:
- Xung nhịp hệ thống: $f_{\text{CLK}} = 72\,	ext{MHz} = 72.000.000\,	ext{Hz}$.
- Chọn bộ chia tần số Timer (Prescaler - PSC) sao cho đồng hồ đếm Timer nhảy mỗi $0.1\,	ext{ms}$ ($10\,	ext{kHz}$):
  $$\text{PSC} = \frac{72.000.000}{10.000} - 1 = 7199$$
- Chu kỳ ngắt mong muốn là $20\,	ext{ms}$ ($50\,	ext{Hz}$):
  $$\text{ARR} = \frac{20\,	ext{ms}}{0.1\,	ext{ms}} - 1 = 199$$

#### 2. Khởi tạo ngắt Timer trong `main.c`:
```c
/* Khởi động Timer 2 ở chế độ ngắt */
HAL_TIM_Base_Start_IT(&htim2);
```

#### 3. Hàm phục vụ ngắt (ISR Callback):
```c
volatile uint8_t flag_telemetry_50hz = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        flag_telemetry_50hz = 1; // Chỉ bật cờ hiệu, tốn đúng 1 chu kỳ lệnh!
    }
}
```

#### 4. Điều phối trong vòng lặp chính `while(1)`:
```c
while (1)
{
    ZLAC_StateMachine();

    // 1. Nhận và giải mã lệnh điều khiển UDP (nếu có)
    if (getSn_RX_RSR(0) >= sizeof(UDP_ControlPacket_t))
    {
        // Nhận gói tin, kiểm tra CRC-16, set vận tốc ZLAC
    }

    // 2. Watchdog an toàn 250ms
    if (HAL_GetTick() - last_udp_rx_time > 250)
    {
        ZLAC_Stop();
    }

    // 3. Đúng nhịp 20.00ms được kích hoạt bởi ngắt TIM2
    if (flag_telemetry_50hz)
    {
        flag_telemetry_50hz = 0; // Xóa cờ ngay lập tức

        // Phát lệnh SYNC CAN
        HAL_CAN_AddTxMessage(&hcan, &sync_hdr, NULL, &sync_mailbox);

        // Bắn Telemetry UDP lên PC
        sendto(0, (uint8_t*)&fb_pkt, sizeof(UDP_FeedbackPacket_t), remote_ip, remote_port);
    }

    // Hoàn toàn không cần HAL_Delay()! Vòng lặp phản hồi tức thì và không bị nghẽn.
}
```

### D. Hiệu quả đạt được:
- **Độ ổn định tần số:** Triệt tiêu hoàn toàn Jitter và lỗi lượng tử hóa bước thời gian; tần số cố định chính xác $50.00\,	ext{Hz}$.
- **Độ an toàn:** Không bao giờ xảy ra xung đột tài nguyên giữa các ngoại vi SPI và CAN, không gây deadlock với SysTick.
