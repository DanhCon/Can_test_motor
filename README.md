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

### B. Gói tin Telemetry từ STM32 phản hồi lên PC (22 Bytes - UDP Port 8888)
```
Offset:  0      1      2..3          4..5          6..9         10..13       14..15       16..17        18..19        20..21
Byte:   [0x55] [0xAA] [vel_a(i16)]  [vel_b(i16)]  [pos_a(i32)] [pos_b(i32)] [error(u16)] [cur_a(i16)]  [cur_b(i16)]  [volt(u16)]
```
- **Header:** `0x55 0xAA` (2 bytes nhận diện bắt đầu gói tin).
- **vel_a, vel_b:** Vận tốc thực tế của Motor A và Motor B (đơn vị: $0.1\,\text{RPM}$, kiểu `int16_t`).
- **pos_a, pos_b:** Giá trị xung Encoder tích lũy của Motor A và Motor B ($4096\,\text{xung/vòng}$, kiểu `int32_t`).
- **error_code:** Mã lỗi hệ thống (kiểu `uint16_t`, `0` = Bình thường, `0xEEEE` = Kích hoạt bảo vệ kẹt tải hoặc quá dòng).
- **cur_a, cur_b:** Dòng điện thực tế của Motor A và Motor B (đơn vị: $0.1\,\text{A}$, kiểu `int16_t`).
- **volt:** Điện áp DC Bus / Pin (đơn vị: $0.1\,\text{V}$, kiểu `uint16_t`, đọc qua SDO `0x2035` mỗi 1 giây).

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


---

## 📚 7. Lý Thuyết Hệ Thống, Luồng Biến Đổi Dữ Liệu & Bảng Tra Cứu Hàm Toàn Diện

### A. Tổng quát hệ thống đang làm gì?
Hệ thống là một **Cầu nối phần cứng thời gian thực (Hard Real-Time Embedded Gateway)** giữa:
- **Tầng điều khiển cấp cao (High-level Control):** Máy tính PC (Ubuntu / ROS / ROS 2 / Python) đảm nhiệm định vị, quy hoạch quỹ đạo và tính toán vận tốc xe $(v, \omega)$.
- **Tầng truyền động chấp hành (Actuator Level):** Bộ điều khiển động cơ servo vi sai kép công nghiệp **ZLAC8015D** kéo 2 bánh xe AGV/AMR qua bus **CANopen 500kbps**.
- **Tầng trung gian nhúng (Embedded Gateway):** Vi điều khiển **STM32F103C8T6** kết hợp chip Ethernet phần cứng **W5500**, chịu trách nhiệm:
  1. Giải mã gói tin UDP, kiểm tra tính toàn vẹn CRC-16 Modbus.
  2. Phân giải động học vi sai nghịch chuyển $(v, \omega)$ thành vòng tua 2 bánh $RPM_L, RPM_R$ (đơn vị: $1\,	ext{RPM}$).
  3. Kích hoạt và điều khiển servo theo chuẩn quốc tế **CiA 402**.
  4. Thu thập dữ liệu phản hồi (vận tốc, encoder, dòng điện, điện áp pin) qua TPDO và SDO với chu kỳ $20\,	ext{ms}$.
  5. Bảo vệ an toàn đa tầng: Watchdog mất mạng 250ms, bảo vệ kẹt tải & quá dòng (ngắt tức thời $>12A$ hoặc $>6A$ kéo dài $>400ms$ gán mã lỗi `0xEEEE`, tự phục hồi sau 2s).
  6. Phát Telemetry định kỳ $50\,	ext{Hz}$ ($20\,	ext{ms}$) chuẩn 22 bytes lên PC.

---

### B. Dữ liệu thông tin đi qua những thứ gì và biến đổi từ dạng gì sang dạng gì?

```
========================================================================================================
                                     CHIỀU ĐIỀU KHIỂN (DOWNLINK)
========================================================================================================
[PC Python/ROS]             v, omega (float: m/s, rad/s)
      │
      ▼ (struct.pack '<BBff')
[RAM PC / Socket]           Gói nhị phân 12B: [0xAA, 0x55, v, omega, CRC-16]
      │
      ▼ (Cáp RJ45)
[Biến áp xung & W5500]      Tín hiệu vi sai 100BASE-TX TX+/TX- (±1V) ──> Byte trong W5500 RX Buffer
      │
      ▼ (SPI1 @ 4.5MHz)
[STM32F103 RAM]             Đọc Burst SPI ──> Struct UDP_ControlPacket_t ──> Kiểm tra CRC-16
      │
      ▼ (Kinematics)
[Toán học STM32]            v_L = v - omega*L/2, v_R = v + omega*L/2 ──> RPM_L, RPM_R (int16_t: 1 RPM)
      │
      ▼ (bxCAN & TJA1050)
[Bus CAN 500kbps]           Frame CAN 2.0B COB-ID 0x601 (SDO) hoặc 0x201 (RPDO0) (V_CAN_H - V_CAN_L)
      │
      ▼ (CAN Controller)
[Driver ZLAC8015D]          DSP giải mã SDO/PDO ──> Vòng lặp PID vận tốc 10kHz
      │
      ▼ (Cầu H MOSFET)
[Động cơ BLDC/PMSM]         Nghịch lưu PWM 3 pha (U, V, W) 24V-48V ──> Từ trường quay kéo Rotor quay cơ học!

========================================================================================================
                                     CHIỀU PHẢN HỒI (UPLINK)
========================================================================================================
[Đĩa từ Encoder 4096 xung]  Chuyển động quay bánh xe ──> Xung vuông 2 pha A/B
[Shunt & Phân áp Driver]    Dòng tải motor & Điện áp nguồn DC Bus
      │
      ▼ (Đo lường & CANopen)
[Driver ZLAC8015D]          Đóng 4 Frame CAN:
                            • TPDO0 (0x181): Vận tốc Motor A & B (đơn vị: 0.1 RPM)
                            • TPDO1 (0x281): Dòng điện Motor A & B (đơn vị: 0.1 A)
                            • TPDO2 (0x381): Tọa độ xung tích lũy Encoder (4096 xung/vòng)
                            • SDO Resp (0x581): Điện áp DC Bus / Pin (đơn vị: 0.1 V, chu kỳ 1Hz)
      │
      ▼ (Bus CAN 2 dây xoắn)
[Transceiver ──> PA11]      Tín hiệu điện vi sai ──> Ngắt phần cứng HAL_CAN_RxFifo0MsgPendingCallback()
      │
      ▼ (Xử lý STM32)
[RAM STM32F103]             • Lưu vào zlac_fb
                            • Kiểm tra kẹt tải / quá dòng: I > 12A hoặc (I > 6A & RPM < 3 quá 400ms) ──> 0xEEEE
                            • Tích phân Odometry (x, y, theta)
      │
      ▼ (Đóng gói nhịp 20ms)
[Struct 22 Bytes]           [0x55, 0xAA | vel_a, vel_b | pos_a, pos_b | error_code | cur_a, cur_b | volt]
      │
      ▼ (SPI1 ──> W5500)
[Cáp Ethernet RJ45]         Bắn gói UDP qua Socket 0 Port 8888 ──> PC
      │
      ▼ (Python unpack)
[Màn hình Terminal / ROS]   struct.unpack('<BBhhllHhhH') ──> Hiển thị RPM, Dòng A/B, Điện áp Pin, Cảnh báo 0xEEEE
```

---

### C. Bảng tra cứu & Phân tích toàn bộ các hàm trong `main.c`

| Tên hàm | Hoạt động khái quát | Tín hiệu đầu vào (Inputs) | Tín hiệu đầu ra (Outputs) | Tác động phần cứng |
| :--- | :--- | :--- | :--- | :--- |
| `main` | Hàm chính khởi chạy toàn bộ vi điều khiển, mạng W5500, CAN ZLAC và vòng lặp `while(1)`. | Không có (chạy từ Reset). | Không bao giờ return. | Điều phối toàn bộ phần cứng. |
| `Calculate_CRC16` | Tính mã kiểm tra toàn vẹn CRC-16 Modbus (Đa thức `0xA001`, Init `0xFFFF`). | `data`: Con trỏ mảng byte.<br>`length`: Số byte (10 bytes). | `uint16_t`: Mã CRC 16-bit. | Tính toán CPU thuần túy. |
| `W5500_Select` | Kéo chân CS xuống LOW để chọn chip W5500 mở phiên truyền SPI. | Không có. | `void`. | PA4 xuất mức LOW (`0V`). |
| `W5500_Deselect` | Kéo chân CS lên HIGH để kết thúc phiên truyền SPI. | Không có. | `void`. | PA4 xuất mức HIGH (`3.3V`). |
| `W5500_WriteByte` | Ghi 1 byte qua SPI1 song công (`HAL_SPI_TransmitReceive`). | `byte`: Byte dữ liệu cần ghi. | `void`. | Phát xung clock SCK (PA5) và dữ liệu MOSI (PA7). |
| `W5500_ReadByte` | Đọc 1 byte từ W5500 bằng cách phát byte Dummy `0xFF`. | Không có. | `uint8_t`: Byte đọc từ MISO. | Lấy mẫu tín hiệu trên chân PA6 (MISO). |
| `W5500_ReadBurst` | Đọc khối dữ liệu liên tục `len` bytes vào RAM STM32. | `pBuf`: Bộ đệm nhận.<br>`len`: Số lượng byte. | Ghi vào `pBuf`. | Đọc dòng dữ liệu burst tốc độ cao qua SPI1. |
| `W5500_WriteBurst`| Ghi khối dữ liệu liên tục `len` bytes từ RAM xuống W5500. | `pBuf`: Bộ đệm nguồn.<br>`len`: Số lượng byte. | `void`. | Ghi dòng dữ liệu burst tốc độ cao qua SPI1. |
| `SystemClock_Config` | Cấu hình dao động HSE 8MHz qua PLL x9 đạt 72MHz max speed. | Thạch anh 8MHz ngoài. | `void`. | Cấp xung toàn hệ thống. |
| `Error_Handler` | Khóa CPU trong vòng lặp vô hạn khi có ngoại vi khởi tạo thất bại. | Không có. | Không return. | Tắt ngắt toàn cục (`__disable_irq`). |
| `assert_failed` | Báo cáo tên file và dòng code bị lỗi assertion thư viện HAL. | `file`: Tên file.<br>`line`: Số dòng. | `void`. | Hỗ trợ debug phần mềm. |

---

### D. Bảng tra cứu & Phân tích toàn bộ các hàm trong `zlac_can.c`

| Tên hàm | Hoạt động khái quát | Tín hiệu đầu vào (Inputs) | Tín hiệu đầu ra (Outputs) | CAN Frame / Tác động |
| :--- | :--- | :--- | :--- | :--- |
| `_CAN_Send` | Gửi 1 frame CAN 11-bit ID tiêu chuẩn thông qua Mailbox CAN1. | `std_id`: COB-ID.<br>`dlc`: Độ dài (0..8).<br>`data`: Con trỏ mảng byte. | `HAL_StatusTypeDef` (`HAL_OK` / `HAL_ERROR`). | Phát xung điện vi sai ra PA11/PA12 (500 kbps). |
| `SDO_Write` | Ghi thông số vào Object Dictionary (giao thức SDO Expedited). | `node_id` (0x01).<br>`index` (16-bit).<br>`sub` (8-bit).<br>`value` (32-bit int).<br>`size_bytes` (1/2/4). | `void` (chờ 5ms). | CAN ID `0x600 + id` (DLC=8), Byte 0 = `0x2F/0x2B/0x23`. |
| `SDO_Read_Request` | Yêu cầu Driver đọc giá trị 1 thanh ghi (đọc áp pin `0x2035:00`). | `node_id`, `index`, `sub`. | `void`. | CAN ID `0x600 + id` (DLC=8), Byte 0 = `0x40`. |
| `NMT_Send` | Gửi lệnh quản lý mạng NMT chuyển trạng thái vận hành CANopen. | `node_id` (0=all, 1=node 1).<br>`cmd` (0x01, 0x80, 0x81). | `void`. | CAN ID `0x000` (DLC=2), Byte 0 = `cmd`, Byte 1 = `node_id`. |
| `_RPDO0_Config` | Cấu hình nhận lệnh RPDO0: Ánh xạ Controlword `0x6040:00` vào `0x201`. | `id`: ID của Driver (0x01). | Ghi SDO cấu hình. | Ánh xạ nhận lệnh servo tức thời. |
| `_RPDO1_Config` | Cấu hình nhận lệnh RPDO1: Ánh xạ Target Velocity `0x60FF:03` vào `0x301`. | `id`: ID của Driver (0x01). | Ghi SDO cấu hình. | Ánh xạ nhận lệnh vận tốc 2 bánh. |
| `_TPDO0_Config` | Cấu hình phát TPDO0: Ánh xạ Actual Velocity `0x606C:01 & 02` về `0x181` (20ms). | `id`: ID của Driver (0x01). | Ghi SDO cấu hình. | Driver tự phát tốc độ 2 motor mỗi 20ms. |
| `_TPDO1_Config` | Cấu hình phát TPDO1: Ánh xạ Actual Current `0x6077:03` về `0x281` (20ms). | `id`: ID của Driver (0x01). | Ghi SDO cấu hình. | Driver tự phát dòng điện 2 motor mỗi 20ms. |
| `_TPDO2_Config` | Cấu hình phát TPDO2: Ánh xạ Actual Position `0x6064:01 & 02` về `0x381` (20ms). | `id`: ID của Driver (0x01). | Ghi SDO cấu hình. | Driver tự phát số xung Encoder mỗi 20ms. |
| `_ProfileVelocity_Init` | Cài đặt Mode 3 (Profile Velocity) và gia tốc tăng/giảm tốc 200ms. | `id`: ID của Driver (0x01). | Ghi liên tiếp 5 lệnh SDO. | Động cơ tăng tốc êm, chống giật cơ khí. |
| `_ZLAC_EnableServo` | Kích hoạt Servo CiA 402: Shutdown (0x06) &rarr; Switch On (0x07) &rarr; Enable (0x0F). | `id`: ID của Driver (0x01). | Phát 3 frame qua RPDO0. | Đóng relay cấp lực cho motor ("tách"). |
| `ZLAC_CAN_Init` | Cấu hình bộ lọc CAN mở toàn bộ, kích hoạt ngắt FIFO0, bật CAN1. | Không có. | `void`. | Ngoại vi CAN chuyển sang Normal mode. |
| `ZLAC_StateMachine` | Máy trạng thái phi chặn điều phối khởi động, vận hành và bảo vệ kẹt tải. | Gọi liên tục trong `while(1)`. | Cập nhật `zlac_state` và `error_code`. | Ngắt xung an toàn khi dòng cao, tự phục hồi sau 2s. |
| `ZLAC_IsReady` | Kiểm tra Driver đã hoàn tất khởi động và sẵn sàng nhận lệnh chưa. | Không có. | `bool` (`true` nếu sẵn sàng). | Bảo đảm an toàn không gửi lệnh khi servo chưa bật. |
| `ZLAC_SetSpeed_mps` | Hàm điều khiển chính: Động học vi sai, kẹp $V_{\max}$, gửi tốc độ 2 bánh. | `v` (m/s, float), `omega` (rad/s, float). | Gửi SDO `0x60FF` (đơn vị: **1 RPM**). | Điều khiển trực tiếp 2 bánh xe di chuyển. |
| `ZLAC_SetSpeed_raw` | Đặt trực tiếp tốc độ vòng/phút cho từng motor bằng giá trị RPM nguyên bản. | `vel_a`, `vel_b` (đơn vị: 1 RPM). | Gửi 2 frame SDO `0x60FF`. | Chạy động cơ theo số vòng/phút chỉ định. |
| `ZLAC_Stop` | Dừng khẩn cấp: Đặt tốc độ cả 2 motor bằng 0 RPM ngay lập tức. | Không có. | Gửi 0 RPM xuống 2 motor. | Triệt tiêu lực đẩy, dừng xe an toàn. |
| `ZLAC_GetVelA_mps` | Đọc vận tốc thực tế của bánh xe Trái (A) theo mét/giây. | Không có. | `float` ($m/s$). | Phục vụ tính toán Odometry và PID ngoài. |
| `ZLAC_GetVelB_mps` | Đọc vận tốc thực tế của bánh xe Phải (B) theo mét/giây. | Không có. | `float` ($m/s$). | Phục vụ tính toán Odometry và PID ngoài. |
| `ZLAC_Odom_Update` | Cập nhật tích phân tọa độ Dead-Reckoning $(x, y, 	heta)$ sau khoảng $dt$ giây. | `dt`: Thời gian trôi qua (giây). | Cập nhật `zlac_odom` ($x, y, 	heta$). | Cung cấp tọa độ mặt phẳng 2D cho robot. |
| `ZLAC_Odom_Reset` | Reset toàn bộ tọa độ Odometry về mốc gốc $(0, 0, 0)$. | Không có. | `x=0, y=0, theta=0`. | Thiết lập lại mốc tọa độ ban đầu. |
| `HAL_CAN_RxFifo0MsgPendingCallback` | **Hàm ngắt phần cứng nhận CAN (ISR).** Phân loại CAN ID và trích xuất dữ liệu. | `hcan`: Con trỏ CAN HAL. | Cập nhật tức thì `zlac_fb`. | • 0x701: Bootup.<br>• 0x181: Vận tốc.<br>• 0x281: Dòng điện.<br>• 0x381: Encoder.<br>• 0x581: Điện áp Pin. |

---

## 8. VẬN HÀNH ROS 2 TELEOP & TỐI ƯU HÓA CHUYỂN ĐỘNG ROBOT

### 8.1 Sơ đồ ánh xạ tay cầm PS4 (DualShock 4 Bluetooth)
Hệ thống sử dụng file launch `launch/teleop.launch.py` kết nối trực tiếp tay cầm Bluetooth (`/dev/input/js0`):
* **Cần TRÁI (Axis 1 - Lên / Xuống):** Điều khiển Vận tốc tiến / lùi ($v = \pm 0.8\,\text{m/s}$).
* **Cần PHẢI (Axis 2/3 - Trái / Phải):** Bẻ lái quay góc ($\omega = \pm 0.8\,\text{rad/s}$).
* **Nút L1 (Button 9):** Cò an toàn Deadman Switch (Bắt buộc giữ L1 để cấp lực chạy xe; buông tay là xe dừng ngay).
* **Nút R1 (Button 5):** Turbo Boost (Tăng tốc độ lên $1.2\,\text{m/s}$).
* **Nút B / Tròn (Button 1):** Phanh dừng khẩn cấp E-Stop.
* **Nút Y / Tam giác (Button 3):** Reset mốc tọa độ Odometry về $(0, 0, 0)$.

### 8.2 Cấu hình gia tốc chống giật (Acceleration Profile Tuning)
1. **Trên Driver ZLAC8015D (`Core/Src/zlac_can.c`):**
   * Object `0x6083:01 & 02` (Profile Acceleration): Cài đặt **`700ms`** (Tăng tốc đầm chắc, giảm dòng xung kích).
   * Object `0x6084:01 & 02` (Profile Deceleration): Cài đặt **`900ms`** (Hãm dừng êm ái, chống lật xe và giật quán tính khi buông cần).
2. **Trên ROS 2 Gateway (`scripts/zlac_udp_odom_node.py`):**
   * `linear_accel = 0.8 m/s²`, `angular_accel = 1.2 rad/s²` (Làm mượt vận tốc tuyến tính ở tần số 50Hz).
   * `min_breakaway_velocity = 0.04 m/s` (Breakaway Kick: Cung cấp lực tức thời ~7.1 RPM để bứt phá ma sát tĩnh ở vạch xuất phát).

### 8.3 Ghi chú hiện tượng ma sát tĩnh (Breakaway Stiction)
Do 2 động cơ Hub Motor lắp quay mặt vào nhau ($180^\circ$), khi xe tiến/lùi, một motor quay chiều Thuận và motor kia quay chiều Nghịch. Chiều quay Nghịch của động cơ BLDC có mô-men khởi động nhỉnh hơn chiều Thuận ở dải vận tốc siêu chậm ($< 3\,\text{RPM}$), dẫn đến độ lệch nhẹ khi vừa xuất phát rồi cả 2 bánh cùng chuyển động đồng bộ bình thường.

