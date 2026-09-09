# SỔ TAY KỸ THUẬT: CHẨN ĐOÁN & KHẮC PHỤC SỰ CỐ ROBOT AMR
**Hệ thống:** Jetson TX2 (Ubuntu 22.04, ROS 2 Humble) + STM32 W5500 + 2x ZLAC8015D + OLE LiDAR + BNO055
**Kiến trúc hiện tại:** ros2_control (`ZlacHardwareInterface` C++) + `diff_drive_controller` + `twist_mux` + EKF 20Hz + AMCL + Nav2 (MPPI)
**Launch chính:** `ros2 launch can_test_motor bringup_all.launch.py` (full) hoặc `robot.launch.py` (phần cứng)

> Quy ước mỗi mục: **Hiện tượng** → **Tác hại** → **Nguyên nhân** → **Cách xử lý**.
> Các mục đánh dấu [ĐÃ SỬA] là lỗi quá khứ, giữ lại để không tái phạm.

---

## 1. SƠ ĐỒ ĐỊA CHỈ IP & CỔNG MẠNG CHUẨN
| Thiết bị | Địa chỉ IP | Giao thức / Port | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Jetson TX2 (PC)** | `192.168.1.10` | - | Cổng mạng `eth0`, đặt IP tĩnh cùng subnet |
| **STM32 + W5500** | `192.168.1.100` | UDP `8888` | Điều khiển 2 động cơ ZLAC8015D qua CANopen |
| **OLE LiDAR 2D** | `192.168.1.101` | UDP `2368` | Cảm biến Laser 2D quét môi trường (15 Hz) |

> ⚠️ Một số tài liệu cũ ghi Jetson là `192.168.1.50`. Chuẩn hiện tại là **`192.168.1.10`** (khớp Host Destination của OLE). Nếu đổi IP máy, phải đổi đồng bộ Host Destination trên Web GUI OLE.

---

## 2. NHÓM LỖI KHỞI ĐỘNG & TÀI NGUYÊN (BLOCKING)

### 🚨 Lỗi 1: Cổng UDP 8888 bị chiếm dụng (`Address already in use`)
* **Hiện tượng:** Node phần cứng hoặc script báo:
  ```text
  OSError: [Errno 98] Address already in use
  ```
* **Tác hại:** `ros2_control_node` không bind được socket → hardware interface `on_activate` FAILED → toàn bộ controller không lên, xe liệt hoàn toàn.
* **Nguyên nhân:** Nhấn `Ctrl+C` tiến trình cũ (`ros2_control_node`, script test) chưa giải phóng socket; hoặc chạy nhầm 2 launch cùng lúc (`robot.launch.py` + `launch/legacy/teleop*.launch.py` / `test_robot_telemetry.py` đều bind 8888).
* **Cách khắc phục:**
  ```bash
  # Tìm và diệt đúng PID đang giữ port (đọc trực tiếp kernel, chính xác hơn lsof):
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
  * **Phòng ngừa:** Không bao giờ chạy `launch/legacy/*` hay `test_robot_telemetry.py` khi robot đang chạy (script test đã tự bắt lỗi và in hướng dẫn).

### 🚨 Lỗi 2: Spawner timeout khi khởi động (`Failed to contact service /controller_manager`)
* **Hiện tượng:** `spawner` báo timeout, `diff_drive_controller` ở trạng thái `unconfigured`, xe không nhận lệnh.
* **Tác hại:** Controller không active → không có `/diff_drive_controller/odom`, twist_mux phát lệnh vào hư không.
* **Nguyên nhân:** Hai spawner chạy song song, `diff_drive_controller` gọi service khi `controller_manager` chưa sẵn sàng (đặc biệt sau khi dọn SHM hoặc CPU bận).
* **Cách xử lý:** `robot.launch.py` đã cấu hình khởi động tuần tự: `TimerAction(3s)` → `joint_state_broadcaster` → `OnProcessExit` → `diff_drive_controller`, timeout 30s. Nếu vẫn lỗi: kiểm tra `ros2 control list_controllers` rồi spawn tay:
  ```bash
  ros2 run controller_manager spawner joint_state_broadcaster
  ros2 run controller_manager spawner diff_drive_controller
  ```

### 🚨 Lỗi 3: Kẹt bộ nhớ chia sẻ FastDDS (`RTPS_TRANSPORT_SHM Error`)
* **Hiện tượng:**
  ```text
  [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portXXXX: open_and_lock_file failed
  ```
* **Tác hại:** Node không khởi tạo được transport → không pub/sub được gì.
* **Nguyên nhân:** FastDDS để lại file lock rác trong `/dev/shm` khi bị tắt đột ngột.
* **Cách khắc phục:** `rm -rf /dev/shm/fastrtps*` trước mỗi lần launch (`robot.launch.py` đã tự dọn khi parse).

### 🚨 Lỗi 4: LiDAR dừng ở `out1.. out2..` khi chạy `ros2 run`
* **Hiện tượng:** `ros2 run ros2_lidar lidar_driver` in `out1.. out2..` rồi đứng yên.
* **Tác hại:** Không có `/scan` → costmap trống, AMCL/Nav2 mù.
* **Nguyên nhân:** `lidar_driver` là LifecycleNode, chờ `configure → activate`.
* **Cách khắc phục:** Luôn chạy qua launch (`robot.launch.py`, `ole_lidar.launch.py`) chứa cơ chế activate tự động. Không gọi node trực tiếp.

---

## 3. NHÓM LỖI MẠNG & GIAO TIẾP PHẦN CỨNG

### 🚨 Lỗi 5: Xung đột IP giữa LiDAR và STM32
* **Hiện tượng:** LiDAR bật nhưng `/scan` lúc có lúc mất (`Publisher count: 0`).
* **Tác hại:** Mất cảm biến môi trường, Nav2/AMCL dừng.
* **Nguyên nhân:** OLE xuất xưởng trùng IP `192.168.1.100` với STM32; driver nối nhầm vào W5500.
* **Cách xử lý:**
  1. `arp -a` phân biệt MAC: `00:08:dc:...` (WIZnet) = STM32 `...100`, MAC còn lại = LiDAR → đổi LiDAR sang `...101` trên Web GUI.
  2. Sửa params **của package ngoài** (không phải repo này):
     ```bash
     sed -i 's/lidar_ip: 192.168.1.100/lidar_ip: 192.168.1.101/g' /home/nhatbot_ws/src/nhatbot_drivers/oleros2/src/ros2_lidar/params/ole2dv2.yaml
     ```
  3. Kiểm tra `ping 192.168.1.101` và Host Destination trên Web OLE trỏ về IP Jetson.

### 🚨 Lỗi 6: Tiến trình LiDAR cũ chiếm cổng UDP 2368
* **Hiện tượng:** IP đã đúng nhưng vẫn không có tia quét.
* **Tác hại:** Như Lỗi 5.
* **Nguyên nhân:** Tiến trình `lidar_driver`/`laser_filters` cũ còn sống ngầm giữ port 2368.
* **Cách khắc phục:**
  ```bash
  kill -9 $(ps -ef | grep -E 'ros2_lidar|laser_filters|lidar_driver' | grep -v grep | awk '{print $2}') 2>/dev/null
  ```

### 🚨 Lỗi 7: Mất kết nối UDP tới STM32 khi đang chạy (đứt dây LAN, sập nguồn STM32)
* **Hiện tượng:** Log throttle `[ZlacHardwareInterface] CẢNH BÁO: Mất kết nối UDP...`, vận tốc phản hồi về 0, xe dừng.
* **Tác hại:** Xe dừng tại chỗ; **đã sửa** để controller không sập (xem dưới).
* **Nguyên nhân:** Mất gói telemetry quá 1.0s (grace period).
* **Cách xử lý (hành vi hiện tại sau fix E1):** `read()` giữ position cuối, gán velocity 0, trả `OK` → controller sống, STM32 tự phanh bằng watchdog 250ms của nó; khi có mạng lại, log `[RECOVERY]` và xe chạy tiếp. Kiểm tra dây LAN, nguồn STM32, `ping 192.168.1.100`.

### 🚨 Lỗi 8: Mất kết nối CAN STM32 ↔ ZLAC (`0xEE01`) / Kẹt tải quá dòng (`0xEEEE`)
* **Hiện tượng:** Log `MẤT KẾT NỐI CAN (0xEE01)` hoặc `BẢO VỆ KẸT TẢI/QUÁ DÒNG (0xEEEE)`; trường hợp `0xEEEE` STM32 ngắt bảo vệ 2 giây.
* **Tác hại:** `0xEE01`: không có telemetry → xe dừng (như Lỗi 7). `0xEEEE`: xe khựng 2s giữa đường.
* **Nguyên nhân:** Tuột cáp CAN H/L, thiếu trở đầu cuối 120 Ohm (`0xEE01`); bánh bị chặn cơ khí, dòng vọt > 6A (`0xEEEE`).
* **Cách xử lý:** Kiểm tra cáp CAN + trở 120 Ohm; với `0xEEEE` thì dọn vật cản, **kê bổng 2 bánh** khi test phần mềm (xem Lỗi 10).

---

## 4. NHÓM LỖI ĐIỀU KHIỂN & CHUYỂN ĐỘNG

### 🚨 Lỗi 9: Xung đột kiểu `Twist` vs `TwistStamped` [ĐÃ SỬA]
* **Hiện tượng:** `Cannot echo topic '/diff_drive_controller/cmd_vel', more than one type: [Twist, TwistStamped]`.
* **Tác hại:** Lệnh không tới controller, bánh không quay.
* **Nguyên nhân:** Trên Humble, `twist_mux` chỉ phát `Twist`, còn `diff_drive_controller` mặc định nghe `TwistStamped`.
* **Cách xử lý (đã áp dụng):** `use_stamped_vel: false` + `enable_stamped_cmd_vel: false`, remap `cmd_vel_out → .../cmd_vel_unstamped`. **Cấm** đổi ngược lại một trong hai mà không đổi cả chuỗi.

### 🚨 Lỗi 10: Xe không lăn bánh dưới sàn (ma sát tĩnh)
* **Hiện tượng:** Đặt `cmd_v = 0.3 m/s`, encoder nhích nhẹ nhưng bánh không quay dưới đất; kê bổng lên quay tít (~18 RPM).
* **Tác hại:** Tưởng phần mềm hỏng, mất thời gian debug sai hướng.
* **Nguyên nhân:** Tốc độ thấp (~15-18 RPM) không thắng ma sát tĩnh của khung nặng + lốp.
* **Quy tắc:** Luôn **kê bổng 2 bánh** khi test phần mềm/firmware; đặt xuống sàn thì tăng dần vận tốc.

### 🚨 Lỗi 11: Xe chạy thẳng nhưng odom báo quay (hoặc ngược lại)
* **Hiện tượng:** TF `odom → base_link` xoay vòng khi xe đi thẳng.
* **Tác hại:** EKF/Nav2/AMCL định vị sai hoàn toàn.
* **Nguyên nhân:** Sai cờ `motor_b_reverse` (2 motor lắp đối xứng 180°), sai CPR (phải 4096), sai `wheel_radius 0.0535` / `wheel_base 0.45`.
* **Cách xử lý:** Kiểm tra `motor_b_reverse=true` trong `urdf/zlac_robot.urdf.xacro` (code đã đọc param này từ URDF); verify `ros2 topic echo /diff_drive_controller/odom` khi đẩy xe tay.

### 🚨 Lỗi 12: Nút Y (reset odom) không có tác dụng cơ khí
* **Hiện tượng:** Bấm Y chỉ thấy log DEBUG `Service /reset_odom chua san sang`.
* **Tác hại:** Hiểu nhầm là odom đã reset trong khi encoder vẫn giữ số cũ.
* **Nguyên nhân:** Kiến trúc ros2_control không có service `/reset_odom`; nút Y hiện chỉ phát `/set_pose` reset EKF.
* **Cách xử lý:** Muốn reset cả odom bánh xe thì restart `diff_drive_controller` (unspawn/spawn lại) hoặc chấp nhận reset EKF-only. Đừng cố gọi service không tồn tại.

---

## 5. NHÓM LỖI ĐỊNH VỊ & TỰ HÀNH (EKF / AMCL / NAV2)

### 🚨 Lỗi 13: `[ekf_node] Failed to meet update rate!`
* **Hiện tượng:** EKF spam `Took 0.05s...0.25s`.
* **Tác hại:** TF `odom→base_link` cũ → AMCL + 2 costmap đồng loạt drop scan, pose mới không lan được.
* **Nguyên nhân:** CPU đói khi full stack Nav2 cùng chạy (load ~14/6 nhân). EKF 20Hz chỉ êm khi chưa bật Nav2.
* **Cách xử lý:**
  1. `config/ekf.yaml`: `frequency: 10.0` (controller Nav2 cũng 10Hz nên không mất gì).
  2. `config/amcl_config.yaml`: `max_particles: 1500`, `min_particles: 400`.
  3. **Không chạy RViz trên Jetson** (ngốn ~50% CPU) — chạy RViz ở máy tính remote cùng mạng.
  4. Nếu `bno055` chiếm >10% CPU (`top`), kiểm tra log I2C lỗi/reconnect — driver 50Hz bình thường chỉ tốn vài %.
* **Ngoại lệ bình thường:** đúng 1 dòng `Took 0.05x` ngay khi bấm Y (reset) do nạp lại ma trận `/set_pose` — vô hại.

### 🚨 Lỗi 14: Nav2 `Failed to make progress` / xe đứng yên (board RK3399)
* **Hiện tượng:** MPPI tính toán chậm (~6-10Hz thực tế), `model_dt` lệch, xe không bám đường.
* **Tác hại:** Tự hành liệt dù localization tốt.
* **Nguyên nhân:** `controller_frequency: 20Hz` quá sức CPU RK3399.
* **Cách xử lý (đã áp dụng trong `controller.yaml`):** Hạ `controller_frequency` → `10.0`, `model_dt: 0.1`, `local_costmap update_frequency: 10.0`; giới hạn MPPI khớp phần cứng (`vx_max 0.30`, `wz_max 0.60`). Đừng tăng lại khi chưa đổi board mạnh hơn.

### 🚨 Lỗi 15: AMCL phân tán hạt / nhảy pose
* **Hiện tượng:** Pose trên RViz nhảy lung tung khi khởi động.
* **Tác hại:** Nav2 đi sai đường từ đầu.
* **Nguyên nhân:** `initial_pose` mặc định (0,0,0) khác vị trí thật; `laser_max_range -1` dùng tầm scan xa nhiễu.
* **Cách xử lý:** Dùng `2D Pose Estimate` trên RViz đặt pose ban đầu; giữ `scan_topic: /scan` (bản đã lọc); kiểm tra `header.frame_id` của `/scan` là `laser_frame` và TF `base_link → laser_frame` tồn tại (`ros2 run tf2_ros tf2_echo base_link laser_frame`).

### 🚨 Lỗi 16: Tên plugin Nav2 sai kiểu `::` vs `/` (planner/behavior không tạo được)
* **Hiện tượng:** `planner_server` hoặc `behavior_server` báo FATAL:
  ```text
  Failed to create global planner. Exception: ... class nav2_theta_star_planner::ThetaStarPlanner ... does not exist.
  Declared types are ... nav2_theta_star_planner/ThetaStarPlanner ...
  ```
  Kèm theo `lifecycle_manager_navigation: Failed to bring up all requested nodes. Aborting bringup` → `bt_navigator` không active → click goal bị ngó lơ.
* **Tác hại:** Toàn bộ Nav2 liệt dù config nhìn đúng.
* **Nguyên nhân:** Bản Nav2 này đăng ký một số plugin bằng `/` (`nav2_theta_star_planner/ThetaStarPlanner`, `nav2_behaviors/Spin|BackUp|DriveOnHeading|Wait`), còn file yaml ghi `::`. Các plugin `nav2_controller::*`, `dwb_*::*`, `nav2_bt_navigator::*` vẫn dùng `::` bình thường.
* **Cách xử lý:** Sửa đúng tên trong `config/nav2/planner_server.yaml` và `config/nav2/recovery.yaml` theo danh sách `Declared types` trong thông báo lỗi. Quy tắc: khi thêm plugin mới, đối chiếu tên với `Declared types` trong log FATAL.

### 🚨 Lỗi 17: Robot đứng ngoài bản đồ (`Robot is out of bounds of the costmap`)
* **Hiện tượng:** `planner_server` spam `Sensor origin at (x, y) is out of map bounds ... cannot raytrace`, đặt goal không đi.
* **Tác hại:** Global costmap không raytrace được, planner không lập đường đi từ vị trí robot.
* **Nguyên nhân:** AMCL `initial_pose` mặc định (0,0,0) nằm ngoài vùng bản đồ đã quét (VD map `x∈[0, 31.78], y∈[-13.54, -4.06]` không chứa `(0,0)`).
* **Cách xử lý:** Trên RViz dùng `2D Pose Estimate` đặt pose đúng vị trí thật của xe trong map trước khi đặt goal (hoặc CLI: `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {...}}"`). Kiểm tra biên map: `ros2 topic echo /map --once | grep -E "width|height|resolution|origin"`.

### 🚨 Lỗi 18: `controller_server` chết SIGILL trên TX2 (MPPI chứa lệnh ARMv8.1)
* **Hiện tượng:** `process has died [exit code -4]`, GDB: `SIGILL` tại lệnh `ldaddal` trong `MPPIController::configure()` (`libmppi_controller.so` apt 1.1.18 và 1.1.20 đều dính, 117+146 lệnh LSE).
* **Tác hại:** Nav2 liệt hoàn toàn.
* **Nguyên nhân:** Binary apt build cho ARMv8.1+ (LSE atomics), TX2 (Cortex-A57/Denver 2, ARMv8.0) không thực thi được.
* **Cách xử lý:** (1) Tạm: chuyển `FollowPath` sang `dwb_core::DWBLocalPlanner` (DWB + critics đã quét sạch LSE). Quét lib mới bằng `objdump -d <lib> | grep -cE "ldaddal|staddl|casal|swpal"` — phải ra 0 mới dùng. (2) Triệt để: build `nav2_mppi_controller` từ source ngay trên TX2 trong workspace overlay riêng.

### 🚨 Lỗi 19: CLI mù tịt dù node đang chạy (`ros2 node list` rỗng)
* **Hiện tượng:** Process đầy (`ps` thấy hết) nhưng `ros2 node/lifecycle/topic list` rỗng dù `ROS_DOMAIN_ID` khớp.
* **Tác hại:** Không kiểm tra/chẩn đoán được gì bằng CLI.
* **Nguyên nhân:** ros2 daemon kẹt (không phải DDS — data-path giữa các node vẫn sống, AMCL vẫn nhận scan).
* **Cách xử lý:** Mọi lệnh check thêm cờ `--no-daemon` (VD `ros2 node list --no-daemon`), hoặc `ros2 daemon stop` rồi thử lại.

### 🚨 Lỗi 20: Trùng tên node filter LiDAR (driver ngoài tự spawn filter)
* **Hiện tượng:** ROS cảnh báo nodes share an exact name; 2 `scan_to_scan_filter_chain` lọc trùng việc.
* **Tác hại:** Nhầm lẫn topic, phí CPU, khó debug.
* **Nguyên nhân:** `ole2dv2_launch.py` của driver ngoài tự kèm 1 filter (config của package khác), cộng filter của `robot.launch.py`.
* **Cách xử lý:** Đổi tên filter bên mình thành `scan_to_scan_filter_main` (đã làm). Về lâu dài: vô hiệu filter thừa bên driver ngoài hoặc gộp về 1 tầng duy nhất.

---

## 6. LỖI FIRMWARE/PHẦN CỨNG ĐÃ SỬA (CẤM TÁI PHẠM)

### ❌ [ĐÃ SỬA] Đổi SDO sang RPDO gửi vận tốc → motor chết lặng
* Driver ZLAC8015D từ chối RPDO chưa map EEPROM. Giữ nguyên `SDO Write 0x60FF:01/02` + `HAL_Delay(2)`.

### ❌ [ĐÃ SỬA] Dòng điện đọc luôn 0.0A
* Map sai `0x6077 sub 03` (32-bit gộp). Phải map rời `0x6077:01` + `0x6077:02` (16-bit) trong `_TPDO1_Config`.

### ❌ [ĐÃ SỬA] Breakaway kick làm xe không xoay tại chỗ
* Kick ma sát chỉ áp dụng cho vận tốc tịnh tiến, giữ `min_kick = 0.0` cho `omega`. (Logic thời Python; kiến trúc ros2_control hiện tại nhường tăng/giảm tốc cho profile driver 700/900ms.)

---

## 7. BẢNG LỆNH "CỨU HỘ & KHỞI ĐỘNG NHANH" (QUICK CHEATSHEET)

```bash
# 1. Dọn dẹp tiến trình treo & file rác bộ nhớ
rm -rf /dev/shm/fastrtps*
kill -9 $(ps -ef | grep -E 'ros2_control_node|zlac|lidar|laser_filters|twist_mux' | grep -v grep | awk '{print $2}') 2>/dev/null

# 2. LỰA CHỌN KHỞI CHẠY:
# Cách A: TOÀN BỘ TỰ HÀNH (Hardware + AMCL + Nav2) trong 1 lệnh duy nhất:
source /home/nhatbot_ws/install/setup.bash
ros2 launch can_test_motor bringup_all.launch.py enable_deadman:=false

# Cách B: Chỉ phần cứng để test tay cầm / cảm biến (không nạp bản đồ):
source /home/nhatbot_ws/install/setup.bash
ros2 launch can_test_motor robot.launch.py enable_deadman:=false
```

Kiểm tra sức khỏe ở Terminal 2:
```bash
ros2 topic hz /scan                        # ~15 Hz (bản đã lọc)
ros2 topic hz /odometry/filtered           # ~20 Hz (EKF)
ros2 topic echo /diff_drive_controller/odom --once
ros2 control list_controllers              # joint_state_broadcaster + diff_drive_controller: active
ros2 run tf2_ros tf2_echo odom base_link   # TF liền mạch, không nhảy
ros2 topic echo /scan --once | grep frame_id  # phải là laser_frame
```
