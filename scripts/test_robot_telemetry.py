#!/usr/bin/env python3
"""
Test script for STM32 + W5500 + ZLAC8015D CANopen Gateway
Protocol: UDP Binary Packet with CRC-16/MODBUS
Rate: 50 Hz (20ms cycle time)
"""

import socket
import struct
import time
import math

# --- CẤU HÌNH KẾT NỐI MẠNG ---
STM32_IP = "192.168.1.100"
STM32_PORT = 8888
LOCAL_PORT = 8888

# --- THÔNG SỐ CƠ KHÍ ROBOT ---
WHEEL_RADIUS_M = 0.08      # Bán kính bánh xe: 0.08 m (8 cm)
ENCODER_CPR = 4096         # Số xung trên 1 vòng quay bánh xe

# Hàm tính CRC-16/MODBUS (Đồng bộ với STM32)
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    sock.settimeout(0.01)

    tx_count = 0
    rx_count = 0
    last_stat_time = time.time()
    rtt_ms = 0.0

    initial_enc_a = None
    initial_enc_b = None
    vel_a_rpm = 0.0
    vel_b_rpm = 0.0
    vel_a_mps = 0.0
    distance_m = 0.0
    pos_a = 0
    pos_b = 0
    cur_a = 0.0
    cur_b = 0.0
    v_bus = 0.0
    last_err = 0

    print("=" * 75)
    print(f"[*] ĐANG KẾT NỐI TỚI STM32 GATEWAY ({STM32_IP}:{STM32_PORT}) QUA CRC-16...")
    print("[*] Tần số mục tiêu: 50 Hz | Nhấn Ctrl + C để dừng xe an toàn.")
    print("=" * 75)

    # Vận tốc đặt (Thay đổi tại đây):
    v_target = 0.1         # Vận tốc dài tiến thẳng (m/s)
    omega_target = 0.0     # Vận tốc góc quay (rad/s)

    try:
        while True:
            loop_start = time.time()

            # 1. Đóng gói 10 bytes payload: Header (2B) + float v (4B) + float omega (4B)
            payload = struct.pack('<BBff', 0xAA, 0x55, v_target, omega_target)
            crc = calculate_crc16(payload)
            packet = payload + struct.pack('<H', crc)

            t_send = time.time()
            sock.sendto(packet, (STM32_IP, STM32_PORT))
            tx_count += 1

            # 2. Hứng phản hồi Telemetry từ STM32
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) >= 22:
                    rx_count += 1
                    rtt_ms = (time.time() - t_send) * 1000.0

                    h1, h2, vel_a_raw, vel_b_raw, pos_a, pos_b, err, cur_a_raw, cur_b_raw, v_bus_raw = struct.unpack('<BBhhllHhhH', data[:22])

                    if h1 == 0x55 and h2 == 0xAA:
                        vel_a_rpm = vel_a_raw / 10.0
                        vel_b_rpm = vel_b_raw / 10.0
                        vel_a_mps = (vel_a_rpm * 2.0 * math.pi * WHEEL_RADIUS_M) / 60.0
                        cur_a = cur_a_raw / 10.0
                        cur_b = cur_b_raw / 10.0
                        v_bus = v_bus_raw / 10.0 if v_bus_raw < 1000 else v_bus_raw / 100.0  # Hỗ trợ cả 0.1V lẫn 0.01V
                        last_err = err

                        if initial_enc_a is None:
                            initial_enc_a = pos_a
                            initial_enc_b = pos_b

                        delta_counts = pos_a - initial_enc_a
                        distance_m = (delta_counts / ENCODER_CPR) * (2.0 * math.pi * WHEEL_RADIUS_M)

                elif len(data) >= 16:
                    rx_count += 1
                    rtt_ms = (time.time() - t_send) * 1000.0

                    h1, h2, vel_a_raw, vel_b_raw, pos_a, pos_b, err = struct.unpack('<BBhhllH', data[:16])

                    if h1 == 0x55 and h2 == 0xAA:
                        vel_a_rpm = vel_a_raw / 10.0
                        vel_b_rpm = vel_b_raw / 10.0
                        vel_a_mps = (vel_a_rpm * 2.0 * math.pi * WHEEL_RADIUS_M) / 60.0
                        last_err = err

                        if initial_enc_a is None:
                            initial_enc_a = pos_a
                            initial_enc_b = pos_b

                        delta_counts = pos_a - initial_enc_a
                        distance_m = (delta_counts / ENCODER_CPR) * (2.0 * math.pi * WHEEL_RADIUS_M)

            except socket.timeout:
                pass

            # 3. Hiển thị thống kê định kỳ mỗi 0.5 giây
            now = time.time()
            if now - last_stat_time >= 0.5:
                dt = now - last_stat_time
                tx_hz = tx_count / dt
                rx_hz = rx_count / dt
                tx_count = 0
                rx_count = 0
                last_stat_time = now

                err_str = f"Err:0x{last_err:04X}"
                if last_err == 0xEEEE:
                    err_str = "\033[91m[CẢNH BÁO: KẸT TẢI / QUÁ DÒNG 0xEEEE]\033[0m"

                print(f"[50Hz] TX:{tx_hz:4.1f}Hz | RX:{rx_hz:4.1f}Hz | Trễ:{rtt_ms:4.2f}ms | "
                      f"Vel_A:{vel_a_rpm:5.1f}RPM ({vel_a_mps:4.2f}m/s) | "
                      f"I_A:{cur_a:4.1f}A | I_B:{cur_b:4.1f}A | V_Bus:{v_bus:4.1f}V | {err_str}")

            # 4. Duy trì chu kỳ 50Hz (20ms)
            elapsed = time.time() - loop_start
            sleep_time = max(0.0, 0.02 - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n" + "=" * 75)
        print("[!] ĐANG DỪNG ĐỘNG CƠ AN TOÀN VÀ ĐÓNG KẾT NỐI...")
        stop_payload = struct.pack('<BBff', 0xAA, 0x55, 0.0, 0.0)
        stop_packet = stop_payload + struct.pack('<H', calculate_crc16(stop_payload))
        for _ in range(5):
            sock.sendto(stop_packet, (STM32_IP, STM32_PORT))
            time.sleep(0.01)
        sock.close()
        print("[✓] ĐÃ PHANH DỪNG ROBOT AN TOÀN. HOÀN TẤT!")
        print("=" * 75)

if __name__ == '__main__':
    main()
