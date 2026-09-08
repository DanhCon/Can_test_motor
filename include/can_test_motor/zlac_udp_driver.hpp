#ifndef CAN_TEST_MOTOR__ZLAC_UDP_DRIVER_HPP_
#define CAN_TEST_MOTOR__ZLAC_UDP_DRIVER_HPP_
#include <string>
#include <cstdint>
#include <chrono>



namespace can_test_motor { // này là gì ta ????
#pragma pack(push, 1)
// Gói điều khiển gửi xuống STM32 (12 bytes)
struct UDP_ControlPacket {
    uint8_t header[2];   // 0xAA, 0x55
    float v;             // Vận tốc tịnh tiến (m/s)
    float omega;         // Vận tốc xoay (rad/s)
    uint16_t checksum;   // CRC-16 Modbus (10 bytes đầu)
};


struct UDP_FeedbackPacket {
    uint8_t header[2];   // 0x55, 0xAA
    int16_t vel_a;       // Vận tốc Motor A (0.1 RPM)
    int16_t vel_b;       // Vận tốc Motor B (0.1 RPM)
    int32_t pos_a;       // Xung tích lũy Motor A (Left)
    int32_t pos_b;       // Xung tích lũy Motor B (Right)
    uint16_t error_code; // Mã lỗi (0: OK, 0xEEEE: Quá dòng/Kẹt tải, 0xEE01: Mất CAN)
    int16_t current_a;   // Dòng điện Motor A (0.1 A)
    int16_t current_b;   // Dòng điện Motor B (0.1 A)
    uint16_t bus_voltage;// Điện áp Pin / DC Bus
};

#pragma pack(pop)   // này là gì ta ????

struct ZlacFeedbackData {
    int32_t pos_left{0};
    int32_t pos_right{0};
    double vel_left_rad_s{0.0};
    double vel_right_rad_s{0.0};
    double battery_voltage{0.0};
    double current_left{0.0};
    double current_right{0.0};
    uint16_t error_code{0};
    bool valid{false};
};
class ZlacUdpDriver {
public:
    ZlacUdpDriver(const std::string &stm32_ip, int stm32_port, int local_port = 8888);
    ~ZlacUdpDriver();
    bool init();
    void close_socket();
    bool sendCommand(float v, float omega);
    bool receiveFeedback(ZlacFeedbackData &out_data);
    static uint16_t calculateCrc16(const uint8_t *data, size_t length);
private:
    std::string stm32_ip_;
    int stm32_port_;
    int local_port_;
    int sock_fd_{-1};
};
}  // namespace can_test_motor
#endif  // CAN_TEST_MOTOR__ZLAC_UDP_DRIVER_HPP_