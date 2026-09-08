#include "can_test_motor/zlac_udp_driver.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace can_test_motor {

uint16_t ZlacUdpDriver::calculateCrc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

ZlacUdpDriver::ZlacUdpDriver(const std::string &stm32_ip, int stm32_port, int local_port)
    : stm32_ip_(stm32_ip), stm32_port_(stm32_port), local_port_(local_port) {}

ZlacUdpDriver::~ZlacUdpDriver() {
    close_socket();
}

bool ZlacUdpDriver::init() {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) return false;

    int opt = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Đặt timeout nhận gói 20ms (tránh treo luồng)
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20000;
    setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(local_port_);

    if (bind(sock_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        close_socket();
        return false;
    }
    return true;
}

void ZlacUdpDriver::close_socket() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool ZlacUdpDriver::sendCommand(float v, float omega) {
    if (sock_fd_ < 0) return false;

    UDP_ControlPacket pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;
    pkt.v = v;
    pkt.omega = omega;
    pkt.checksum = calculateCrc16(reinterpret_cast<uint8_t *>(&pkt), 10);

    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(stm32_port_);
    inet_pton(AF_INET, stm32_ip_.c_str(), &dest_addr.sin_addr);

    ssize_t sent = sendto(sock_fd_, &pkt, sizeof(pkt), 0,
                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    return sent == sizeof(pkt);
}

bool ZlacUdpDriver::receiveFeedback(ZlacFeedbackData &out_data) {
    if (sock_fd_ < 0) return false;

    UDP_FeedbackPacket pkt;
    ssize_t bytes_recv = recvfrom(sock_fd_, &pkt, sizeof(pkt), 0, nullptr, nullptr);
    if (bytes_recv < static_cast<ssize_t>(sizeof(pkt))) {
        return false;
    }

    if (pkt.header[0] != 0x55 || pkt.header[1] != 0xAA) {
        return false;
    }

    out_data.pos_left = pkt.pos_a;
    out_data.pos_right = pkt.pos_b;
    // vel_a, vel_b là 0.1 RPM -> chuyển ra rad/s: (vel / 10.0) * (2*pi / 60)
    constexpr double rpm_to_rad_s = (2.0 * 3.141592653589793) / 60.0;
    out_data.vel_left_rad_s = (pkt.vel_a / 10.0) * rpm_to_rad_s;
    out_data.vel_right_rad_s = (pkt.vel_b / 10.0) * rpm_to_rad_s;
    out_data.error_code = pkt.error_code;
    out_data.current_left = pkt.current_a / 10.0;
    out_data.current_right = pkt.current_b / 10.0;
    out_data.battery_voltage = (pkt.bus_voltage < 1000) ? (pkt.bus_voltage / 10.0) : (pkt.bus_voltage / 100.0);
    out_data.valid = true;
    return true;
}

}  // namespace can_test_motor