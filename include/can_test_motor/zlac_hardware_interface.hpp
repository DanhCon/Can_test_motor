#ifndef CAN_TEST_MOTOR__ZLAC_HARDWARE_INTERFACE_HPP_
#define CAN_TEST_MOTOR__ZLAC_HARDWARE_INTERFACE_HPP_


#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>



#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>



#include "can_test_motor/zlac_udp_driver.hpp"


namespace can_test_motor {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class ZlacHardwareInterface : public hardware_interface::SystemInterface {
public:
    ZlacHardwareInterface() = default;
    ~ZlacHardwareInterface() override;
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
private:
    void ioLoop();  // Luồng ngầm độc lập gửi nhận UDP
    std::unique_ptr<ZlacUdpDriver> driver_;
    std::thread io_thread_;
    std::atomic<bool> io_running_{false};
    std::mutex io_mutex_;
    // Thông số cơ học
    double wheel_radius_{0.0535};   // Bán kính bánh xe (m)
    double wheel_base_{0.45};       // Khoảng cách 2 bánh (m)
    double cpr_{4096.0};            // Xung/vòng encoder
    bool motor_b_reverse_{true};    // Đảo chiều motor B
    // Bộ nhớ đệm lệnh (ros2_control)
    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    // Bộ nhớ đệm trao đổi giữa ioLoop và read()/write()
    float cmd_v_{0.0f};
    float cmd_omega_{0.0f};
    int64_t accumulated_pos_left_{0};
    int64_t accumulated_pos_right_{0};
    double current_vel_left_{0.0};
    double current_vel_right_{0.0};
    // Giám sát Grace Period (1.0s)
    std::chrono::steady_clock::time_point last_rx_time_;
    double feedback_grace_period_sec_{1.0};
    std::atomic<bool> connection_healthy_{true};
};
}  // namespace can_test_motor
#endif  // CAN_TEST_MOTOR__ZLAC_HARDWARE_INTERFACE_HPP_