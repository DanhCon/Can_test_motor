#include "can_test_motor/zlac_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace can_test_motor {

constexpr double kPi = 3.14159265358979323846;

ZlacHardwareInterface::~ZlacHardwareInterface() {
    on_deactivate(rclcpp_lifecycle::State());
}

CallbackReturn ZlacHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    // Đọc thông số cơ học & mạng từ URDF/Xacro
    wheel_radius_ = std::stod(info_.hardware_parameters.count("wheel_radius") ? info_.hardware_parameters.at("wheel_radius") : "0.0535");
    wheel_base_ = std::stod(info_.hardware_parameters.count("wheel_base") ? info_.hardware_parameters.at("wheel_base") : "0.45");
    cpr_ = std::stod(info_.hardware_parameters.count("cpr") ? info_.hardware_parameters.at("cpr") : "4096.0");
    std::string ip = info_.hardware_parameters.count("stm32_ip") ? info_.hardware_parameters.at("stm32_ip") : "192.168.1.100";
    int port = std::stoi(info_.hardware_parameters.count("stm32_port") ? info_.hardware_parameters.at("stm32_port") : "8888");

    hw_commands_velocities_.resize(2, 0.0);
    hw_states_positions_.resize(2, 0.0);
    hw_states_velocities_.resize(2, 0.0);

    driver_ = std::make_unique<ZlacUdpDriver>(ip, port, 8888);
    RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Khởi tạo ZlacHardwareInterface kết nối STM32 tại %s:%d", ip.c_str(), port);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZlacHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // Bánh trái: index 0
    state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[0]);
    state_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[0]);
    // Bánh phải: index 1
    state_interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[1]);
    state_interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[1]);
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZlacHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[0]);
    command_interfaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[1]);
    return command_interfaces;
}

CallbackReturn ZlacHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    if (!driver_->init()) {
        RCLCPP_ERROR(rclcpp::get_logger("ZlacHardwareInterface"), "Không thể mở Socket UDP!");
        return CallbackReturn::FAILURE;
    }

    last_rx_time_ = std::chrono::steady_clock::now();
    io_running_ = true;
    io_thread_ = std::thread(&ZlacHardwareInterface::ioLoop, this);

    RCLCPP_INFO(rclcpp::get_logger("ZlacHardwareInterface"), "Đã kích hoạt ZlacHardwareInterface & khởi động luồng ioLoop (50Hz)");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ZlacHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    io_running_ = false;
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    if (driver_) {
        driver_->sendCommand(0.0f, 0.0f);
        driver_->close_socket();
    }
    return CallbackReturn::SUCCESS;
}

void ZlacHardwareInterface::ioLoop() {
    auto last_stat_time = std::chrono::steady_clock::now();
    uint32_t tx_fail_count = 0;

    while (io_running_) {
        // 1. Gửi lệnh vận tốc xuống STM32
        float v, omega;
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            v = cmd_v_;
            omega = cmd_omega_;
        }
        if (!driver_->sendCommand(v, omega)) {
            tx_fail_count++;
        }

        // 2. Hứng gói phản hồi Telemetry từ STM32
        ZlacFeedbackData fb;
        if (driver_->receiveFeedback(fb)) {
            std::lock_guard<std::mutex> lock(io_mutex_);
            accumulated_pos_left_ = fb.pos_left;
            accumulated_pos_right_ = motor_b_reverse_ ? -fb.pos_right : fb.pos_right;
            current_vel_left_ = fb.vel_left_rad_s;
            current_vel_right_ = motor_b_reverse_ ? -fb.vel_right_rad_s : fb.vel_right_rad_s;
            last_rx_time_ = std::chrono::steady_clock::now();
            connection_healthy_ = true;

            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time).count() >= 2) {
                last_stat_time = now;
                RCLCPP_INFO(
                    rclcpp::get_logger("ZlacHardwareInterface"),
                    "[STM32_STAT] RX OK: err=0x%04X, vbus=%.1fV, pos_L=%d, pos_R=%d, cmd_v=%.2f",
                    fb.error_code, fb.battery_voltage, fb.pos_left, fb.pos_right, v);
            }
        } else {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time).count() >= 2) {
                last_stat_time = now;
                RCLCPP_WARN(
                    rclcpp::get_logger("ZlacHardwareInterface"),
                    "[STM32_STAT] KHONG NHAN DUOC PHAN HOI UDP TU STM32! (tx_fail=%u)", tx_fail_count);
            }
        }

        // Tần số 50 Hz (20ms/chu kỳ)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

hardware_interface::return_type ZlacHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
    auto now = std::chrono::steady_clock::now();
    double time_since_rx = std::chrono::duration<double>(now - last_rx_time_).count();

    // Cơ chế Grace Period 1.0 giây
    if (time_since_rx > feedback_grace_period_sec_) {
        if (connection_healthy_) {
            RCLCPP_ERROR(rclcpp::get_logger("ZlacHardwareInterface"),
                         "CẢNH BÁO: Mất kết nối UDP tới STM32 quá %.2fs! Dừng an toàn.", time_since_rx);
            connection_healthy_ = false;
        }
        hw_states_velocities_[0] = 0.0;
        hw_states_velocities_[1] = 0.0;
        return hardware_interface::return_type::ERROR;
    }

    double ticks_to_rad = (2.0 * kPi) / cpr_;
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        hw_states_positions_[0] = static_cast<double>(accumulated_pos_left_) * ticks_to_rad;
        hw_states_positions_[1] = static_cast<double>(accumulated_pos_right_) * ticks_to_rad;
        hw_states_velocities_[0] = current_vel_left_;
        hw_states_velocities_[1] = current_vel_right_;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZlacHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // diff_drive_controller cung cấp vận tốc góc rad/s của 2 bánh xe
    double w_left = hw_commands_velocities_[0];
    double w_right = hw_commands_velocities_[1];

    if (!std::isfinite(w_left) || !std::isfinite(w_right)) {
        w_left = 0.0;
        w_right = 0.0;
    }

    // Chuyển đổi từ vận tốc 2 bánh xe sang (v, omega) của tâm robot
    float v = static_cast<float>((w_right + w_left) / 2.0 * wheel_radius_);
    float omega = static_cast<float>((w_right - w_left) / wheel_base_ * wheel_radius_);

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();
    if ((std::abs(v) > 0.001f || std::abs(omega) > 0.001f) &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now_time - last_log_time).count() >= 500) {
        last_log_time = now_time;
        RCLCPP_INFO(
            rclcpp::get_logger("ZlacHardwareInterface"),
            "[HW_IF] Nhan lenh tu diff_drive_controller: v=%.2f m/s, omega=%.2f rad/s", v, omega);
    }

    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        cmd_v_ = v;
        cmd_omega_ = omega;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace can_test_motor

PLUGINLIB_EXPORT_CLASS(can_test_motor::ZlacHardwareInterface, hardware_interface::SystemInterface)