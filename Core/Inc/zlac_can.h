/**
 * ============================================================================
 * File: zlac_can.h
 * Mô tả: Driver giao tiếp STM32F103 <-> ZLAC8015D qua giao thức CANopen
 * Phần cứng: STM32F103C8T6 + TJA1050/SN65HVD230 + ZLAC8015D
 * Baudrate: 500kbps (CAN1: Prescaler 12, BS1 3TQ, BS2 2TQ, SJW 1TQ)
 * ============================================================================
 *
 * CÁCH SỬ DỤNG:
 * 1. Gọi ZLAC_CAN_Init() một lần trong main() sau khi MX_CAN_Init() đã chạy.
 * 2. Gọi ZLAC_StateMachine() định kỳ trong vòng lặp while(1) của main.
 * 3. Khi ZLAC_IsReady() trả về true, gọi ZLAC_SetSpeed_mps(v, omega) để điều khiển xe.
 * 4. Nhận dữ liệu phản hồi (vận tốc, encoder, dòng điện, điện áp) từ biến toàn cục zlac_fb.
 * ============================================================================
 */

#ifndef ZLAC_CAN_H
#define ZLAC_CAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * PHẦN 1: CẤU HÌNH PHẦN CỨNG & THÔNG SỐ CƠ KHÍ ROBOT
 * ============================================================================ */
#define ZLAC_NODE_ID              0x01   /* Node ID của ZLAC8015D (cài đặt bằng công tắc DIP) */
#define ZLAC_WHEEL_RADIUS_M       0.08f  /* Bán kính bánh xe: 0.08m (80mm) */
#define ZLAC_WHEELBASE_M          0.35f  /* Khoảng cách giữa 2 tâm bánh xe: 0.35m (350mm) */
#define ZLAC_MAX_SPEED_MPS        1.5f   /* Vận tốc dài tối đa cho phép của robot (m/s) */
#define ZLAC_MOTOR_B_REVERSE      1      /* Đặt = 1 nếu motor B lắp đối xứng ngược chiều quay */

/* ============================================================================
 * PHẦN 2: NGƯỠNG BẢO VỆ AN TOÀN (QUÁ DÒNG & KẸT TẢI - STALL PROTECTION)
 * ============================================================================ */
#define ZLAC_STALL_CURRENT_THRESHOLD   60     /* Dòng kẹt: 6.0A (đơn vị: 0.1A) */
#define ZLAC_STALL_VELOCITY_THRESHOLD  30     /* Vận tốc quay gần đứng yên: 3.0 RPM (đơn vị: 0.1 RPM) */
#define ZLAC_STALL_TIMEOUT_MS          400    /* Kẹt liên tục quá 400ms -> ngắt bảo vệ khẩn cấp */
#define ZLAC_OVERCURRENT_THRESHOLD     120    /* Quá dòng cực đại tức thời: 12.0A -> ngắt ngay */
#define ZLAC_ERR_STALL_OVERCURRENT     0xEEEE /* Mã lỗi tự định nghĩa khi kích hoạt bảo vệ kẹt tải */

/* ============================================================================
 * PHẦN 3: CẤU TRÚC DỮ LIỆU & KIỂU ĐỊNH NGHĨA
 * ============================================================================ */

/**
 * @brief Trạng thái vận hành của State Machine quản lý ZLAC
 */
typedef enum {
    ZLAC_WAIT_HB   = 0,   /**< Chờ nhận tín hiệu Heartbeat từ ZLAC */
    ZLAC_CONFIG    = 1,   /**< Đang cấu hình PDO mapping và tham số vận hành */
    ZLAC_ENABLING  = 2,   /**< Đang kích hoạt servo (trình tự Controlword 0x06->0x07->0x0F) */
    ZLAC_READY     = 3,   /**< Sẵn sàng nhận lệnh điều khiển vận tốc */
    ZLAC_FAULT     = 4,   /**< Có lỗi (phần cứng driver hoặc ngắt mềm do kẹt tải) */
} ZLAC_State_t;

/**
 * @brief Dữ liệu phản hồi thực tế đọc từ driver ZLAC8015D
 */
typedef struct {
    volatile int16_t  vel_a;       /**< Vận tốc thực tế motor A (rpm × 10) */
    volatile int16_t  vel_b;       /**< Vận tốc thực tế motor B (rpm × 10) */
    volatile int32_t  pos_a;       /**< Vị trí encoder motor A (counts) */
    volatile int32_t  pos_b;       /**< Vị trí encoder motor B (counts) */
    volatile int16_t  current_a;   /**< Dòng điện thực tế motor A (đơn vị: 0.1A) */
    volatile int16_t  current_b;   /**< Dòng điện thực tế motor B (đơn vị: 0.1A) */
    volatile uint16_t bus_voltage; /**< Điện áp DC Bus / Pin (đơn vị: 0.1V hoặc 0.01V) */
    volatile uint16_t error_code;  /**< Mã lỗi (0 = bình thường, 0xEEEE = quá dòng/kẹt tải) */
    volatile uint32_t hb_tick;     /**< Mốc thời gian nhận Heartbeat cuối (ms) */
    volatile uint8_t  hb_received; /**< Cờ báo hiệu đã nhận Heartbeat */
} ZLAC_Feedback_t;

/**
 * @brief Tọa độ và trạng thái Odometry của robot
 */
typedef struct {
    float x;       /**< Tọa độ X trong hệ quy chiếu robot (m) */
    float y;       /**< Tọa độ Y trong hệ quy chiếu robot (m) */
    float theta;   /**< Góc hướng mũi xe (rad, trong khoảng [-PI, PI]) */
    float v;       /**< Vận tốc dài tịnh tiến tức thời (m/s) */
    float omega;   /**< Vận tốc góc quay tức thời (rad/s) */
} ZLAC_Odometry_t;

/* ============================================================================
 * PHẦN 4: BIẾN TOÀN CỤC (EXTERN)
 * ============================================================================ */
extern ZLAC_Feedback_t  zlac_fb;
extern ZLAC_Odometry_t  zlac_odom;
extern ZLAC_State_t     zlac_state;

/* ============================================================================
 * PHẦN 5: KHAI BÁO CÁC HÀM API CÔNG KHAI
 * ============================================================================ */

/* --- 5.1 Khởi tạo và Quản lý trạng thái --- */
/** @brief Khởi tạo phần cứng CAN, cấu hình bộ lọc (filter) và bật ngắt nhận RX0 */
void ZLAC_CAN_Init(void);

/** @brief State Machine tự động quản lý kết nối, cấu hình PDO, bật servo và bảo vệ */
void ZLAC_StateMachine(void);

/** @brief Kiểm tra driver đã sẵn sàng nhận lệnh điều khiển hay chưa */
bool ZLAC_IsReady(void);

/* --- 5.2 Điều khiển chuyển động --- */
/** @brief Đặt vận tốc dài v (m/s) và vận tốc góc omega (rad/s) cho robot */
void ZLAC_SetSpeed_mps(float v, float omega);

/** @brief Đặt vận tốc trực tiếp cho từng bánh xe (đơn vị: 1 RPM) */
void ZLAC_SetSpeed_raw(int16_t vel_a, int16_t vel_b);

/** @brief Phanh dừng xe khẩn cấp (gửi lệnh vận tốc bằng 0) */
void ZLAC_Stop(void);

/* --- 5.3 Đọc dữ liệu vận tốc & Odometry --- */
/** @brief Lấy vận tốc thực tế bánh A (m/s) */
float ZLAC_GetVelA_mps(void);

/** @brief Lấy vận tốc thực tế bánh B (m/s) */
float ZLAC_GetVelB_mps(void);

/** @brief Cập nhật tọa độ Odometry dựa trên sự thay đổi của Encoder */
void ZLAC_Odom_Update(float dt);

/** @brief Đặt lại tọa độ Odometry về mốc 0 (X=0, Y=0, Theta=0) */
void ZLAC_Odom_Reset(void);

/* --- 5.4 Giao thức CANopen cấp thấp --- */
/** @brief Ghi giá trị vào Object Dictionary của driver qua SDO (blocking) */
void SDO_Write(uint8_t node_id, uint16_t index, uint8_t sub, int32_t value, uint8_t size_bytes);

/** @brief Gửi yêu cầu đọc một tham số trong Object Dictionary qua SDO */
void SDO_Read_Request(uint8_t node_id, uint16_t index, uint8_t sub);

/** @brief Gửi lệnh quản lý mạng NMT (Start, Stop, Pre-Op, Reset) */
void NMT_Send(uint8_t node_id, uint8_t cmd);

#endif /* ZLAC_CAN_H */
