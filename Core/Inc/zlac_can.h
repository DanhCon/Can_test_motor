/**
 * ============================================================================
 * File: zlac_can.h
 * Mô tả: Driver giao tiếp STM32F103 <-> ZLAC8015D qua CANopen
 * Phần cứng: STM32F103 + TJA1050/SN65HVD230 + ZLAC8015D
 * Baudrate: 500kbps
 * ============================================================================
 *
 * CÁCH SỬ DỤNG:
 *
 * 1. Thêm file này và zlac_can.c vào project Keil MDK
 * 2. Include "zlac_can.h" vào main.c
 * 3. Cấu hình CAN trong CubeMX:
 *    - CAN1, Mode: Normal
 *    - Prescaler: 12, BS1: 3TQ, BS2: 2TQ, SJW: 1TQ → 500kbps
 *    - Enable CAN1 RX0 Interrupt
 *    - PA11 = CAN_RX, PA12 = CAN_TX (hoặc PB8/PB9 với remap)
 * 4. Trong main():
 *      ZLAC_CAN_Init();         // Khởi tạo CAN + filter + interrupt
 *      while(1) {
 *          ZLAC_StateMachine();  // Gọi mỗi vòng lặp
 *          if (ZLAC_IsReady()) {
 *              ZLAC_SetSpeed_mps(0.5f, 0.0f);  // Đi thẳng 0.5 m/s
 *          }
 *          HAL_Delay(10);
 *      }
 *
 * ============================================================================
 */

#ifndef ZLAC_CAN_H
#define ZLAC_CAN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * CẤU HÌNH – Thay đổi theo phần cứng của bạn
 * ============================================================================ */
#define ZLAC_NODE_ID          0x01    /* Node ID của ZLAC (đặt bằng DIP switch) */
#define ZLAC_WHEEL_RADIUS_M   0.08f  /* Bán kính bánh xe (m) */
#define ZLAC_WHEELBASE_M      0.35f  /* Khoảng cách 2 bánh (m) */
#define ZLAC_MAX_SPEED_MPS    1.5f   /* Tốc độ tối đa robot (m/s) */
#define ZLAC_MOTOR_B_REVERSE  1      /* Set = 1 nếu motor B lắp ngược chiều */

/* Ngưỡng bảo vệ kẹt tải & quá dòng (Stall & Overcurrent Protection) */
#define ZLAC_STALL_CURRENT_THRESHOLD   60     /* Dòng kẹt: 6.0A (đơn vị: 0.1A) */
#define ZLAC_STALL_VELOCITY_THRESHOLD  30     /* Tốc độ quay gần đứng yên: 3.0 RPM (đơn vị: 0.1 RPM) */
#define ZLAC_STALL_TIMEOUT_MS          400    /* Kẹt liên tục > 400ms -> ngắt bảo vệ */
#define ZLAC_OVERCURRENT_THRESHOLD     120    /* Quá dòng nguy hiểm tức thời: 12.0A -> ngắt ngay */
#define ZLAC_ERR_STALL_OVERCURRENT     0xEEEE /* Mã lỗi tự định nghĩa khi ngắt kẹt tải/quá dòng */

/* ============================================================================
 * CẤU TRÚC DỮ LIỆU
 * ============================================================================ */

/** Trạng thái khởi động */
typedef enum {
    ZLAC_WAIT_HB   = 0,   /**< Chờ Heartbeat từ ZLAC */
    ZLAC_CONFIG    = 1,   /**< Đang cấu hình PDO + mode */
    ZLAC_ENABLING  = 2,   /**< Đang bật servo */
    ZLAC_READY     = 3,   /**< Sẵn sàng nhận lệnh */
    ZLAC_FAULT     = 4,   /**< Có lỗi */
} ZLAC_State_t;

/** Dữ liệu feedback từ ZLAC */
typedef struct {
    volatile int16_t  vel_a;       /**< Tốc độ motor A (rpm×10) */
    volatile int16_t  vel_b;       /**< Tốc độ motor B (rpm×10) */
    volatile int32_t  pos_a;       /**< Vị trí motor A (encoder counts) */
    volatile int32_t  pos_b;       /**< Vị trí motor B (encoder counts) */
    volatile int16_t  current_a;   /**< Dòng điện motor A (đơn vị: 0.1A) */
    volatile int16_t  current_b;   /**< Dòng điện motor B (đơn vị: 0.1A) */
    volatile uint16_t bus_voltage; /**< Điện áp DC Bus (đơn vị: 0.1V hoặc 0.01V) */
    volatile uint16_t error_code;  /**< Mã lỗi (0 = không lỗi, 0xEEEE = quá dòng/kẹt tải) */
    volatile uint32_t hb_tick;     /**< Tick nhận Heartbeat cuối */
    volatile uint8_t  hb_received; /**< Flag: đã nhận Heartbeat */
} ZLAC_Feedback_t;

/** Trạng thái odometry robot */
typedef struct {
    float x;       /**< Tọa độ X (m) */
    float y;       /**< Tọa độ Y (m) */
    float theta;   /**< Góc hướng (rad) */
    float v;       /**< Vận tốc tịnh tiến (m/s) */
    float omega;   /**< Vận tốc góc (rad/s) */
} ZLAC_Odometry_t;

/* ============================================================================
 * BIẾN TOÀN CỤC (extern để dùng ở file khác)
 * ============================================================================ */
extern ZLAC_Feedback_t  zlac_fb;
extern ZLAC_Odometry_t  zlac_odom;
extern ZLAC_State_t     zlac_state;

/* ============================================================================
 * KHAI BÁO HÀM
 * ============================================================================ */

/**
 * @brief Khởi tạo CAN: cấu hình filter + bật interrupt + start peripheral
 *        Gọi 1 lần trong main() sau MX_CAN_Init()
 */
void ZLAC_CAN_Init(void);
void SDO_Read_Request(uint8_t node_id, uint16_t index, uint8_t sub);

/**
 * @brief State machine quản lý khởi động ZLAC
 *        Gọi liên tục trong vòng lặp main (hoặc task FreeRTOS)
 *        Tự động: chờ HB → config PDO → enable servo
 */
void ZLAC_StateMachine(void);

/**
 * @brief Kiểm tra ZLAC đã sẵn sàng nhận lệnh chưa
 * @retval true nếu servo đã Enable và không có lỗi
 */
bool ZLAC_IsReady(void);

/**
 * @brief Đặt tốc độ robot
 * @param v     Tốc độ tịnh tiến (m/s), dương = tiến, âm = lùi
 * @param omega Tốc độ góc (rad/s), dương = quay trái, âm = quay phải
 */
void ZLAC_SetSpeed_mps(float v, float omega);

/**
 * @brief Đặt tốc độ từng bánh trực tiếp (rpm×10)
 * @param vel_a Motor A (rpm×10)
 * @param vel_b Motor B (rpm×10)
 */
void ZLAC_SetSpeed_raw(int16_t vel_a, int16_t vel_b);

/**
 * @brief Dừng robot khẩn cấp (gửi 0 tốc độ)
 */
void ZLAC_Stop(void);

/**
 * @brief Lấy tốc độ bánh A thực tế (m/s)
 */
float ZLAC_GetVelA_mps(void);

/**
 * @brief Lấy tốc độ bánh B thực tế (m/s)
 */
float ZLAC_GetVelB_mps(void);

/**
 * @brief Cập nhật odometry – gọi mỗi dt giây
 * @param dt Delta time (giây) kể từ lần gọi trước
 */
void ZLAC_Odom_Update(float dt);

/**
 * @brief Reset tọa độ odometry về 0
 */
void ZLAC_Odom_Reset(void);

/**
 * @brief Ghi 1 thông số SDO (blocking, chỉ dùng lúc init)
 */
void SDO_Write(uint8_t node_id, uint16_t index, uint8_t sub,
               int32_t value, uint8_t size_bytes);

/**
 * @brief Gửi lệnh NMT
 * @param node_id  ID của node (0 = broadcast)
 * @param cmd      0x01=Start, 0x02=Stop, 0x80=PreOp, 0x81=Reset
 */
void NMT_Send(uint8_t node_id, uint8_t cmd);

#endif /* ZLAC_CAN_H */
