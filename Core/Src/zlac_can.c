/**
 * ============================================================================
 * File: zlac_can.c
 * Mô tả: Hiện thực Driver giao tiếp STM32F103 <-> ZLAC8015D qua CANopen
 *
 * Tài liệu tham khảo:
 * - ZLAC8015D CANopen Communication Quick Start Guide Version 1.00
 * - ZLAC8015D CANopen Aging V1.0 C Code Routine
 * ============================================================================
 */

#include "zlac_can.h"
#include "can.h"    /* HAL CAN handle (hcan) sinh bởi CubeMX */
#include <math.h>   /* sinf, cosf, fabsf, roundf, fmaxf */
#include <string.h> /* memset */

/* ============================================================================
 * PHẦN 1: BIẾN TOÀN CỤC & BIẾN NỘI BỘ MODULE
 * ============================================================================ */
ZLAC_Feedback_t  zlac_fb    = {0};
ZLAC_Odometry_t  zlac_odom  = {0};
ZLAC_State_t     zlac_state = ZLAC_WAIT_HB;

/* Mốc thời gian chuyển trạng thái (dùng để tính timeout) */
static uint32_t state_entry_tick = 0;

/* Biến lưu trữ giá trị lệnh tốc độ gần nhất */
static int16_t g_vel_a = 0, g_vel_b = 0;

/* ============================================================================
 * PHẦN 2: MACRO HỖ TRỢ NỘI BỘ
 * ============================================================================ */
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* Giới hạn giá trị nằm trong đoạn [lo, hi] */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* Đọc số nguyên có dấu 16-bit và 32-bit dạng Little-Endian từ mảng byte */
#define RD_I16(b)   ((int16_t)((uint16_t)(b)[0] | ((uint16_t)(b)[1] << 8)))
#define RD_I32(b)   ((int32_t)((uint32_t)(b)[0] | ((uint32_t)(b)[1] << 8) \
                              | ((uint32_t)(b)[2] << 16) | ((uint32_t)(b)[3] << 24)))

/* ============================================================================
 * PHẦN 3: HÀM NỘI BỘ GỬI CAN FRAME
 * ============================================================================ */
/**
 * @brief Gửi 1 frame dữ liệu CAN tiêu chuẩn (Standard ID 11-bit)
 * @param std_id CAN ID tiêu chuẩn
 * @param dlc    Số byte dữ liệu (0 đến 8)
 * @param data   Con trỏ mảng byte chứa dữ liệu
 * @return HAL_StatusTypeDef HAL_OK nếu gửi thành công
 */
static HAL_StatusTypeDef _CAN_Send(uint32_t std_id, uint8_t dlc, uint8_t *data)
{
    extern CAN_HandleTypeDef hcan;
    CAN_TxHeaderTypeDef hdr;
    uint32_t mailbox;

    hdr.StdId              = std_id;
    hdr.ExtId              = 0;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = dlc;
    hdr.TransmitGlobalTime = DISABLE;

    /* Chờ giải phóng mailbox gửi (tối đa 5ms để tránh treo CPU) */
    uint32_t t = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        if (HAL_GetTick() - t > 5) return HAL_TIMEOUT;
    }

    return HAL_CAN_AddTxMessage(&hcan, &hdr, data, &mailbox);
}

/* ============================================================================
 * PHẦN 4: CÁC HÀM NGUYÊN THỦY CANOPEN (SDO & NMT)
 * ============================================================================ */

/**
 * @brief Ghi tham số vào Object Dictionary của driver qua giao thức SDO
 * @param node_id    Node ID của driver (thường là 1)
 * @param index      Chỉ mục Object (16-bit)
 * @param sub        Chỉ mục phụ Subindex (8-bit)
 * @param value      Giá trị cần ghi
 * @param size_bytes Kích thước dữ liệu: 1, 2, hoặc 4 bytes
 */
void SDO_Write(uint8_t node_id, uint16_t index, uint8_t sub,
               int32_t value, uint8_t size_bytes)
{
    /*
     * SDO Write Request (Client -> Server):
     * Byte 0: Command Specifier (0x2F: 1 byte, 0x2B: 2 bytes, 0x27: 3 bytes, 0x23: 4 bytes)
     * Byte 1-2: Index (Little-Endian)
     * Byte 3: Subindex
     * Byte 4-7: Dữ liệu ghi (Little-Endian)
     */
    static const uint8_t cs_table[] = {0, 0x2F, 0x2B, 0x27, 0x23};
    uint8_t frame[8];

    frame[0] = (size_bytes <= 4) ? cs_table[size_bytes] : 0x23;
    frame[1] = (uint8_t)(index & 0xFF);
    frame[2] = (uint8_t)((index >> 8) & 0xFF);
    frame[3] = sub;
    frame[4] = (uint8_t)(value & 0xFF);
    frame[5] = (uint8_t)((value >> 8) & 0xFF);
    frame[6] = (uint8_t)((value >> 16) & 0xFF);
    frame[7] = (uint8_t)((value >> 24) & 0xFF);

    _CAN_Send(0x600 + node_id, 8, frame);
}

/**
 * @brief Gửi yêu cầu đọc một tham số trong Object Dictionary qua SDO
 * @param node_id Node ID của driver
 * @param index   Chỉ mục Object (16-bit)
 * @param sub     Chỉ mục phụ Subindex (8-bit)
 */
void SDO_Read_Request(uint8_t node_id, uint16_t index, uint8_t sub)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x40; /* Mã lệnh 0x40: Đọc dữ liệu (Upload Request) */
    frame[1] = (uint8_t)(index & 0xFF);
    frame[2] = (uint8_t)((index >> 8) & 0xFF);
    frame[3] = sub;

    _CAN_Send(0x600 + node_id, 8, frame);
}

/**
 * @brief Gửi lệnh quản lý mạng NMT
 * @param node_id ID của node (0 = gửi broadcast tất cả node)
 * @param cmd     0x01: Start Node, 0x02: Stop, 0x80: Enter Pre-Operational, 0x81: Reset Node
 */
void NMT_Send(uint8_t node_id, uint8_t cmd)
{
    uint8_t frame[2] = {cmd, node_id};
    _CAN_Send(0x000, 2, frame);
}

/* ============================================================================
 * PHẦN 5: CẤU HÌNH CÁC KHỐI PDO (RPDO VÀ TPDO)
 * ============================================================================ */

/**
 * @brief Cấu hình RPDO0: Nhận Controlword (Object 0x6040, 16-bit)
 * COB-ID: 0x200 + Node_ID (0x201)
 */
static void _RPDO0_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Vô hiệu hóa RPDO0 để cấu hình: Transmission Type = 0xFE (Async) */
    d[0]=0x2F; d[1]=0x00; d[2]=0x14; d[3]=0x02; d[4]=0xFE; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 2. Map Object 0x6040 (Controlword, 16-bit) vào vị trí Mapping 1 */
    d[0]=0x23; d[1]=0x00; d[2]=0x16; d[3]=0x01; d[4]=0x10; d[5]=0x00; d[6]=0x40; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 3. Kích hoạt: Số object mapping = 1 */
    d[0]=0x2F; d[1]=0x00; d[2]=0x16; d[3]=0x00; d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}

/**
 * @brief Cấu hình RPDO1: Nhận Target Velocity đồng thời cho 2 motor
 * COB-ID: 0x300 + Node_ID (0x301)
 * Map: Object 0x60FF sub 03 (32-bit chứa cả 2 motor)
 */
static void _RPDO1_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Vô hiệu hóa RPDO1 */
    d[0]=0x2F; d[1]=0x01; d[2]=0x14; d[3]=0x02; d[4]=0xFE; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 2. Map 0x60FF sub 03 (32-bit) */
    d[0]=0x23; d[1]=0x01; d[2]=0x16; d[3]=0x01; d[4]=0x20; d[5]=0x03; d[6]=0xFF; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 3. Kích hoạt: Số object mapping = 1 */
    d[0]=0x2F; d[1]=0x01; d[2]=0x16; d[3]=0x00; d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}

/**
 * @brief Cấu hình TPDO0: Driver tự động gửi VẬN TỐC THỰC TẾ (0x606C sub 01 & 02)
 * COB-ID: 0x180 + Node_ID (0x181) | Chu kỳ: 20ms (40 * 0.5ms = 20ms)
 */
static void _TPDO0_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Xóa mapping cũ */
    d[0]=0x2F; d[1]=0x00; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 2. Map 0x606C sub 01 (Vận tốc Motor A - 32 bit) */
    d[0]=0x23; d[1]=0x00; d[2]=0x1A; d[3]=0x01; d[4]=0x20; d[5]=0x01; d[6]=0x6C; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 3. Map 0x606C sub 02 (Vận tốc Motor B - 32 bit) */
    d[0]=0x23; d[1]=0x00; d[2]=0x1A; d[3]=0x02; d[4]=0x20; d[5]=0x02; d[6]=0x6C; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 4. Cài chế độ Timer (0xFF) */
    d[0]=0x2F; d[1]=0x00; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 5. Chu kỳ gửi: 20ms (40 * 0.5ms = 0x28) */
    d[0]=0x2B; d[1]=0x00; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 6. Kích hoạt 2 object mapping */
    d[0]=0x2F; d[1]=0x00; d[2]=0x1A; d[3]=0x00; d[4]=0x02; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}

/**
 * @brief Cấu hình TPDO1: Driver tự động gửi DÒNG ĐIỆN / MOMENT THỰC TẾ (0x6077 sub 03)
 * COB-ID: 0x280 + Node_ID (0x281) | Chu kỳ: 20ms (40 * 0.5ms = 20ms)
 * Object 0x6077 sub 03 (32-bit): Byte 0-1 là Motor A, Byte 2-3 là Motor B (đơn vị: 0.1A)
 */
static void _TPDO1_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Xóa mapping cũ */
    d[0]=0x2F; d[1]=0x01; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 2. Map 0x6077 sub 03 (32-bit chứa dòng cả 2 motor) */
    d[0]=0x23; d[1]=0x01; d[2]=0x1A; d[3]=0x01; d[4]=0x20; d[5]=0x03; d[6]=0x77; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 3. Cài chế độ Timer (0xFF) */
    d[0]=0x2F; d[1]=0x01; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 4. Chu kỳ gửi: 20ms (40 * 0.5ms = 0x28) */
    d[0]=0x2B; d[1]=0x01; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 5. Kích hoạt 1 object mapping */
    d[0]=0x2F; d[1]=0x01; d[2]=0x1A; d[3]=0x00; d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}

/**
 * @brief Cấu hình TPDO2: Driver tự động gửi VỊ TRÍ ENCODER (0x6064 sub 01 & 02)
 * COB-ID: 0x380 + Node_ID (0x381) | Chu kỳ: 20ms (40 * 0.5ms = 20ms)
 */
static void _TPDO2_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Xóa mapping cũ */
    d[0]=0x2F; d[1]=0x02; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 2. Map 0x6064 sub 01 (Encoder Motor A - 32 bit) */
    d[0]=0x23; d[1]=0x02; d[2]=0x1A; d[3]=0x01; d[4]=0x20; d[5]=0x01; d[6]=0x64; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 3. Map 0x6064 sub 02 (Encoder Motor B - 32 bit) */
    d[0]=0x23; d[1]=0x02; d[2]=0x1A; d[3]=0x02; d[4]=0x20; d[5]=0x02; d[6]=0x64; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 4. Cài chế độ Timer (0xFF) */
    d[0]=0x2F; d[1]=0x02; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 5. Chu kỳ gửi: 20ms (40 * 0.5ms = 0x28) */
    d[0]=0x2B; d[1]=0x02; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    /* 6. Kích hoạt 2 object mapping */
    d[0]=0x2F; d[1]=0x02; d[2]=0x1A; d[3]=0x00; d[4]=0x02; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}

/* ============================================================================
 * PHẦN 6: KHỞI TẠO CHẾ ĐỘ VẬN TỐC & BẬT SERVO
 * ============================================================================ */

/**
 * @brief Khởi tạo chế độ Profile Velocity (Mode 3) và cài đặt gia tốc / giảm tốc
 */
static void _ProfileVelocity_Init(uint8_t id)
{
    /* Cài đặt chế độ vận hành: Profile Velocity (Mode 3) */
    SDO_Write(id, 0x6060, 0x00, 0x03, 1);
    HAL_Delay(5);

    /* Cài đặt gia tốc tăng tốc Profile Acceleration (0x6083) = 400ms */
    SDO_Write(id, 0x6083, 0x01, 400, 4);   /* Motor A */
    HAL_Delay(5);
    SDO_Write(id, 0x6083, 0x02, 400, 4);   /* Motor B */
    HAL_Delay(5);

    /* Cài đặt giảm tốc phanh dừng Profile Deceleration (0x6084) = 200ms */
    SDO_Write(id, 0x6084, 0x01, 200, 4);   /* Motor A */
    HAL_Delay(5);
    SDO_Write(id, 0x6084, 0x02, 200, 4);   /* Motor B */
    HAL_Delay(5);
}

/**
 * @brief Trình tự kích hoạt Servo (CiA 402 State Machine qua RPDO0)
 */
static void _ZLAC_EnableServo(uint8_t id)
{
    uint8_t d[2];

    /* Bước 1: Lệnh Shutdown (Controlword = 0x0006) */
    d[0] = 0x06; d[1] = 0x00;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(5);

    /* Bước 2: Lệnh Switch On (Controlword = 0x0007) */
    d[0] = 0x07;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(5);

    /* Bước 3: Lệnh Enable Operation (Controlword = 0x000F) -> Đóng relay cấp lực */
    d[0] = 0x0F;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(10);
}

/* ============================================================================
 * PHẦN 7: KHỞI TẠO DRIVER & STATE MACHINE QUẢN LÝ TRẠNG THÁI
 * ============================================================================ */

/**
 * @brief Khởi tạo phần cứng CAN, cấu hình bộ lọc nhận tất cả frame và kích hoạt ngắt
 */
void ZLAC_CAN_Init(void)
{
    extern CAN_HandleTypeDef hcan;
    CAN_FilterTypeDef filter;

    /* Cấu hình bộ lọc CAN: Nhận toàn bộ ID (mask = 0) */
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;  /* Mask = 0: cho phép tất cả các frame đi qua */
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;

    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);

    /* Xóa sạch các biến phản hồi và khởi động State Machine từ bước chờ driver đồng bộ */
    memset(&zlac_fb,   0, sizeof(zlac_fb));
    memset(&zlac_odom, 0, sizeof(zlac_odom));
    zlac_state = ZLAC_WAIT_HB;
    state_entry_tick = HAL_GetTick();
}

/**
 * @brief State Machine quản lý khởi động, bảo vệ an toàn kẹt tải và tự động phục hồi
 * Gọi liên tục trong vòng lặp while(1) của main()
 */
void ZLAC_StateMachine(void)
{
    uint8_t id = ZLAC_NODE_ID;
    uint32_t now = HAL_GetTick();

    switch (zlac_state)
    {
        /* -------------------------------------------------------------
         * TRẠNG THÁI 0: Chờ nhận Bootup/Heartbeat từ ZLAC hoặc Timeout khởi động
         * ------------------------------------------------------------- */
        case ZLAC_WAIT_HB:
            /* Điều kiện 1: Nhận được bản tin Bootup (0x701) hoặc Heartbeat từ ZLAC */
            if (zlac_fb.hb_received)
            {
                zlac_fb.hb_received = 0;
                zlac_state = ZLAC_CONFIG;
                state_entry_tick = now;
            }
            /* Điều kiện 2: Timeout 1000ms dự phòng nếu ZLAC đã bật trước đó */
            else if ((now - state_entry_tick) >= 1000)
            {
                zlac_state = ZLAC_CONFIG;
                state_entry_tick = now;
            }
            break;

        /* -------------------------------------------------------------
         * TRẠNG THÁI 1: Cấu hình RPDO, TPDO, thông số gia tốc và Start Node
         * ------------------------------------------------------------- */
        case ZLAC_CONFIG:
            /* Chuẩn CANopen DS301: Đưa ZLAC về Pre-Operational để chấp nhận cấu hình PDO */
            NMT_Send(id, 0x80);
            HAL_Delay(10);

            _RPDO0_Config(id);            /* Cấu hình nhận Controlword (0x201) */
            _RPDO1_Config(id);            /* Cấu hình nhận Target Velocity (0x301) */
            _TPDO0_Config(id);            /* Bật tự động gửi Vận tốc thực tế (0x181 - 20ms) */
            _TPDO1_Config(id);            /* Bật tự động gửi Dòng điện thực tế (0x281 - 20ms) */
            _TPDO2_Config(id);            /* Bật tự động gửi Vị trí Encoder (0x381 - 20ms) */
            _ProfileVelocity_Init(id);    /* Cài đặt Mode 3, Accel 400ms, Decel 200ms */
            NMT_Send(id, 0x01);           /* Start Node -> Chuyển ZLAC sang trạng thái Operational */
            HAL_Delay(50);
            zlac_state = ZLAC_ENABLING;
            state_entry_tick = HAL_GetTick();
            break;

        /* -------------------------------------------------------------
         * TRẠNG THÁI 2: Bật Servo (Enable Operation)
         * ------------------------------------------------------------- */
        case ZLAC_ENABLING:
            _ZLAC_EnableServo(id);
            zlac_state = ZLAC_READY;
            state_entry_tick = HAL_GetTick();
            zlac_fb.last_rx_tick = HAL_GetTick();
            break;

        /* -------------------------------------------------------------
         * TRẠNG THÁI 3: Sẵn sàng vận hành & Giám sát an toàn (Stall / Overcurrent)
         * ------------------------------------------------------------- */
        case ZLAC_READY:
            /* 1. CAN Link Watchdog: Mất kết nối CAN quá 1.5 giây -> Phanh dừng xe an toàn */
            if ((now - zlac_fb.last_rx_tick) > 1500)
            {
                ZLAC_Stop();
                zlac_fb.vel_a = 0;
                zlac_fb.vel_b = 0;
                zlac_fb.error_code = ZLAC_ERR_CAN_TIMEOUT;
                zlac_state = ZLAC_WAIT_HB;
                state_entry_tick = now;
                break;
            }

            /* 2. Kiểm tra mã lỗi phần cứng từ driver (TPDO3 CAN ID 0x481) */
            if (zlac_fb.error_code != 0)
            {
                ZLAC_Stop();
                zlac_state = ZLAC_FAULT;
                state_entry_tick = now;
                break;
            }

            /* 2. CƠ CHẾ BẢO VỆ KẸT TẢI CƠ KHÍ & QUÁ DÒNG (STALL PROTECTION) */
            {
                static uint32_t stall_timer = 0;
                int16_t abs_i_a = (zlac_fb.current_a < 0) ? -zlac_fb.current_a : zlac_fb.current_a;
                int16_t abs_i_b = (zlac_fb.current_b < 0) ? -zlac_fb.current_b : zlac_fb.current_b;
                int16_t abs_v_a = (zlac_fb.vel_a < 0) ? -zlac_fb.vel_a : zlac_fb.vel_a;
                int16_t abs_v_b = (zlac_fb.vel_b < 0) ? -zlac_fb.vel_b : zlac_fb.vel_b;

                /* Tình huống A: Quá dòng cực đại tức thời (> 12.0A) -> Ngắt xung lập tức */
                if (abs_i_a > ZLAC_OVERCURRENT_THRESHOLD || abs_i_b > ZLAC_OVERCURRENT_THRESHOLD)
                {
                    ZLAC_Stop();
                    zlac_fb.error_code = ZLAC_ERR_STALL_OVERCURRENT;
                    zlac_state = ZLAC_FAULT;
                    state_entry_tick = now;
                    stall_timer = 0;
                    break;
                }

                /* Tình huống B: Kẹt cơ khí (Dòng cao > 6.0A nhưng bánh đứng yên < 3.0 RPM) */
                if ((abs_i_a > ZLAC_STALL_CURRENT_THRESHOLD && abs_v_a < ZLAC_STALL_VELOCITY_THRESHOLD) ||
                    (abs_i_b > ZLAC_STALL_CURRENT_THRESHOLD && abs_v_b < ZLAC_STALL_VELOCITY_THRESHOLD))
                {
                    if (stall_timer == 0)
                    {
                        stall_timer = now;
                    }
                    else if ((now - stall_timer) >= ZLAC_STALL_TIMEOUT_MS)
                    {
                        /* Kẹt liên tục quá 400ms -> Ngắt động cơ khẩn cấp để chống cháy cuộn dây */
                        ZLAC_Stop();
                        zlac_fb.error_code = ZLAC_ERR_STALL_OVERCURRENT;
                        zlac_state = ZLAC_FAULT;
                        state_entry_tick = now;
                        stall_timer = 0;
                        break;
                    }
                }
                else
                {
                    stall_timer = 0; /* Reset timer khi dòng điện hoặc vận tốc quay trở lại bình thường */
                }
            }
            break;

        /* -------------------------------------------------------------
         * TRẠNG THÁI 4: Xử lý lỗi & Tự động phục hồi
         * ------------------------------------------------------------- */
        case ZLAC_FAULT:
            /* Chờ 2 giây để động cơ dừng hẳn và tản nhiệt, sau đó thử xả lỗi */
            if ((now - state_entry_tick) > 2000)
            {
                uint8_t d[2] = {0x80, 0x00};  /* Fault Reset qua Controlword bit 7 */
                _CAN_Send(0x200 + id, 2, d);
                HAL_Delay(100);
                zlac_fb.error_code = 0;        /* Xóa cờ lỗi */
                zlac_state = ZLAC_ENABLING;    /* Thử bật lại servo nếu nguyên nhân kẹt đã được loại bỏ */
                state_entry_tick = HAL_GetTick();
            }
            break;
    }
}

/**
 * @brief Kiểm tra driver đã sẵn sàng nhận lệnh điều khiển hay chưa
 * @return true nếu driver đang ở trạng thái ZLAC_READY
 */
bool ZLAC_IsReady(void)
{
    return (zlac_state == ZLAC_READY);
}

/* ============================================================================
 * PHẦN 8: CÁC HÀM ĐIỀU KHIỂN CHUYỂN ĐỘNG
 * ============================================================================ */

/**
 * @brief Đặt vận tốc trực tiếp cho từng bánh xe (đơn vị: 1 RPM)
 * @param vel_a Vận tốc Motor A (RPM)
 * @param vel_b Vận tốc Motor B (RPM)
 */
void ZLAC_SetSpeed_raw(int16_t vel_a, int16_t vel_b)
{
    g_vel_a = vel_a;
    g_vel_b = vel_b;

#if ZLAC_MOTOR_B_REVERSE
    vel_b = -vel_b;   /* Đảo dấu vận tốc motor B nếu lắp đối xứng ngược chiều */
#endif

    /* Sử dụng giao thức SDO ghi vào 0x60FF: chuẩn xác 100% không bị sót frame */
    SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x01, (int32_t)vel_a, 4);
    HAL_Delay(2);     /* Nghỉ 2ms giữa 2 bánh */
    SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x02, (int32_t)vel_b, 4);
}

/**
 * @brief Đặt vận tốc robot theo mô hình vi sai (Differential Drive Kinematics)
 * @param v     Vận tốc dài tịnh tiến (m/s), dương = tiến, âm = lùi
 * @param omega Vận tốc góc quay (rad/s), dương = quay trái, âm = quay phải
 */
void ZLAC_SetSpeed_mps(float v, float omega)
{
    /* Giới hạn vận tốc tịnh tiến trong ngưỡng an toàn */
    v = CLAMP(v, -ZLAC_MAX_SPEED_MPS, ZLAC_MAX_SPEED_MPS);

    /* Động học vi sai thuận: Tính vận tốc bánh trái (vL) và bánh phải (vR) */
    float vL = v - (omega * ZLAC_WHEELBASE_M / 2.0f);
    float vR = v + (omega * ZLAC_WHEELBASE_M / 2.0f);

    /* Co giãn tỉ lệ (scale) nếu có bánh vượt quá vận tốc tối đa cho phép */
    float v_max = ZLAC_MAX_SPEED_MPS;
    float max_v = fmaxf(fabsf(vL), fabsf(vR));
    if (max_v > v_max) {
        float scale = v_max / max_v;
        vL *= scale;
        vR *= scale;
    }

    /* Đổi từ m/s sang vòng/phút (RPM) */
    float rpm_L = (vL / ZLAC_WHEEL_RADIUS_M) * (60.0f / (2.0f * M_PI));
    float rpm_R = (vR / ZLAC_WHEEL_RADIUS_M) * (60.0f / (2.0f * M_PI));

    /* Đổi từ RPM sang đơn vị gửi driver ZLAC (đơn vị: 1 RPM) */
    int16_t zlac_L = (int16_t)roundf(rpm_L);
    int16_t zlac_R = (int16_t)roundf(rpm_R);

    ZLAC_SetSpeed_raw(zlac_L, zlac_R);
}

/**
 * @brief Dừng robot khẩn cấp (gửi lệnh vận tốc bằng 0)
 */
void ZLAC_Stop(void)
{
    ZLAC_SetSpeed_raw(0, 0);
}

/* ============================================================================
 * PHẦN 9: CÁC HÀM TÍNH TOÁN ODOMETRY & VẬN TỐC THỰC TẾ
 * ============================================================================ */

/**
 * @brief Lấy vận tốc thực tế bánh A (m/s)
 */
float ZLAC_GetVelA_mps(void)
{
    float rpm = (float)zlac_fb.vel_a / 10.0f;
    return rpm * (2.0f * M_PI / 60.0f) * ZLAC_WHEEL_RADIUS_M;
}

/**
 * @brief Lấy vận tốc thực tế bánh B (m/s)
 */
float ZLAC_GetVelB_mps(void)
{
    float rpm = (float)zlac_fb.vel_b / 10.0f;
    float v = rpm * (2.0f * M_PI / 60.0f) * ZLAC_WHEEL_RADIUS_M;
#if ZLAC_MOTOR_B_REVERSE
    v = -v;   /* Đảo dấu nếu motor B lắp đối xứng */
#endif
    return v;
}

/**
 * @brief Cập nhật tọa độ Odometry dựa trên sự dịch chuyển của xung Encoder
 * @param dt Khoảng thời gian (giây) giữa 2 lần gọi
 */
void ZLAC_Odom_Update(float dt)
{
    static int32_t last_pos_a = 0;
    static int32_t last_pos_b = 0;
    static uint8_t first_time = 1;

    if (dt <= 0.0f || dt > 0.5f) return;

    /* Lần đầu tiên gọi hàm: Lưu mốc ban đầu làm điểm gốc */
    if (first_time) {
        last_pos_a = zlac_fb.pos_a;
        last_pos_b = zlac_fb.pos_b;
        first_time = 0;
        return;
    }

    /* Tính độ chênh lệch xung Encoder kể từ chu kỳ trước */
    int32_t delta_a = zlac_fb.pos_a - last_pos_a;
    int32_t delta_b = zlac_fb.pos_b - last_pos_b;

    last_pos_a = zlac_fb.pos_a;
    last_pos_b = zlac_fb.pos_b;

    /* Giả định độ phân giải encoder là 4096 xung/vòng (tùy chỉnh theo motor thực tế) */
    float meter_per_count = (2.0f * M_PI * ZLAC_WHEEL_RADIUS_M) / 4096.0f;

    /* Quãng đường lăn thực tế của từng bánh (m) */
    float dist_L = delta_a * meter_per_count;
    float dist_R = delta_b * meter_per_count;

#if ZLAC_MOTOR_B_REVERSE
    dist_R = -dist_R; /* Tự động đảo dấu nếu motor B lắp đối xứng */
#endif

    /* Động học: Quãng đường tâm xe đi được và góc quay của thân xe */
    float dist_center = (dist_L + dist_R) / 2.0f;
    float d_theta     = (dist_R - dist_L) / ZLAC_WHEELBASE_M;

    /* Cập nhật tọa độ vị trí X, Y và góc Theta */
    float theta_mid = zlac_odom.theta + d_theta / 2.0f;
    zlac_odom.x     += dist_center * cosf(theta_mid);
    zlac_odom.y     += dist_center * sinf(theta_mid);
    zlac_odom.theta += d_theta;

    /* Chuẩn hóa góc quay về khoảng [-PI, PI] */
    while (zlac_odom.theta >  M_PI) zlac_odom.theta -= 2.0f * M_PI;
    while (zlac_odom.theta < -M_PI) zlac_odom.theta += 2.0f * M_PI;

    /* Vận tốc tức thời */
    zlac_odom.v     = dist_center / dt;
    zlac_odom.omega = d_theta / dt;
}

/**
 * @brief Đặt lại tọa độ Odometry về mốc 0 (X=0, Y=0, Theta=0, V=0, Omega=0)
 */
void ZLAC_Odom_Reset(void)
{
    memset(&zlac_odom, 0, sizeof(zlac_odom));
}

/* ============================================================================
 * PHẦN 10: HÀM NGẮT NHẬN CAN (HAL_CAN_RxFifo0MsgPendingCallback)
 * ============================================================================ */
/**
 * @brief Callback tự động được phần cứng HAL gọi khi có frame CAN đến FIFO0
 * @note Chạy trong Interrupt Context -> Không dùng HAL_Delay(), không gọi hàm nặng
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_buf[8] = {0};

    /* Lấy message ra khỏi FIFO0 ngay lập tức để tránh tràn bộ đệm */
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_hdr, rx_buf) != HAL_OK)
        return;

    uint32_t id = rx_hdr.StdId;

    /* Cập nhật mốc thời gian nhận dữ liệu từ ZLAC và xóa cờ mất kết nối CAN nếu có */
    if (id == (0x180 + ZLAC_NODE_ID) || id == (0x280 + ZLAC_NODE_ID) ||
        id == (0x380 + ZLAC_NODE_ID) || id == (0x480 + ZLAC_NODE_ID) ||
        id == (0x580 + ZLAC_NODE_ID) || ((id & 0x780) == 0x700))
    {
        zlac_fb.last_rx_tick = HAL_GetTick();
        if (zlac_fb.error_code == ZLAC_ERR_CAN_TIMEOUT)
        {
            zlac_fb.error_code = 0;
        }
    }

    /* --- TPDO0 (CAN ID: 0x181): VẬN TỐC THỰC TẾ 2 MOTOR --- */
    if (id == (0x180 + ZLAC_NODE_ID))
    {
        /* Object 0x606C: Byte 0..3: Motor A (int32), Byte 4..7: Motor B (int32), đơn vị 0.1 rpm */
        zlac_fb.vel_a = (int16_t)RD_I32(rx_buf);
        zlac_fb.vel_b = (int16_t)RD_I32(rx_buf + 4);
    }
    /* --- TPDO1 (CAN ID: 0x281): DÒNG ĐIỆN / MOMENT THỰC TẾ 2 MOTOR --- */
    else if (id == (0x280 + ZLAC_NODE_ID))
    {
        /* Object 0x6077 sub 03: Byte 0..1: Motor A (int16), Byte 2..3: Motor B (int16), đơn vị 0.1A */
        zlac_fb.current_a = RD_I16(rx_buf);
        zlac_fb.current_b = RD_I16(rx_buf + 2);
    }
    /* --- TPDO2 (CAN ID: 0x381): VỊ TRÍ XUNG ENCODER 2 MOTOR --- */
    else if (id == (0x380 + ZLAC_NODE_ID))
    {
        /* Object 0x6064: Byte 0..3: Motor A (int32 counts), Byte 4..7: Motor B (int32 counts) */
        zlac_fb.pos_a = RD_I32(rx_buf);
        zlac_fb.pos_b = RD_I32(rx_buf + 4);
    }
    /* --- TPDO3 (CAN ID: 0x481): MÃ LỖI PHẦN CỨNG DRIVER --- */
    else if (id == (0x480 + ZLAC_NODE_ID))
    {
        zlac_fb.error_code = (uint16_t)(rx_buf[0] | ((uint16_t)rx_buf[1] << 8));
    }
    /* --- NMT Bootup / Heartbeat (CAN ID: 0x700 - 0x77F) --- */
    else if ((id & 0x780) == 0x700)
    {
        zlac_fb.hb_received = 1;
        zlac_fb.hb_tick = HAL_GetTick();

        /* Nếu Driver vừa khởi động lại (bản tin Bootup 0x701) khi đang vận hành -> tự động cấu hình lại */
        if (rx_buf[0] == 0x00 || rx_buf[0] == 0x7F)
        {
            if (zlac_state == ZLAC_READY || zlac_state == ZLAC_FAULT)
            {
                zlac_state = ZLAC_CONFIG;
            }
        }
    }
    /* --- SDO Response (CAN ID: 0x581): Phản hồi từ lệnh đọc SDO --- */
    else if (id == (0x580 + ZLAC_NODE_ID))
    {
        uint16_t idx = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
        uint8_t sub = rx_buf[3];

        /* Object 0x6064: Vị trí Encoder */
        if (idx == 0x6064)
        {
            int32_t enc_val = (int32_t)((uint32_t)rx_buf[4] | ((uint32_t)rx_buf[5] << 8)
                                      | ((uint32_t)rx_buf[6] << 16) | ((uint32_t)rx_buf[7] << 24));
            if (sub == 0x01) zlac_fb.pos_a = enc_val;
            if (sub == 0x02) zlac_fb.pos_b = enc_val;
        }
        /* Object 0x2035: Điện áp DC Bus / Pin (đơn vị: 0.1V hoặc 0.01V) */
        else if (idx == 0x2035)
        {
            zlac_fb.bus_voltage = (uint16_t)rx_buf[4] | ((uint16_t)rx_buf[5] << 8);
        }
    }
}
