/**
 * ============================================================================
 * File: zlac_can.c
 * Mô tả: Implement driver giao tiếp STM32F103 <-> ZLAC8015D qua CANopen
 *
 * Tham khảo từ: ZLAC8015D CANopen Aging V1.0 source code
 *               ZLAC8015D CANopen Quick Start Guide V1.00
 * ============================================================================
 */

#include "zlac_can.h"
#include "can.h"    /* HAL CAN handle (hcan) từ CubeMX */
#include <math.h>   /* sinf, cosf, fabsf */
#include <string.h> /* memset */

/* ============================================================================
 * BIẾN NỘI BỘ
 * ============================================================================ */
ZLAC_Feedback_t  zlac_fb    = {0};
ZLAC_Odometry_t  zlac_odom  = {0};
ZLAC_State_t     zlac_state = ZLAC_WAIT_HB;

/* Thời điểm bắt đầu trạng thái hiện tại (để timeout) */
static uint32_t state_entry_tick = 0;

/* Biến lưu lệnh tốc độ hiện tại (để retry nếu cần) */
static int16_t g_vel_a = 0, g_vel_b = 0;

/* ============================================================================
 * HELPER MACROS
 * ============================================================================ */
#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* Clamp giá trị vào [lo, hi] */
#define CLAMP(x, lo, hi)  ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))

/* Đọc int16 Little-Endian từ byte array */
#define RD_I16(b)   ((int16_t)((uint16_t)(b)[0] | ((uint16_t)(b)[1] << 8)))
#define RD_I32(b)   ((int32_t)((uint32_t)(b)[0] | ((uint32_t)(b)[1]<<8) \
                              | ((uint32_t)(b)[2]<<16)| ((uint32_t)(b)[3]<<24)))

/* ============================================================================
 * HÀM NỘI BỘ: Gửi 1 CAN frame
 * ============================================================================ */
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

    /* Chờ mailbox tối đa 5ms */
    uint32_t t = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        if (HAL_GetTick() - t > 5) return HAL_TIMEOUT;
    }

    return HAL_CAN_AddTxMessage(&hcan, &hdr, data, &mailbox);
}

/* ============================================================================
 * API CÔNG KHAI: SDO_Write – Ghi 1 object vào OD của ZLAC
 * ============================================================================ */
void SDO_Write(uint8_t node_id, uint16_t index, uint8_t sub,
               int32_t value, uint8_t size_bytes)
{
    /*
     * SDO Write Request:
     * Byte 0: Command specifier (cs)
     *   0x2F = 1 byte, 0x2B = 2 bytes, 0x27 = 3 bytes, 0x23 = 4 bytes
     * Byte 1: Index Low
     * Byte 2: Index High
     * Byte 3: Subindex
     * Byte 4-7: Value (Little-Endian, zero-padded)
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
/* ============================================================================
 * API CÔNG KHAI: Yêu cầu đọc dữ liệu
 * ============================================================================ */
void SDO_Read_Request(uint8_t node_id, uint16_t index, uint8_t sub)
{
    uint8_t frame[8] = {0};
    frame[0] = 0x40; /* Mã lệnh 0x40: Đọc dữ liệu (Read Request) */
    frame[1] = (uint8_t)(index & 0xFF);
    frame[2] = (uint8_t)((index >> 8) & 0xFF);
    frame[3] = sub;
    /* Byte 4 đến 7 để trống (bằng 0) khi gửi lệnh Đọc */
    
    _CAN_Send(0x600 + node_id, 8, frame);
}

/* ============================================================================
 * API CÔNG KHAI: NMT_Send
 * ============================================================================ */
void NMT_Send(uint8_t node_id, uint8_t cmd)
{
    uint8_t frame[2] = {cmd, node_id};
    _CAN_Send(0x000, 2, frame);
}

/* ============================================================================
 * HÀM NỘI BỘ: Cấu hình RPDO0 – Nhận Controlword (0x6040)
 * COB-ID: 0x200 + Node_ID
 * ============================================================================ */
static void _RPDO0_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Disable RPDO0: Set Transmission Type = 0xFE (Async) */
    d[0]=0x2F; d[1]=0x00; d[2]=0x14; d[3]=0x02;  /* Idx=0x1400, Sub=02 */
    d[4]=0xFE; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);

    /* 2. Map Object 0x6040 (Controlword, 16-bit) vào Mapping Entry 1 */
    d[0]=0x23; d[1]=0x00; d[2]=0x16; d[3]=0x01;  /* Idx=0x1600, Sub=01 */
    d[4]=0x10; d[5]=0x00; d[6]=0x40; d[7]=0x60;  /* 0x60400010 LE */
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);

    /* 3. Set số object mapped = 1 */
    d[0]=0x2F; d[1]=0x00; d[2]=0x16; d[3]=0x00;  /* Idx=0x1600, Sub=00 */
    d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);
}

/* ============================================================================
 * HÀM NỘI BỘ: Cấu hình RPDO1 – Nhận Target Velocity (cả 2 motor)
 * COB-ID: 0x300 + Node_ID
 * Map: 0x60FF sub 0x03 (int32 chứa cả 2 motor ×10)
 * ============================================================================ */
static void _RPDO1_Config(uint8_t id)
{
    uint8_t d[8];

    /* 1. Disable RPDO1 */
    d[0]=0x2F; d[1]=0x01; d[2]=0x14; d[3]=0x02;  /* Idx=0x1401, Sub=02 */
    d[4]=0xFE; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);

    /* 2. Map 0x60FF sub 0x03 (32-bit = cả 2 motor) */
    d[0]=0x23; d[1]=0x01; d[2]=0x16; d[3]=0x01;  /* Idx=0x1601, Sub=01 */
    d[4]=0x20; d[5]=0x03; d[6]=0xFF; d[7]=0x60;  /* 0x60FF0320 LE */
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);

    /* 3. Set số object mapped = 1 */
    d[0]=0x2F; d[1]=0x01; d[2]=0x16; d[3]=0x00;
    d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600+id, 8, d);
    HAL_Delay(5);
}
/* ============================================================
 * Cấu hình TPDO0: Driver tự động bắn VẬN TỐC (0x606C sub 03) về 0x181
 * Chu kỳ: 50ms (Timer trigger)
 * ============================================================ */
static void _TPDO0_Config(uint8_t id)
{
    uint8_t d[8];
    // 1. Xóa mapping cũ
    d[0]=0x2F; d[1]=0x00; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 2. Map 0x606C sub 01 (Vel A - 32 bit)
    d[0]=0x23; d[1]=0x00; d[2]=0x1A; d[3]=0x01;
    d[4]=0x20; d[5]=0x01; d[6]=0x6C; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 3. Map 0x606C sub 02 (Vel B - 32 bit)
    d[0]=0x23; d[1]=0x00; d[2]=0x1A; d[3]=0x02;
    d[4]=0x20; d[5]=0x02; d[6]=0x6C; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 4. Cài chế độ Timer (0xFF)
    d[0]=0x2F; d[1]=0x00; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 5. Chu kỳ gửi: 50ms (100 * 0.5ms)
    d[0]=0x2B; d[1]=0x00; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 6. Kích hoạt 2 object mapping
    d[0]=0x2F; d[1]=0x00; d[2]=0x1A; d[3]=0x00; d[4]=0x02; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}
/* ============================================================
 * Cấu hình TPDO1: Driver tự động bắn DÒNG ĐIỆN / MOMENT (0x6077 sub 03) về 0x281
 * Chu kỳ: 20ms (40 * 0.5ms = 20ms) đồng bộ 50Hz
 * ============================================================ */
static void _TPDO1_Config(uint8_t id)
{
    uint8_t d[8];
    // 1. Xóa mapping cũ
    d[0]=0x2F; d[1]=0x01; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    // 2. Map 0x6077 sub 03 (32 bit: chứa Torque/Dòng điện cả 2 motor A và B)
    d[0]=0x23; d[1]=0x01; d[2]=0x1A; d[3]=0x01;
    d[4]=0x20; d[5]=0x03; d[6]=0x77; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    // 3. Cài chế độ Timer (0xFF)
    d[0]=0x2F; d[1]=0x01; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    // 4. Chu kỳ gửi: 20ms (40 * 0.5ms = 20ms)
    d[0]=0x2B; d[1]=0x01; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);

    // 5. Kích hoạt 1 object mapping
    d[0]=0x2F; d[1]=0x01; d[2]=0x1A; d[3]=0x00; d[4]=0x01; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}
/* ============================================================
 * Cấu hình TPDO2: Driver tự động bắn VỊ TRÍ ENCODER (0x6064) về 0x381
 * Chu kỳ: 50ms (Timer trigger)
 * ============================================================ */
static void _TPDO2_Config(uint8_t id)
{
    uint8_t d[8];
    // 1. Xóa mapping cũ
    d[0]=0x2F; d[1]=0x02; d[2]=0x1A; d[3]=0x00; d[4]=0x00; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 2. Map 0x6064 sub 01 (Pos A - 32 bit)
    d[0]=0x23; d[1]=0x02; d[2]=0x1A; d[3]=0x01;
    d[4]=0x20; d[5]=0x01; d[6]=0x64; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 3. Map 0x6064 sub 02 (Pos B - 32 bit)
    d[0]=0x23; d[1]=0x02; d[2]=0x1A; d[3]=0x02;
    d[4]=0x20; d[5]=0x02; d[6]=0x64; d[7]=0x60;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 4. Cài chế độ Timer (0xFF)
    d[0]=0x2F; d[1]=0x02; d[2]=0x18; d[3]=0x02; d[4]=0xFF; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 5. Chu kỳ gửi: 50ms (100 * 0.5ms)
    d[0]=0x2B; d[1]=0x02; d[2]=0x18; d[3]=0x05; d[4]=0x28; d[5]=0x00; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
    // 6. Kích hoạt 2 object mapping
    d[0]=0x2F; d[1]=0x02; d[2]=0x1A; d[3]=0x00; d[4]=0x02; d[5]=0; d[6]=0; d[7]=0;
    _CAN_Send(0x600 + id, 8, d);
    HAL_Delay(5);
}
/* ============================================================================
 * HÀM NỘI BỘ: Khởi tạo chế độ Profile Velocity
 * ============================================================================ */
static void _ProfileVelocity_Init(uint8_t id)
{
    /* Set mode = Profile Velocity (0x03) */
    SDO_Write(id, 0x6060, 0x00, 0x03, 1);
    HAL_Delay(5);

    /* Gia tốc Motor A và B (0x6083 sub 01 và 02) */
    SDO_Write(id, 0x6083, 0x01, 400, 4);   /* Motor A */
    HAL_Delay(5);
    SDO_Write(id, 0x6083, 0x02, 400, 4);   /* Motor B */
    HAL_Delay(5);

    /* Giảm tốc (0x6084) */
    SDO_Write(id, 0x6084, 0x01, 200, 4);
    HAL_Delay(5);
    SDO_Write(id, 0x6084, 0x02, 200, 4);
    HAL_Delay(5);
}

/* ============================================================================
 * HÀM NỘI BỘ: Bật servo (Controlword sequence qua RPDO0)
 * ============================================================================ */
static void _ZLAC_EnableServo(uint8_t id)
{
    uint8_t d[2];

    /* Shutdown: Controlword = 0x0006 */
    d[0] = 0x06; d[1] = 0x00;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(5);

    /* Switch On: 0x0007 */
    d[0] = 0x07;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(5);

    /* Enable Operation: 0x000F */
    d[0] = 0x0F;
    _CAN_Send(0x200 + id, 2, d);
    HAL_Delay(10);
}

/* ============================================================================
 * API CÔNG KHAI: ZLAC_CAN_Init
 * ============================================================================ */
void ZLAC_CAN_Init(void)
{
    extern CAN_HandleTypeDef hcan;
    CAN_FilterTypeDef filter;

    /* Filter: Nhận tất cả frame (mask = 0) */
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;  /* 0 = không lọc */
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;

    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);

    /* Reset trạng thái */
    memset(&zlac_fb,   0, sizeof(zlac_fb));
    memset(&zlac_odom, 0, sizeof(zlac_odom));
    zlac_state = ZLAC_CONFIG;
    state_entry_tick = HAL_GetTick();
}

/* ============================================================================
 * API CÔNG KHAI: ZLAC_StateMachine
 * Gọi trong vòng lặp main hoặc FreeRTOS task mỗi 10-50ms
 * ============================================================================ */
void ZLAC_StateMachine(void)
{
    uint8_t id = ZLAC_NODE_ID;
    uint32_t now = HAL_GetTick();

    switch (zlac_state)
    {
        /* ─────────────────────────────────────────────
         * Chờ Heartbeat từ ZLAC
         * ───────────────────────────────────────────── */
        case ZLAC_WAIT_HB:
            if (zlac_fb.hb_received &&
                (now - zlac_fb.hb_tick) < 3000)  /* Heartbeat trong vòng 3s */
            {
                zlac_fb.hb_received = 0;
                zlac_state = ZLAC_CONFIG;
                state_entry_tick = now;
            }
            break;

        /* ─────────────────────────────────────────────
         * Cấu hình PDO và mode (chạy 1 lần)
         * ───────────────────────────────────────────── */
        case ZLAC_CONFIG:
            _RPDO0_Config(id);            /* ~15ms */
            _RPDO1_Config(id);            /* ~15ms */
            _TPDO0_Config(id);            /* Bật tự động gửi Vận tốc (0x181) */
            _TPDO1_Config(id);            /* Bật tự động gửi Dòng điện (0x281) */
            _TPDO2_Config(id);            /* Bật tự động gửi Vị trí (0x381) */
            _ProfileVelocity_Init(id);    /* ~25ms */
            NMT_Send(id, 0x01);           /* Start node */
            HAL_Delay(50);
            zlac_state = ZLAC_ENABLING;
            state_entry_tick = HAL_GetTick();
            break;

        /* ─────────────────────────────────────────────
         * Bật servo
         * ───────────────────────────────────────────── */
        case ZLAC_ENABLING:
            _ZLAC_EnableServo(id);
            zlac_state = ZLAC_READY;
            state_entry_tick = HAL_GetTick();
            break;

        /* ─────────────────────────────────────────────
         * Đang chạy
         * ───────────────────────────────────────────── */
        case ZLAC_READY:
            /* 1. Kiểm tra mã lỗi phần cứng từ driver (TPDO3 0x481) */
            if (zlac_fb.error_code != 0)
            {
                ZLAC_Stop();
                zlac_state = ZLAC_FAULT;
                state_entry_tick = now;
                break;
            }

            /* 2. BẢO VỆ KẸT TẢI & QUÁ DÒNG (Stall & Overcurrent Protection) */
            {
                static uint32_t stall_timer = 0;
                int16_t abs_i_a = (zlac_fb.current_a < 0) ? -zlac_fb.current_a : zlac_fb.current_a;
                int16_t abs_i_b = (zlac_fb.current_b < 0) ? -zlac_fb.current_b : zlac_fb.current_b;
                int16_t abs_v_a = (zlac_fb.vel_a < 0) ? -zlac_fb.vel_a : zlac_fb.vel_a;
                int16_t abs_v_b = (zlac_fb.vel_b < 0) ? -zlac_fb.vel_b : zlac_fb.vel_b;

                /* Quá dòng nguy hiểm tức thời (> 12.0A) -> Ngắt xung lập tức */
                if (abs_i_a > ZLAC_OVERCURRENT_THRESHOLD || abs_i_b > ZLAC_OVERCURRENT_THRESHOLD)
                {
                    ZLAC_Stop();
                    zlac_fb.error_code = ZLAC_ERR_STALL_OVERCURRENT;
                    zlac_state = ZLAC_FAULT;
                    state_entry_tick = now;
                    stall_timer = 0;
                    break;
                }

                /* Kẹt cơ khí: Dòng cao (> 6.0A) nhưng bánh đứng yên (< 3.0 RPM) */
                if ((abs_i_a > ZLAC_STALL_CURRENT_THRESHOLD && abs_v_a < ZLAC_STALL_VELOCITY_THRESHOLD) ||
                    (abs_i_b > ZLAC_STALL_CURRENT_THRESHOLD && abs_v_b < ZLAC_STALL_VELOCITY_THRESHOLD))
                {
                    if (stall_timer == 0)
                    {
                        stall_timer = now;
                    }
                    else if ((now - stall_timer) >= ZLAC_STALL_TIMEOUT_MS)
                    {
                        /* Kẹt liên tục quá 400ms -> Ngắt động cơ an toàn */
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
                    stall_timer = 0; /* Reset timer khi dòng hoặc vận tốc bình thường */
                }
            }
            break;

        /* ─────────────────────────────────────────────
         * Xử lý lỗi
         * ───────────────────────────────────────────── */
        case ZLAC_FAULT:
            /* Chờ 2 giây rồi thử xả lỗi và kích hoạt lại nếu đã hết kẹt */
            if ((now - state_entry_tick) > 2000)
            {
                uint8_t d[2] = {0x80, 0x00};  /* Fault Reset */
                _CAN_Send(0x200 + id, 2, d);
                HAL_Delay(100);
                zlac_fb.error_code = 0;        /* Xóa cờ lỗi */
                zlac_state = ZLAC_ENABLING;    /* Thử Enable lại */
                state_entry_tick = HAL_GetTick();
            }
            break;
    }
}

/* ============================================================================
 * API CÔNG KHAI: Kiểm tra sẵn sàng
 * ============================================================================ */
bool ZLAC_IsReady(void)
{
    return (zlac_state == ZLAC_READY);
}

/* ============================================================================
 * API CÔNG KHAI: ZLAC_SetSpeed_raw – Gửi tốc độ trực tiếp (rpm×10)
 * ============================================================================ */
void ZLAC_SetSpeed_raw(int16_t vel_a, int16_t vel_b)
{
    g_vel_a = vel_a;
    g_vel_b = vel_b;
#if ZLAC_MOTOR_B_REVERSE
    vel_b = -vel_b;   /* Đảo dấu motor B nếu lắp ngược */
#endif
    // Dùng SDO chuẩn xác 100% của ZLAC
    SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x01, (int32_t)vel_a, 4);
    HAL_Delay(2); // Chỉ cần nghỉ nhẹ 2ms
    SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x02, (int32_t)vel_b, 4);
}
/* ============================================================================
 * API CÔNG KHAI: ZLAC_SetSpeed_mps – Tốc độ robot (m/s, rad/s)
 * ============================================================================ */
void ZLAC_SetSpeed_mps(float v, float omega)
{
    /* Clamp tốc độ tịnh tiến */
    v = CLAMP(v, -ZLAC_MAX_SPEED_MPS, ZLAC_MAX_SPEED_MPS);

    /* Forward kinematics: v, omega → vL (m/s), vR (m/s) */
    float vL = v - (omega * ZLAC_WHEELBASE_M / 2.0f);
    float vR = v + (omega * ZLAC_WHEELBASE_M / 2.0f);

    /* Scale nếu bánh nào vượt quá tốc độ tối đa */
    float v_max = ZLAC_MAX_SPEED_MPS;
    float max_v = fmaxf(fabsf(vL), fabsf(vR));
    if (max_v > v_max) {
        float scale = v_max / max_v;
        vL *= scale;
        vR *= scale;
    }

    /* m/s → rpm */
    float rpm_L = (vL / ZLAC_WHEEL_RADIUS_M) * (60.0f / (2.0f * M_PI));
    float rpm_R = (vR / ZLAC_WHEEL_RADIUS_M) * (60.0f / (2.0f * M_PI));

    /* rpm → ZLAC unit (×10) */
    int16_t zlac_L = (int16_t)roundf(rpm_L);
    int16_t zlac_R = (int16_t)roundf(rpm_R);

    ZLAC_SetSpeed_raw(zlac_L, zlac_R);
}

/* ============================================================================
 * API CÔNG KHAI: Dừng khẩn cấp
 * ============================================================================ */
void ZLAC_Stop(void)
{
    ZLAC_SetSpeed_raw(0, 0);
}

/* ============================================================================
 * API CÔNG KHAI: Lấy tốc độ thực (m/s)
 * ============================================================================ */
float ZLAC_GetVelA_mps(void)
{
    /* vel_a_raw (rpm×10) → rpm → rad/s → m/s */
    float rpm = (float)zlac_fb.vel_a / 10.0f;
    return rpm * (2.0f * M_PI / 60.0f) * ZLAC_WHEEL_RADIUS_M;
}

float ZLAC_GetVelB_mps(void)
{
    float rpm = (float)zlac_fb.vel_b / 10.0f;
    float v = rpm * (2.0f * M_PI / 60.0f) * ZLAC_WHEEL_RADIUS_M;
#if ZLAC_MOTOR_B_REVERSE
    v = -v;   /* Đảo dấu nếu motor B lắp ngược */
#endif
    return v;
}

/* ============================================================================
 * API CÔNG KHAI: Cập nhật odometry
 * ============================================================================ */
/* ============================================================================
 * API CÔNG KHAI: Cập nhật odometry (Phiên bản dùng Encoder siêu chính xác)
 * ============================================================================ */
void ZLAC_Odom_Update(float dt)
{
    static int32_t last_pos_a = 0;
    static int32_t last_pos_b = 0;
    static uint8_t first_time = 1;
    
    if (dt <= 0.0f || dt > 0.5f) return;

    // Lần đầu tiên chạy, chỉ lưu mốc ban đầu chứ chưa tính toán
    if (first_time) {
        last_pos_a = zlac_fb.pos_a;
        last_pos_b = zlac_fb.pos_b;
        first_time = 0;
        return;
    }

    // Tính sự thay đổi của Encoder kể từ lần cập nhật trước
    int32_t delta_a = zlac_fb.pos_a - last_pos_a;
    int32_t delta_b = zlac_fb.pos_b - last_pos_b;
    
    last_pos_a = zlac_fb.pos_a;
    last_pos_b = zlac_fb.pos_b;

    // Ở đây giả sử 1 vòng bánh xe = 4096 xung (Tùy theo con Motor của bạn).
    // Có Motor là 4096, có con là 10000, có con 8192. Bạn có thể chỉnh lại số này cho đúng thực tế!
    float meter_per_count = (2.0f * M_PI * ZLAC_WHEEL_RADIUS_M) / 4096.0f;
    
    // Quãng đường đi được của 2 bánh (m)
    float dist_L = delta_a * meter_per_count;
    float dist_R = delta_b * meter_per_count;
    
#if ZLAC_MOTOR_B_REVERSE
    dist_R = -dist_R; // Tự động đảo dấu nếu đã bật công tắc ZLAC_MOTOR_B_REVERSE
#endif

    // Kinematics: Chuyển quãng đường 2 bánh thành quãng đường trung tâm và góc xoay
    float dist_center = (dist_L + dist_R) / 2.0f;             // Xe đi tới bao nhiêu mét
    float d_theta     = (dist_R - dist_L) / ZLAC_WHEELBASE_M; // Xe xoay góc bao nhiêu rad

    // Cập nhật Odom (Tọa độ X, Y, Góc Theta)
    float theta_mid = zlac_odom.theta + d_theta / 2.0f;
    zlac_odom.x     += dist_center * cosf(theta_mid);
    zlac_odom.y     += dist_center * sinf(theta_mid);
    zlac_odom.theta += d_theta;

    /* Đưa góc quay về giới hạn [-π, π] */
    while (zlac_odom.theta >  M_PI) zlac_odom.theta -= 2.0f * M_PI;
    while (zlac_odom.theta < -M_PI) zlac_odom.theta += 2.0f * M_PI;

    // Vận tốc tức thời
    zlac_odom.v     = dist_center / dt;
    zlac_odom.omega = d_theta / dt;
}

void ZLAC_Odom_Reset(void)
{
    memset(&zlac_odom, 0, sizeof(zlac_odom));
}

/* ============================================================================
 * CALLBACK HAL: Nhận CAN frame từ interrupt
 * Hàm này được HAL tự gọi khi có message trong FIFO0
 * QUAN TRỌNG: Hàm này chạy trong interrupt context!
 *             → Không dùng HAL_Delay(), không gọi printf()
 *             → Chỉ set biến volatile, không thực hiện xử lý nặng
 * ============================================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_buf[8] = {0};

    /* Đọc message ra khỏi FIFO (phải làm ngay, trước khi FIFO đầy) */
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_hdr, rx_buf) != HAL_OK)
        return;

    uint32_t id = rx_hdr.StdId;

    /*
     * Phân loại theo CAN ID (cho Node 1)
     * TPDO0: 0x181, TPDO1: 0x281, TPDO2: 0x381, TPDO3: 0x481
     */

    if (id == (0x180 + ZLAC_NODE_ID))       /* TPDO0 (0x181): VẬN TỐC THỰC TẾ A + B */
    {
        /*
         * Map 0x606C sub 01 (Motor A) và sub 02 (Motor B)
         * Mỗi motor chiếm 4 bytes (int32), đơn vị: 0.1 rpm
         */
        zlac_fb.vel_a = (int16_t)RD_I32(rx_buf);      /* Byte 0..3: Vận tốc Motor A */
        zlac_fb.vel_b = (int16_t)RD_I32(rx_buf + 4);  /* Byte 4..7: Vận tốc Motor B */
    }
    else if (id == (0x280 + ZLAC_NODE_ID))  /* TPDO1 (0x281): DÒNG ĐIỆN / MOMENT THỰC TẾ A + B */
    {
        /*
         * Map 0x6077 sub 03 (32 bit: chứa cả 2 motor)
         * Byte 0..1: Dòng điện Motor A (int16, đơn vị: 0.1A)
         * Byte 2..3: Dòng điện Motor B (int16, đơn vị: 0.1A)
         */
        zlac_fb.current_a = RD_I16(rx_buf);
        zlac_fb.current_b = RD_I16(rx_buf + 2);
    }
    else if (id == (0x380 + ZLAC_NODE_ID))  /* TPDO2 (0x381): VỊ TRÍ ENCODER A + B */
    {
        /*
         * 8 bytes: Motor A (int32) + Motor B (int32)
         * Đơn vị: encoder counts
         */
        zlac_fb.pos_a = RD_I32(rx_buf);      /* Byte 0..3: Encoder Motor A */
        zlac_fb.pos_b = RD_I32(rx_buf + 4);  /* Byte 4..7: Encoder Motor B */
    }
    else if (id == (0x480 + ZLAC_NODE_ID))  /* TPDO3 (0x481): Error Code */
    {
        zlac_fb.error_code = (uint16_t)(rx_buf[0] | ((uint16_t)rx_buf[1] << 8));
    }
    else if ((id & 0x780) == 0x700)          /* NMT Heartbeat (0x700-0x77F) */
    {
        /*
         * rx_buf[0] = NMT State:
         *   0x04 = Stopped
         *   0x05 = Operational
         *   0x7F = Pre-Operational
         */
        zlac_fb.hb_received = 1;
        zlac_fb.hb_tick = HAL_GetTick();
    }
    /* SDO Response (0x580 + Node_ID): Chỉ cần nếu muốn verify SDO */
    else if (id == (0x580 + ZLAC_NODE_ID))  /* Thư phản hồi SDO (SDO Response) */
    {
        uint16_t idx = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
        uint8_t sub = rx_buf[3];
        
        // Nếu mã Index trả về là 0x6064 (Vị trí thực tế của Encoder)
        if (idx == 0x6064) 
        {
            // Lắp ráp 4 byte lại thành một số nguyên 32-bit (vị trí encoder)
            int32_t enc_val = (int32_t)((uint32_t)rx_buf[4] | ((uint32_t)rx_buf[5]<<8) | ((uint32_t)rx_buf[6]<<16) | ((uint32_t)rx_buf[7]<<24));
            
            if (sub == 0x01) zlac_fb.pos_a = enc_val; // Lưu Encoder bánh trái
            if (sub == 0x02) zlac_fb.pos_b = enc_val; // Lưu Encoder bánh phải
        }
        else if (idx == 0x2035) // Mã Index 0x2035: Điện áp DC Bus
        {
            zlac_fb.bus_voltage = (uint16_t)rx_buf[4] | ((uint16_t)rx_buf[5] << 8);
        }
    }
}
