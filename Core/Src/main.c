/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chương trình chính STM32 + W5500 + ZLAC8015D CANopen Gateway
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zlac_can.h"       /* Driver giao tiếp động cơ ZLAC8015D qua CANopen */
#include "wizchip_conf.h"   /* Thư viện cấu hình chip mạng Ethernet WIZnet W5500 */
#include "socket.h"         /* API Socket UDP / TCP của WIZnet */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ============================================================================
 * CẤU TRÚC GÓI TIN MẠNG UDP (GIAO TIẾP VỚI MÁY TÍNH PC / ROS2)
 * ============================================================================ */
#pragma pack(push, 1)

/**
 * @brief Gói tin điều khiển vận tốc nhận từ máy tính (12 bytes)
 */
typedef struct {
    uint8_t  header[2];   /**< Header cố định: 0xAA, 0x55 */
    float    v;           /**< Vận tốc dài đặt cho robot (m/s) */
    float    omega;       /**< Vận tốc góc quay đặt cho robot (rad/s) */
    uint16_t checksum;    /**< Mã kiểm tra toàn vẹn dữ liệu CRC-16/MODBUS */
} UDP_ControlPacket_t;

/**
 * @brief Gói tin Telemetry trạng thái phản hồi về máy tính (22 bytes)
 */
typedef struct {
    uint8_t  header[2];   /**< Header cố định: 0x55, 0xAA */
    int16_t  vel_a;       /**< Vận tốc thực tế Motor A (rpm / 10) */
    int16_t  vel_b;       /**< Vận tốc thực tế Motor B (rpm / 10) */
    int32_t  pos_a;       /**< Vị trí xung Encoder Motor A */
    int32_t  pos_b;       /**< Vị trí xung Encoder Motor B */
    uint16_t error_code;  /**< Mã lỗi (0: Bình thường, 0xEEEE: Kẹt tải / Quá dòng) */
    int16_t  current_a;   /**< Dòng điện thực tế Motor A (đơn vị: 0.1A) */
    int16_t  current_b;   /**< Dòng điện thực tế Motor B (đơn vị: 0.1A) */
    uint16_t bus_voltage; /**< Điện áp DC Bus / Pin (đơn vị: 0.1V hoặc 0.01V) */
} UDP_FeedbackPacket_t;

#pragma pack(pop)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ============================================================================
 * KHU VỰC 1: CÁC BIẾN QUẢN LÝ MẠNG ETHERNET W5500 & GIAO TIẾP UDP
 * ============================================================================ */
volatile uint8_t w5500_version     = 0;                /**< Phiên bản chip W5500 đọc từ thanh ghi */
uint8_t          udp_rx_buf[64];                       /**< Bộ đệm nhận UDP */
uint8_t          remote_ip[4]      = {192, 168, 1, 10}; /**< Địa chỉ IP máy tính nhận telemetry (PC / Ubuntu) */
uint16_t         remote_port       = 8888;              /**< Cổng UDP giao tiếp */
uint32_t         last_udp_rx_time  = 0;                /**< Mốc thời gian nhận gói UDP cuối (Watchdog an toàn) */
uint32_t         last_feedback_time = 0;               /**< Mốc thời gian gửi Telemetry định kỳ 50Hz (20ms) */

/* ============================================================================
 * KHU VỰC 2: CÁC BIẾN DỰ TRÙ MỞ RỘNG (GIỮ NGUYÊN)
 * ============================================================================ */
uint8_t  is_running        = 0;
uint8_t  btn_prev          = GPIO_PIN_SET; /**< Trạng thái nút nhấn chu kỳ trước (chân PA1) */
uint8_t  btn_now           = GPIO_PIN_SET; /**< Trạng thái nút nhấn chu kỳ hiện tại */
uint8_t  mode_return       = 0;            /**< 0: Thả lỏng, 1: Chờ 1 giây, 2: Chạy về */
uint32_t stop_time         = 0;            /**< Đồng hồ bấm giờ */
uint8_t  is_motor_enabled  = 1;            /**< Nhớ trạng thái motor */
int32_t  anchor_pos_a      = 0;            /**< Mốc encoder A cho tính năng lò xo ảo */
int32_t  anchor_pos_b      = 0;            /**< Mốc encoder B cho tính năng lò xo ảo */
uint8_t  has_anchor        = 0;            /**< Cờ đã chốt mốc lò xo ảo */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Hàm tính toán kiểm tra toàn vẹn CRC-16 */
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length);

/* Các hàm callback điều khiển phần cứng W5500 qua SPI */
void    W5500_Select(void);
void    W5500_Deselect(void);
void    W5500_WriteByte(uint8_t byte);
uint8_t W5500_ReadByte(void);
void    W5500_ReadBurst(uint8_t* pBuf, uint16_t len);
void    W5500_WriteBurst(uint8_t* pBuf, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Tính mã kiểm tra CRC-16/MODBUS (Đa thức 0xA001, Giá trị khởi tạo 0xFFFF)
 * @param data   Con trỏ mảng byte cần tính
 * @param length Độ dài mảng dữ liệu
 * @return uint16_t Giá trị CRC-16
 */
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

/* --- CÁC HÀM GIAO TIẾP PHẦN CỨNG SPI VỚI CHIP W5500 --- */

/** @brief Kéo chân CS xuống LOW để chọn chip W5500 */
void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

/** @brief Kéo chân CS lên HIGH để bỏ chọn chip W5500 */
void W5500_Deselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

/** @brief Gửi 1 byte qua SPI */
void W5500_WriteByte(uint8_t byte) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, HAL_MAX_DELAY);
}

/** @brief Đọc 1 byte từ W5500 qua SPI bằng cách phát 1 byte Dummy 0xFF */
uint8_t W5500_ReadByte(void) {
    uint8_t tx_data = 0xFF;
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

/** @brief Đọc một chuỗi byte liên tục (Burst Read) */
void W5500_ReadBurst(uint8_t* pBuf, uint16_t len) {
    HAL_SPI_Receive(&hspi1, pBuf, len, 100);
}

/** @brief Ghi một chuỗi byte liên tục (Burst Write) */
void W5500_WriteBurst(uint8_t* pBuf, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, pBuf, len, 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  /* ============================================================================
   * BƯỚC 1: KHỞI ĐỘNG PHẦN CỨNG MODULE ETHERNET W5500
   * ============================================================================ */
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50); /* Chờ W5500 hoàn tất chu trình nạp nội bộ */

  /* ============================================================================
   * BƯỚC 2: ĐĂNG KÝ HÀM GIAO TIẾP SPI CHO THƯ VIỆN WIZNET
   * ============================================================================ */
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Deselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
  reg_wizchip_spiburst_cbfunc(W5500_ReadBurst, W5500_WriteBurst);

  /* ============================================================================
   * BƯỚC 3: CẤU HÌNH THÔNG SỐ MẠNG TĨNH CHO STM32 (IP: 192.168.1.100)
   * ============================================================================ */
  wiz_NetInfo net_info = {
      .mac  = {0x00, 0x08, 0xDC, 0x55, 0x66, 0x77}, /* Địa chỉ MAC */
      .ip   = {192, 168, 1, 100},                   /* Địa chỉ IP STM32 */
      .sn   = {255, 255, 255, 0},                   /* Subnet Mask */
      .gw   = {192, 168, 1, 1},                     /* Gateway */
      .dns  = {8, 8, 8, 8},                         /* DNS Server */
      .dhcp = NETINFO_STATIC                        /* Chế độ IP tĩnh */
  };
  wizchip_setnetinfo(&net_info);
  w5500_version = getVERSIONR();

  /* ============================================================================
   * BƯỚC 4: KHỞI TẠO SOCKET UDP (SOCKET 0, PORT 8888)
   * ============================================================================ */
  socket(0, Sn_MR_UDP, 8888, 0);

  /* ============================================================================
   * BƯỚC 5: KHỞI TẠO DRIVER GIAO TIẾP CANOPEN CHO ZLAC8015D
   * ============================================================================ */
  ZLAC_CAN_Init(); 

  /*
   * GHI CHÚ: Khối cấu hình CAN thủ công cũ trước đây:
   * Hiện tại toàn bộ quy trình cấu hình Node, chuyển Mode 3, và Enable Servo
   * đã được tự động hóa hoàn toàn và tin cậy trong ZLAC_StateMachine().
   *
   * // NMT_Send(1, 0x01);
   * // SDO_Write(1, 0x6060, 0x00, 3, 1);
   * // SDO_Write(1, 0x6040, 0x00, 0x0006, 2);
   * // SDO_Write(1, 0x6040, 0x00, 0x0007, 2);
   * // SDO_Write(1, 0x6040, 0x00, 0x000F, 2);
   */

  /* ============================================================================
   * BƯỚC 6: RESET TỌA ĐỘ ODOMETRY VỀ MỐC GỐC BAN ĐẦU
   * ============================================================================ */
  ZLAC_Odom_Reset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* --------------------------------------------------------------------------
     * NHIỆM VỤ 1: STATE MACHINE QUẢN LÝ ĐỘNG CƠ & BẢO VỆ KẸT TẢI / QUÁ DÒNG
     * -------------------------------------------------------------------------- */
    ZLAC_StateMachine();

    /* --------------------------------------------------------------------------
     * NHIỆM VỤ 2: ĐỊNH KỲ 1 HZ (1 GIÂY) ĐỌC ĐIỆN ÁP DC BUS / PIN QUA SDO
     * -------------------------------------------------------------------------- */
    static uint32_t last_vbus_poll_time = 0;
    if (HAL_GetTick() - last_vbus_poll_time >= 1000)
    {
      last_vbus_poll_time = HAL_GetTick();
      if (ZLAC_IsReady())
      {
        SDO_Read_Request(ZLAC_NODE_ID, 0x2035, 0x00);
      }
    }

    /* --------------------------------------------------------------------------
     * NHIỆM VỤ 3: KIỂM TRA & TIẾP NHẬN GÓI ĐIỀU KHIỂN UDP TỪ MÁY TÍNH
     * -------------------------------------------------------------------------- */
    uint16_t rx_len = getSn_RX_RSR(0);
    if (rx_len >= sizeof(UDP_ControlPacket_t))
    {
      UDP_ControlPacket_t cmd_pkt;
      /* Đọc gói tin từ W5500 */
      recvfrom(0, (uint8_t*)&cmd_pkt, sizeof(UDP_ControlPacket_t), remote_ip, &remote_port);

      /* Kiểm tra Header hợp lệ (0xAA 0x55) */
      if (cmd_pkt.header[0] == 0xAA && cmd_pkt.header[1] == 0x55)
      {
        /* Tính CRC16 của 10 bytes payload (Header + v + omega) */
        uint16_t expected_crc = Calculate_CRC16((uint8_t*)&cmd_pkt, 10);

        /* So khớp CRC */
        if (expected_crc == cmd_pkt.checksum)
        {
          last_udp_rx_time = HAL_GetTick(); /* Reset Watchdog an toàn */
          if (ZLAC_IsReady())
          {
            ZLAC_SetSpeed_mps(cmd_pkt.v, cmd_pkt.omega);
          }
        }
      }
    }

    /* --------------------------------------------------------------------------
     * NHIỆM VỤ 4: WATCHDOG AN TOÀN MẠNG (MẤT KẾT NỐI > 250MS -> PHANH DỪNG XE)
     * -------------------------------------------------------------------------- */
    if (HAL_GetTick() - last_udp_rx_time > 250)
    {
      ZLAC_Stop();
    }

    /* --------------------------------------------------------------------------
     * NHIỆM VỤ 5: ĐỊNH KỲ 50 HZ (20MS) GỬI TELEMETRY VÀ BẮN LỆNH SYNC (0x080)
     * -------------------------------------------------------------------------- */
    if (HAL_GetTick() - last_feedback_time >= 20)
    {
      /* Cập nhật mốc thời gian cố định (+20ms) để triệt tiêu sai số chu kỳ (jitter) */
      last_feedback_time += 20;
      if (HAL_GetTick() - last_feedback_time > 100) last_feedback_time = HAL_GetTick();

      /* 5.1 Gửi lệnh SYNC (CAN ID 0x080, DLC=0) đòi dữ liệu từ ZLAC8015D */
      extern CAN_HandleTypeDef hcan;
      CAN_TxHeaderTypeDef sync_hdr;
      uint32_t sync_mailbox;
      sync_hdr.StdId = 0x080;
      sync_hdr.ExtId = 0;
      sync_hdr.IDE   = CAN_ID_STD;
      sync_hdr.RTR   = CAN_RTR_DATA;
      sync_hdr.DLC   = 0;
      HAL_CAN_AddTxMessage(&hcan, &sync_hdr, NULL, &sync_mailbox);

      /* 5.2 Đóng gói 22 bytes Telemetry gửi về máy tính */
      UDP_FeedbackPacket_t fb_pkt;
      fb_pkt.header[0]   = 0x55;
      fb_pkt.header[1]   = 0xAA;
      fb_pkt.vel_a       = zlac_fb.vel_a;
      fb_pkt.vel_b       = zlac_fb.vel_b;
      fb_pkt.pos_a       = zlac_fb.pos_a;
      fb_pkt.pos_b       = zlac_fb.pos_b;
      fb_pkt.error_code  = zlac_fb.error_code;
      fb_pkt.current_a   = zlac_fb.current_a;
      fb_pkt.current_b   = zlac_fb.current_b;
      fb_pkt.bus_voltage = zlac_fb.bus_voltage;
      sendto(0, (uint8_t*)&fb_pkt, sizeof(UDP_FeedbackPacket_t), remote_ip, remote_port);
    }

    /* Nhường chu kỳ CPU */
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
