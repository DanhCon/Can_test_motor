/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "zlac_can.h"
#include "wizchip_conf.h"
#include "socket.h"
  uint8_t mode_return = 0;       // 0: Th? l?ng, 1: Ch? 1 giây, 2: Ch?y v?
  uint32_t stop_time = 0;        // Ð?ng h? b?m gi? 1 giây
  uint8_t is_motor_enabled = 1;  // Nh? tr?ng thái d? không b?t/t?t liên t?c
	// Các bi?n qu?n lý tr?ng thái

// BA BI?N M?I DÀNH CHO TÍNH NANG "LÒ XO T?NG BÁNH"
int32_t anchor_pos_a = 0;
int32_t anchor_pos_b = 0;
uint8_t has_anchor = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//////////////////////////////////////////////////////////////////////////////////////////////////




uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; // Ða th?c Modbus
            else
                crc = crc >> 1;
        }
    }
    return crc;
}





//////////////////////////////////////////////////////////////////////////////////////////////////



void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}
// Hàm kéo chân CS lên HIGH d? b? ch?n W5500
void W5500_Deselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}
// Hàm g?i 1 byte qua SPI
void W5500_WriteByte(uint8_t byte) {
    uint8_t rx_data;
    // G?i di 1 byte, d?ng th?i nh?t byte rác ném vào rx_data d? ch?ng tràn OVR
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, HAL_MAX_DELAY);
}
// Hàm d?c 1 byte qua SPI
uint8_t W5500_ReadByte(void) {
    uint8_t tx_data = 0xFF; // G?i 1 byte gi? (Dummy) d? t?o xung Clock
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

// Hàm d?c m?t m?ng d? li?u liên t?c (Burst Read)
void W5500_ReadBurst(uint8_t* pBuf, uint16_t len) {
    HAL_SPI_Receive(&hspi1, pBuf, len, 100);
}
// Hàm ghi m?t m?ng d? li?u liên t?c (Burst Write)
void W5500_WriteBurst(uint8_t* pBuf, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, pBuf, len, 100);
}




//////////////////////////////////////////////////////////////////////////////////////////////////


  uint8_t is_running = 0;
  uint8_t btn_prev = GPIO_PIN_SET; // Bi?n nh? tr?ng thái nút nh?n (chân PA1)
	uint8_t btn_now;

/* USER CODE BEGIN 0 */
volatile uint8_t w5500_version = 0;

#pragma pack(push, 1)
// C?u trúc gói tin nh?n t? máy tính (12 bytes)
typedef struct {
    uint8_t  header[2];   // Luôn là 0xAA, 0x55
    float    v;           // V?n t?c dài (m/s)
    float    omega;       // V?n t?c góc (rad/s)
    uint16_t checksum;    // Ki?m tra tính toàn v?n
} UDP_ControlPacket_t;
// Cấu trúc gói tin gửi về máy tính (22 bytes)
typedef struct {
    uint8_t  header[2];   // Luôn là 0x55, 0xAA
    int16_t  vel_a;       // Tốc độ thực motor A (rpm/10)
    int16_t  vel_b;       // Tốc độ thực motor B (rpm/10)
    int32_t  pos_a;       // Encoder motor A
    int32_t  pos_b;       // Encoder motor B
    uint16_t error_code;  // Mã lỗi từ Driver (0xEEEE: kẹt tải/quá dòng)
    int16_t  current_a;   // Dòng điện motor A (đơn vị: 0.1A)
    int16_t  current_b;   // Dòng điện motor B (đơn vị: 0.1A)
    uint16_t bus_voltage; // Điện áp DC Bus (đơn vị: 0.1V hoặc 0.01V)
} UDP_FeedbackPacket_t;
#pragma pack(pop)
uint8_t udp_rx_buf[64];
uint8_t remote_ip[4] = {192, 168, 1, 10}; // IP máy tính Ubuntu c?a b?n
uint16_t remote_port = 8888;
uint32_t last_udp_rx_time = 0;
uint32_t last_feedback_time = 0;

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
	  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50); // Ch? W5500 kh?i d?ng xong
  // 2. Ðang ký các hàm SPI v?a vi?t ? trên cho thu vi?n WIZnet
  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Deselect);
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	  reg_wizchip_spiburst_cbfunc(W5500_ReadBurst, W5500_WriteBurst);
  // 3. C?u hình d?a ch? m?ng cho STM32
  wiz_NetInfo net_info = {
      .mac  = {0x00, 0x08, 0xDC, 0x55, 0x66, 0x77}, // Ð?a ch? MAC (t? b?a cung du?c)
      .ip   = {192, 168, 1, 100},                   // Ð?a ch? IP c?a STM32
      .sn   = {255, 255, 255, 0},                   // Subnet mask
      .gw   = {192, 168, 1, 1},                     // Gateway (Router)
      .dns  = {8, 8, 8, 8},                         // DNS Server
      .dhcp = NETINFO_STATIC                        // Dùng IP tinh
  };
  wizchip_setnetinfo(&net_info);
    w5500_version = getVERSIONR();
  // 4. Kh?i t?o Socket UDP trên C?ng 8888 (Socket s? 0)
  socket(0, Sn_MR_UDP, 8888, 0);

	ZLAC_CAN_Init(); 
  // 1. Ðánh th?c m?ng
//  NMT_Send(1, 0x01);
//  HAL_Delay(50);
//  // 2. Ch?n ch? d? Profile Velocity (Mode 3)
//  SDO_Write(1, 0x6060, 0x00, 3, 1);
//  HAL_Delay(50);
//  // 3. Quy trình b?t Motor (3 bu?c b?t bu?c)
//  SDO_Write(1, 0x6040, 0x00, 0x0006, 2); // Shutdown
//  HAL_Delay(50);
//  SDO_Write(1, 0x6040, 0x00, 0x0007, 2); // Switch On
//  HAL_Delay(50);
//  SDO_Write(1, 0x6040, 0x00, 0x000F, 2); // Enable Operation
//  HAL_Delay(50);

	// 1. Khai báo 1 bien cau truc de chua info setting fillter
	
	
	
	
//  HAL_CAN_Start(&hcan);
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

ZLAC_Odom_Reset();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ZLAC_StateMachine();

    // 1. Định kỳ 1 giây (1Hz) hỏi điện áp DC Bus / Pin từ ZLAC qua SDO
    static uint32_t last_vbus_poll_time = 0;
    if (HAL_GetTick() - last_vbus_poll_time >= 1000)
    {
      last_vbus_poll_time = HAL_GetTick();
      if (ZLAC_IsReady())
      {
        SDO_Read_Request(ZLAC_NODE_ID, 0x2035, 0x00);
      }
    }
    // 2. Ki?m tra có gói tin UDP m?i d?n hay không
    uint16_t rx_len = getSn_RX_RSR(0);
    if (rx_len >= sizeof(UDP_ControlPacket_t))
    {
      UDP_ControlPacket_t cmd_pkt;
      // Ð?c gói tin t? W5500
      recvfrom(0, (uint8_t*)&cmd_pkt, sizeof(UDP_ControlPacket_t), remote_ip, &remote_port);
      // Ki?m tra Header xem có dúng là 0xAA 0x55 không
      if (cmd_pkt.header[0] == 0xAA && cmd_pkt.header[1] == 0x55)
      {
        // Tính CRC16 c?a 10 bytes d?u tiên
        uint16_t expected_crc = Calculate_CRC16((uint8_t*)&cmd_pkt, 10);
        // So kh?p v?i CRC máy tính g?i xu?ng
        if (expected_crc == cmd_pkt.checksum)
        {
          last_udp_rx_time = HAL_GetTick(); // Reset Watchdog
          if (ZLAC_IsReady())
          {
            ZLAC_SetSpeed_mps(cmd_pkt.v, cmd_pkt.omega);
          }
        }
      }
    }
    // 3. Co ch? WATCHDOG AN TOÀN: N?u m?t k?t n?i m?ng quá 250ms thì l?p t?c d?ng xe
    if (HAL_GetTick() - last_udp_rx_time > 250)
    {
      ZLAC_Stop();
    }
    // 4. G?i Telemetry / Ph?n h?i v? l?i máy tính d?nh k? 50ms (20Hz)
    if (HAL_GetTick() - last_feedback_time >= 20)
    {
      last_feedback_time += 20;
      if (HAL_GetTick() - last_feedback_time > 100) last_feedback_time = HAL_GetTick();
			 // ---- [THÊM M?I] B?N L?NH SYNC (0x080) Ð? ÐÒI D? LI?U T? ZLAC ----
      extern CAN_HandleTypeDef hcan; // G?i bi?n hcan t? main
      CAN_TxHeaderTypeDef sync_hdr;
      uint32_t sync_mailbox;
      sync_hdr.StdId = 0x080;        // ID Chu?n c?a l?nh SYNC trong CANopen
      sync_hdr.ExtId = 0;
      sync_hdr.IDE = CAN_ID_STD;
      sync_hdr.RTR = CAN_RTR_DATA;
      sync_hdr.DLC = 0;              // Gói SYNC không mang data (Ð? dài = 0 byte)
      HAL_CAN_AddTxMessage(&hcan, &sync_hdr, NULL, &sync_mailbox);
      // ----------------------------------------------------------------
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
    HAL_Delay(1); // Nhuong chu ky CPU
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

