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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zlac_can.h"
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
  uint8_t is_running = 0;
  uint8_t btn_prev = GPIO_PIN_SET; // Bi?n nh? tr?ng thái nút nh?n (chân PA1)
	uint8_t btn_now;

/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

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
    ZLAC_StateMachine();
    
    // Ð?c Encoder liên t?c
    SDO_Read_Request(1, 0x6064, 0x01);
    HAL_Delay(5);
    SDO_Read_Request(1, 0x6064, 0x02);
    HAL_Delay(5);

    ZLAC_Odom_Update(0.02f);
		
    if (ZLAC_IsReady()) 
    {
        // 1. CH?T T?A Ð? G?C C?A T?NG BÁNH KHI M?I KH?I Ð?NG
        if (has_anchor == 0) {
            anchor_pos_a = zlac_fb.pos_a;
            anchor_pos_b = zlac_fb.pos_b;
            has_anchor = 1;
        }

        float van_toc_day = zlac_odom.v;
        if (van_toc_day < 0.0f) van_toc_day = -van_toc_day;

        // 2. TÍNH XEM T?NG BÁNH ÐÃ B? KÉO GIÃN RA BAO NHIÊU XUNG
        int32_t err_a = anchor_pos_a - zlac_fb.pos_a;
        int32_t err_b = anchor_pos_b - zlac_fb.pos_b;
        
        // Tính tr? tuy?t d?i d? check xem dã v? nhà chua
        int32_t abs_err_a = (err_a > 0) ? err_a : -err_a;
        int32_t abs_err_b = (err_b > 0) ? err_b : -err_b;

        // ----------------------------------------------------
        // TR?NG THÁI 0: XE NG? (NH? BÁNH CHO NGU?I Ð?Y T? DO)
        // ----------------------------------------------------
        if (mode_return == 0) 
        {
            if (is_motor_enabled == 1) {
                SDO_Write(ZLAC_NODE_ID, 0x6040, 0x00, 0x0006, 2); // C?t di?n
                HAL_Delay(5);
                is_motor_enabled = 0;
            }

            if (van_toc_day < 0.02f) 
            {
                // N?u b? d?y xa hon 1000 Xung (kho?ng vài cm) thì ch? 1 giây
                if (abs_err_a > 1000 || abs_err_b > 1000) {
                    mode_return = 1; 
                    stop_time = HAL_GetTick(); 
                }
            }
            else {
                // Ðang b? d?y, liên t?c reset d?ng h?
                stop_time = HAL_GetTick();
            }
        }
        // ----------------------------------------------------
        // TR?NG THÁI 1: CH? 1 GIÂY Ð? CH?C CH?N ÐÃ BUÔNG TAY
        // ----------------------------------------------------
        else if (mode_return == 1) 
        {
            if (van_toc_day > 0.02f) {
                mode_return = 0; 
            }
            else if (HAL_GetTick() - stop_time > 1000) 
            {
                if (is_motor_enabled == 0) {
                    SDO_Write(ZLAC_NODE_ID, 0x6040, 0x00, 0x0007, 2); // Bom di?n
                    HAL_Delay(10);
                    SDO_Write(ZLAC_NODE_ID, 0x6040, 0x00, 0x000F, 2); // Kích ho?t ch?y
                    HAL_Delay(10);
                    is_motor_enabled = 1;
                }
                mode_return = 2; 
            }
        }
        // ----------------------------------------------------
        // TR?NG THÁI 2: ÐI?U KHI?N T?NG BÁNH LÙI V? ÐÚNG S? XUNG CU!
        // ----------------------------------------------------
        else if (mode_return == 2) 
        {
            // V? d?n nhà v?i sai s? siêu nh? (< 100 xung, siêu chính xác)
            if (abs_err_a < 100 && abs_err_b < 100) 
            {
                ZLAC_Stop(); 
                mode_return = 0; // T?t di?n di ng?
            }
            else 
            {
                // L?c kéo dàn h?i c?a "Dây chun": L?ch càng xa kéo v? càng m?nh
                int16_t out_a = (int16_t)(err_a * 0.02f);
                int16_t out_b = (int16_t)(err_b * 0.02f);
                
                // Hãm t?c d? l?i không cho xe gi?t b?n mình (Max 150 vòng/phút)
                // S?a s? 150 thành 50 ho?c 80
								if(out_a > 50) out_a = 50;     
								if(out_a < -50) out_a = -50;
								if(out_b > 50) out_b = 50;
								if(out_b < -50) out_b = -50;
                
                // Truy?n tr?c ti?p l?nh d?c l?p vào t?ng bánh!
                SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x01, (int32_t)out_a, 4);
                HAL_Delay(5);
                SDO_Write(ZLAC_NODE_ID, 0x60FF, 0x02, (int32_t)out_b, 4);
            }
        }
    }
    
    HAL_Delay(20); 
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
