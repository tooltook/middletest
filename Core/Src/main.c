/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../app/Inc/mpu6050.h"
#include "mahony.h"
#include "servo.h"
#include "pid.h"

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
/* IMU */
MPU6050_t mpu6050;
float ax,ay,az;
float gx,gy,gz;

/* PID 控制器 */
PID_t pid_pitch;
PID_t pid_roll;

/* 目标角度 */
float pitch_target = 0.0f;
float roll_target  = 0.0f;

/* PID 输出（角度补偿） */
float pitch_out = 0.0f;
float roll_out  = 0.0f;

/* 时间 */
static uint8_t i2c_fail_cnt = 0;
static uint32_t last_ctrl = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c1);

  Servo_Init(&htim2);

  Servo_SetAngle(SERVO_PITCH,90.0f);
  Servo_SetAngle(SERVO_ROLL, 90.0f);
  HAL_Delay(500);

  PID_Init(&pid_pitch,
    3.0f,
    0.0f,
    0.0f,
    10.f,
    30.0f);

  PID_Init(&pid_roll,
    3.0f,
    0.0f,
    0.0f,
    10.f,
    30.0f
      );

  uint32_t last=HAL_GetTick();
  last_ctrl = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last >= 5)   // 200Hz
    {
      uint32_t now = HAL_GetTick();
      float dt = (float)(now - last) / 1000.0f;
      last = now;

      if (MPU6050_Read_All(&hi2c1, &mpu6050) == HAL_OK)
      {
        i2c_fail_cnt = 0;

        ax = mpu6050.Ax;
        ay = mpu6050.Ay;
        az = mpu6050.Az;

        gx = mpu6050.Gx;
        gy = mpu6050.Gy;
        gz = mpu6050.Gz;

        Mahony_Update(gx, gy, gz,
                      ax, ay, az,
                      dt);

      }
      else
      {
        i2c_fail_cnt++;

        if (i2c_fail_cnt >= 3)
        {
          HAL_I2C_DeInit(&hi2c1);
          HAL_Delay(10);
          MX_I2C1_Init();
          i2c_fail_cnt = 0;
          continue;
        }
      }

    }

    /* ================= PID + Servo（100Hz） ================= */
    if (HAL_GetTick() - last_ctrl >= 10)   // 100Hz
    {
      float dt_ctrl = (HAL_GetTick() - last_ctrl) * 0.001f;
      last_ctrl = HAL_GetTick();

      /* pitch / roll 来自 Mahony 的全局变量 */
      pitch_out = PID_Update(&pid_pitch,
                             pitch_target,
                             pitch,
                             dt_ctrl);

      roll_out  = PID_Update(&pid_roll,
                             roll_target,
                             roll,
                             dt_ctrl);

      /* 舵机输出（90°为中位） */
      Servo_SetAngle(SERVO_PITCH, 90.0f + pitch_out);
      Servo_SetAngle(SERVO_ROLL,  90.0f + roll_out);
    }


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
#ifdef USE_FULL_ASSERT
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
