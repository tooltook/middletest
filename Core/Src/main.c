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
//开源者说的话：这个项目里其实没有用到pid控制函数，所以扒库的时候没必要扒pid
//pid算法在这里就是垃圾，就没有步进好使
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PITCH_CENTER 90.0f//这里定义了关于舵机机械中位的地方
//其实不用宏定义也可以

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


/* 时间 */
static uint8_t i2c_fail_cnt = 0;
static uint32_t last_ctrl = 0;
static uint8_t imu_ok = 0;
//初始姿态平面的获取参数
static uint32_t imu_ok_since = 0;

/* === Yaw drift calibration + hold (merge from classmate) === */
static uint8_t  yaw_calib_state = 0;    // 0未开始 1已取点1 2点取点2，3完成补偿解算
static uint32_t yaw_t0 = 0;
static uint32_t yaw_t1 = 0, yaw_t2 = 0;
static float    yaw_y1 = 0.0f, yaw_y2 = 0.0f;

static float    yaw_k = 0.0f;           // 漂移斜率(度/毫秒)
static float    yaw_b = 0.0f;           // 截距(度)

static uint8_t  yaw_locked = 0;
static float    yaw0 = 0.0f;            // 锁定目标 yaw（补偿后）
static float    yaw_servo_angle = 90.0f;//机械中位

const float yaw_deadband_deg = 5.0f;   // 死区（度）
const float yaw_step_deg     = 1.0f;   // 每次调整的舵机角度步长（不用改了，很丝滑）
/* 舵机限幅 */
const float yaw_servo_min = 0.0f;
const float yaw_servo_max = 180.0f;

static float pitch_servo_angle = PITCH_CENTER;   // 作为pitch舵机机械中位
static float pitch0 = 0.0f;                      // 锁定参考pitch的平面
static uint8_t pitch_hold_locked = 0;


// 3) 参数：
const float deadband_deg = 3.0f;     // pitch一般可以比yaw小一点：1~3°（建议不要太大，可以自己调）
const float step_deg     = 0.4f;     // 每次舵机走0.5°，先小后大
const float servo_min    = 30.0f;     //因为在测试时会发生大规模到达限位，我缩小了控制范围
const float servo_max    = 150.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}//用于修正在yaw超过-180的补救措施




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

  for (int i=0;i<4;i++) {
    Servo_SetAngle(SERVO_PITCH,90.0f);
    Servo_SetAngle(SERVO_YAW, 90.0f);
  }//初始化使其到达机械限位

  HAL_Delay(500);


  uint32_t last=HAL_GetTick();
  last_ctrl = HAL_GetTick();
//时刻时间戳，一般用于计时
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
        imu_ok = 1;
        if (imu_ok_since == 0) imu_ok_since = HAL_GetTick();

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
        imu_ok = 0;

        if (i2c_fail_cnt >= 3)
        {
          HAL_I2C_DeInit(&hi2c1);
          HAL_Delay(10);
          MX_I2C1_Init();
          i2c_fail_cnt = 0;
          continue;
        }
      }//这里一大坨是读数据以及将其i2c堵塞时恢复

    }

    if (imu_ok && (HAL_GetTick() - last_ctrl >= 10))   // 100Hz
    {
      last_ctrl = HAL_GetTick();


      //pitch控制区
        if (HAL_GetTick()-last_ctrl >= 1000&&!pitch_hold_locked) {//这个地方事实上是有问题的
          // 1) 第一次锁定参考
            pitch0 = pitch;
            pitch_servo_angle = PITCH_CENTER;
            pitch_hold_locked = 1;//事实上这里要修一下但是以我的参考平面刚好参考平面是pitch是0

        }

        //  计算误差（角度制）
        float err = pitch - pitch0;
        // 死区 + 步进
        if (err > deadband_deg) {
          pitch_servo_angle += step_deg;     // 方向反了就改成 +=//现在没问题这里不用改
        } else if (err < -deadband_deg) {
          pitch_servo_angle -= step_deg;     // 方向反了就改成 -=
        }

        //  限幅并输出
        if (pitch_servo_angle < servo_min) pitch_servo_angle = servo_min;
        if (pitch_servo_angle > servo_max) pitch_servo_angle = servo_max;

        Servo_SetAngle(SERVO_PITCH, pitch_servo_angle);//servo是写的一个根据角度控制ccr的函数库，这里可以用一下，
        //servo里有死区设置我没开

     //yaw控制部分
      /* 说明：
         - 依赖 Mahony 输出的 yaw
         - 标定：pitch_ref_locked 后开始，t0+5s取点1，t0+10s取点2，拟合线性漂移
         - 保持：锁定 yaw0，用 deadband + 步进角度调整 yaw 舵机
      */
      uint32_t now_ms = HAL_GetTick();

      /* 取 Mahony 当前 yaw */
      float yaw_now = yaw;
      yaw_now = wrap180(yaw_now);

      /* 1) 在 pitch 已锁定后启动 yaw 标定 */
      if ( yaw_calib_state == 0) {
        yaw_t0 = now_ms;
        yaw_calib_state = 1;
        yaw_locked = 0;

        /* 机械中位输出 */
        yaw_servo_angle = 90.0f;
        Servo_SetAngle(SERVO_YAW, yaw_servo_angle);
      }

      /* 2) 两点法拟合漂移：5s点1，10s点2 *///就是说用一个线性函数来拟合这个误差
      if (yaw_calib_state == 1 && (now_ms - yaw_t0) >= 5000) {
        yaw_t1 = now_ms;
        yaw_y1 = yaw_now;
        yaw_calib_state = 2;
      }
      else if (yaw_calib_state == 2 && (now_ms - yaw_t0) >= 10000) {
        yaw_t2 = now_ms;
        yaw_y2 = yaw_now;

        yaw_k = (yaw_y2 - yaw_y1) / (float)(yaw_t2 - yaw_t1);  // 度/ms
        yaw_b = yaw_y2 - yaw_k * (float)yaw_t2;

        yaw_calib_state = 3;
      }

      /* 3) 计算补偿后的 yaw（不改写原yaw） */
      float yaw_corr = yaw_now;
      if (yaw_calib_state == 3) {
        yaw_corr = yaw_now - (yaw_k * (float)now_ms + yaw_b);
        yaw_corr = wrap180(yaw_corr);
      }

      /* 4) 标定完成后锁定目标 yaw0 */
      if (yaw_calib_state == 3 && !yaw_locked) {
        yaw0 = yaw_corr;
        yaw_locked = 1;
      }

      /* 5) deadband + 步进角度的 yaw 保持控制 */
      if (yaw_locked) {
        float err = wrap180(yaw_corr - yaw0);

        /* 误差大于死区才动，避免抖动 */
        if (err > yaw_deadband_deg) {
          yaw_servo_angle -= yaw_step_deg;   // 方向若反了就改成 +=//这里是好的不用改
        }
        else if (err < -yaw_deadband_deg) {
          yaw_servo_angle += yaw_step_deg;   // 方向若反了就改成 -=
        }

        /* 限幅 */
        if (yaw_servo_angle < yaw_servo_min) yaw_servo_angle = yaw_servo_min;
        if (yaw_servo_angle > yaw_servo_max) yaw_servo_angle = yaw_servo_max;

        Servo_SetAngle(SERVO_YAW, yaw_servo_angle);
      }
      /* ===================== end of YAW control ===================== */




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
