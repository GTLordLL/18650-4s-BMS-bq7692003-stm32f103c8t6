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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BQ_ADDR (0x08 << 1) // 7位地址0x08，HAL库需要左移一位变成0x10
#define PROT_UV    3000   // 欠压保护 3.0V
#define PROT_OV    4200   // 过充保护 4.2V
#define PROT_OT    60.0f  // 过温保护 60℃
#define PROT_UT    -5.0f  // 低温保护 -5℃ (禁止充电)
#define PROT_OC    20000  // 过流保护 20A (根据你的分流电阻能力)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t sys_stat = 0;
HAL_StatusTypeDef bq_status;
uint8_t sys_ctrl1, sys_ctrl2;
float bq_gain = 0.0f;
int8_t bq_offset = 0;
uint8_t tx_packet[3];     // 串口发送缓冲区
uint8_t cell_index_map[4] = {0, 1, 2, 4}; // 定义 4 串模式下的寄存器索引：VC1(0), VC2(1), VC3-VC4(2), VC5(4)
volatile uint8_t mos_state = 0; // 0: 关闭, 1: 开启
uint32_t last_button_time = 0; // 记录上次触发的时间，防止按键抖动
volatile uint8_t button_pressed_flag = 0;
int16_t debug_val = 0;
uint16_t cells[4];
int16_t curr;
float t;
uint8_t error_flag = 0;
volatile uint8_t timer_100ms_flag = 0;
uint16_t task_counter = 0;
uint16_t uv_counter = 0; // 欠压计数器
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BQ76920_Boot(void) {
    // 给 TS1 (PA1) 一个高电平脉冲唤醒芯片
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(3); // 持续 > 1ms 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(10); // 等待 REGOUT 稳定
}

uint8_t CRC8(uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef BQ76920_ReadReg_CRC(uint8_t regAddr, uint8_t *pData) {
    uint8_t buf[3];
    HAL_StatusTypeDef status;

    // 1. 发送寄存器地址
    status = HAL_I2C_Master_Transmit(&hi2c1, BQ_ADDR, &regAddr, 1, 100);
    if (status != HAL_OK) return status;

    // 2. 读取数据字节和 CRC 字节
    uint8_t raw_rx[2]; // [Data, CRC]
    status = HAL_I2C_Master_Receive(&hi2c1, BQ_ADDR, raw_rx, 2, 100);
    if (status != HAL_OK) return status;

    // 3. 校验 CRC
    // 校验序列包括：[Addr_R] + [Data]
    buf[0] = BQ_ADDR | 0x01; // 读地址
    buf[1] = raw_rx[0];      // 数据
    
    if (CRC8(buf, 2) == raw_rx[1]) {
        *pData = raw_rx[0];
        return HAL_OK;
    } else {
        return HAL_ERROR; // CRC 校验失败
    }
}

// 读取 BQ 芯片内部的 ADC 校准参数
void BQ76920_Read_ADC_Params(void) {
    uint8_t g1, g2, off;
    if(BQ76920_ReadReg_CRC(0x50, &g1) == HAL_OK && 
       BQ76920_ReadReg_CRC(0x51, &g2) == HAL_OK) {
        
        /* 严格按照手册公式：
           Gain = 365 + ((g1 & 0x0C) << 1) + ((g2 & 0xE0) >> 5) 
           单位是 uV/LSB
        */
        uint16_t gain_uV = 365 + ((uint16_t)(g1 & 0x0C) << 1) + ((uint16_t)(g2 & 0xE0) >> 5);
		   
        bq_gain = (float)gain_uV / 1000.0f; // 转换为 mV/LSB
    }else {
        bq_gain = 0.380f; // 如果读取失败，给一个典型值，防止电压计算溢出
    }
	
    if(BQ76920_ReadReg_CRC(0x59, &off) == HAL_OK) {
        bq_offset = (int8_t)off; // 偏移量单位是 mV
    }else {
        bq_offset = -99; // 如果读取失败，特意设为-99，方便你在 OLED 上一眼看出是“读取错误”
    }
	
}

// 读取指定单体电压
uint16_t Get_Cell_Voltage_mV(uint8_t cell_idx) {
    uint8_t v_hi, v_lo;
    HAL_StatusTypeDef s1, s2;

    s1 = BQ76920_ReadReg_CRC(0x0C + (cell_idx * 2), &v_hi);
    s2 = BQ76920_ReadReg_CRC(0x0D + (cell_idx * 2), &v_lo);

    if (s1 != HAL_OK || s2 != HAL_OK) return 0; // 通信失败显示 0

    // 重点：必须用 0x3F 屏蔽掉 v_hi 的高两位！
    uint16_t adc_raw = ((uint16_t)(v_hi & 0x3F) << 8) | v_lo;
    
    // 计算
    return (uint16_t)((adc_raw * bq_gain) + (float)bq_offset);
	// return adc_raw; // 直接返回 14 位的原始数据
}

HAL_StatusTypeDef BQ76920_WriteReg_CRC(uint8_t regAddr, uint8_t data) {
    uint8_t buf[4];
    HAL_StatusTypeDef status;

    // 1. 准备 CRC 校验数据
    // 校验序列：[Addr_W] + [RegAddr] + [Data]
    buf[0] = BQ_ADDR;     // 0x10
    buf[1] = regAddr;     // 寄存器地址
    buf[2] = data;        // 要写入的数据
    
    // 2. 计算 CRC 并放入第四个字节
    buf[3] = CRC8(buf, 3);

    // 3. 通过 I2C 发送 [RegAddr] + [Data] + [CRC]
    // 注意：HAL_I2C_Master_Transmit 会自动发送器件地址，所以我们从 buf[1] 开始发 3 个字节
    status = HAL_I2C_Master_Transmit(&hi2c1, BQ_ADDR, &buf[1], 3, 100);
    
    return status;
}

void BQ76920_Set_MOS(uint8_t chg, uint8_t dsg) {
	// 强制清除所有状态标志（过流、欠压等故障标志）
    // 只有清除了这些，DSG 才能被手动开启
    BQ76920_WriteReg_CRC(0x00, 0xFF);
	
    uint8_t ctrl2;
    // 先读出当前状态
    BQ76920_ReadReg_CRC(0x05, &ctrl2);
    
    if(chg) ctrl2 |= 0x02; else ctrl2 &= ~0x02;
    if(dsg) ctrl2 |= 0x01; else ctrl2 &= ~0x01;
    
    // 写入新状态
    BQ76920_WriteReg_CRC(0x05, ctrl2);
}

// 电流测量，放电显示负值，充电显示正值
int16_t Get_Current_mA(void) {
    uint8_t i_hi, i_lo;
    if (BQ76920_ReadReg_CRC(0x32, &i_hi) != HAL_OK || 
        BQ76920_ReadReg_CRC(0x33, &i_lo) != HAL_OK) return 0;

    // 1. 读取原始值
    int16_t cc_raw = (int16_t)((i_hi << 8) | i_lo);
    // debug_val = cc_raw;
    // 2. 减去零漂 (Offset Null)
    // 根据你观察到的 350-353，我们取中间值 352
    int16_t corrected_raw = cc_raw - 352;

    // 3. 计算物理电流
    float current_f = (float)corrected_raw * 8.44f / 3.0f; 
    return (int16_t)current_f;
}

// 温度测量
float Get_Temp_Celsius(void) {
    uint8_t t_hi, t_lo;
    BQ76920_ReadReg_CRC(0x2C, &t_hi);
    BQ76920_ReadReg_CRC(0x2D, &t_lo);
    uint16_t adc = ((uint16_t)(t_hi & 0x3F) << 8) | t_lo;
	debug_val = adc;
    if (adc >= 16383 || adc == 0) return -99.0f;

    // 使用实测的两点斜率公式：Temp = 20 + (adc - 4420) * (-0.053)
    float temp = 20.0f + (float)(adc - 4420) * (-0.0535f) - 1.3f;
    
    return temp;
}

// 容量 SOC 计算
uint16_t Calculate_SOC(uint16_t v_min_mv) {
    if (v_min_mv >= 4150) return 1000; // 满电
    if (v_min_mv <= 3000) return 0;    // 没电

    // 采用三段式拟合，比单一线性更准
    if (v_min_mv > 3800) {
        // 3.8V - 4.15V: 占 40% 容量 (60% -> 100%)
        return 600 + (v_min_mv - 3800) * 400 / (4150 - 3800);
    } 
    else if (v_min_mv > 3600) {
        // 3.6V - 3.8V: 占 50% 容量 (10% -> 60%) —— 平台区
        return 100 + (v_min_mv - 3600) * 500 / (3800 - 3600);
    } 
    else {
        // 3.0V - 3.6V: 占 10% 容量 (0% -> 10%) —— 陡降区
        return (v_min_mv - 3000) * 100 / (3600 - 3000);
    }
}

// 辅助发送函数
void send_packet(uint8_t id, uint16_t value) {
    static uint8_t temp_buf[3]; 

	// 如果 DMA 还在忙（虽然 3 字节在 9600 波特率下只需 3ms，几乎不会发生）
    // 这种判定是“非阻塞”的，如果忙就直接放弃这一包，保证系统不卡死
    if (huart1.gState != HAL_UART_STATE_READY) {
        return; 
    }

    temp_buf[0] = id;
    temp_buf[1] = (uint8_t)(value >> 8);
    temp_buf[2] = (uint8_t)(value & 0xFF);

    HAL_UART_Transmit_DMA(&huart1, temp_buf, 3);
}

void Check_Protections(uint16_t *v_cells, int16_t current, float temp) {
    uint8_t current_errors = 0;

    // 1. 检查各项保护
    // 欠压
    uint16_t min_v = v_cells[0];
    for(int i=1; i<4; i++) if(v_cells[i] < min_v) min_v = v_cells[i];
    
    if (min_v < PROT_UV) {
        // 每 200ms 调用一次，累加 10 次即为 2 秒
        if (uv_counter < 10) uv_counter++;
        else current_errors |= 0x01; 
    } else {
        uv_counter = 0; // 恢复正常立刻重置
    }

    // 过压
    for(int i=0; i<4; i++) if (v_cells[i] > PROT_OV) current_errors |= 0x02;

    // 温度
    if (temp > PROT_OT || temp < PROT_UT) current_errors |= 0x04;

    // 2. 更新全局标志
    error_flag = current_errors;

    // 3. 动作执行：只要有错，立刻关断
    if (error_flag != 0) {
        BQ76920_Set_MOS(0, 0); 
        mos_state = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	// mos管状态灯
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	// 错误报警灯
	}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  BQ76920_Boot(); // 第一步：唤醒芯片
  HAL_Delay(500);


  // 尝试读取 SYS_STAT 寄存器 (0x00)
  bq_status = BQ76920_ReadReg_CRC(0x00, &sys_stat);

  if (bq_status == HAL_OK) {
      // 通信成功：点亮
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  } else {
      // 通信失败：LED 闪烁报错
      while(1) {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
          HAL_Delay(500);
      }
  }
  
    // --- 硬件保护配置 ---
	// 1. PROTECT1 (0x06): 配置短路保护 (SCD)
	// 设置 SCD 阈值为 44mV (对应 3mOhm 下约 14.6A), 延时 100us
	// 查表得：0x00 (22mV), 0x01 (33mV), 0x02 (44mV)...
	BQ76920_WriteReg_CRC(0x06, 0x02); 
  HAL_Delay(10);

	// 2. PROTECT2 (0x07): 配置过流保护 (OCD)
	// 设置 OCD 阈值为 17mV (对应 3mOhm 下约 5.6A), 延时 160ms
	// 查表得：0x00 (8mV), 0x01 (11mV), 0x02 (14mV), 0x03 (17mV)...
	BQ76920_WriteReg_CRC(0x07, 0x03);
  HAL_Delay(10);

	// 3. PROTECT3 (0x08): 配置欠压/过压延时
	// 这个寄存器通常保持默认即可，或者根据需要调整硬件 UV/OV 的动作延迟
	BQ76920_WriteReg_CRC(0x08, 0x00);
  HAL_Delay(10);
  
  // --- 数据采集配置 ---
  // 在读取之前，先尝试清除所有状态寄存器，防止之前的错误挂起
  BQ76920_WriteReg_CRC(0x00, 0xFF); 
  HAL_Delay(10);
  // 强制关闭均衡，防止 100 欧姆电阻产生压降
	BQ76920_WriteReg_CRC(0x01, 0x00);
  HAL_Delay(10);
  
  // --- 第一步：开启电压和温度 ADC ---
	BQ76920_ReadReg_CRC(0x04, &sys_ctrl1);
	// Bit 4 (ADC_EN): 开启电压采样
	// Bit 3 (TEMP_SEL): 1 为外部 NTC, 0 为内部感测。通常选 1
	sys_ctrl1 |= (0x10 | 0x08); 
	BQ76920_WriteReg_CRC(0x04, sys_ctrl1);

	// --- 第二步：开启电流 ADC (库仑计) ---
	BQ76920_ReadReg_CRC(0x05, &sys_ctrl2);
	// Bit 6 (CC_EN): 开启电流连续采样
	// Bit 0 & 1: 确保放电/充电管脚根据你的需求配置（注意：这通常由保护逻辑控制，不要盲目置 1）
	sys_ctrl2 |= 0x40; 
	BQ76920_WriteReg_CRC(0x05, sys_ctrl2);

	// --- 第三步：必要的等待 ---
	// BQ769x0 的 ADC 转换周期通常为 250ms (电压) 和 250ms (电流)
	// 延时 500ms 以上即可，2000ms 非常稳妥
	HAL_Delay(1000); 

	// --- 第四步：读取增益和偏移校准参数 ---
	// 这一步非常关键！没有这些参数，你读到的原始 ADC 值无法转换成物理电压/电流
	// 给读取校准参数增加重试
	uint8_t retry = 5;
	while(retry--) {
		BQ76920_Read_ADC_Params();
		if(bq_gain > 0.30f) break; // 读取成功则跳出
		HAL_Delay(100);
	}
	
	// 强行修正！不管读到什么，都给它一个合理的值
	if(bq_offset < -50 || bq_offset > 50) {
		bq_offset = 0; // 正常的 Offset 只有几个 mV，给 0 比给 -127 准得多
	}

	// 顺便检查 gain 是否合理，如果不合理也手动给一个经验值
	if(bq_gain < 0.35f || bq_gain > 0.40f) {
		bq_gain = 0.381f; // 这是一个非常典型的增益值
	}
	bq_gain = bq_gain * 1.0102f;
	
	HAL_TIM_Base_Start_IT(&htim2); // 以中断模式启动 TIM2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
	  // --- 核心优化 1：按键处理 (实时响应，不受 Delay 影响) ---
    if (button_pressed_flag) {
        button_pressed_flag = 0;
        if (error_flag == 0) { // 只有无故障时才允许手动操作
            mos_state = !mos_state;
            BQ76920_Set_MOS(mos_state, mos_state);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, mos_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }

    // --- 核心优化 2：基于定时器的任务分发 ---
    if (timer_100ms_flag) {
        timer_100ms_flag = 0;
        task_counter++;

        // 每 200ms 运行一次：保护检查 (高频任务)
        if (task_counter % 2 == 0) {
            for(int i=0; i<4; i++) cells[i] = Get_Cell_Voltage_mV(cell_index_map[i]);
            curr = Get_Current_mA();
            t = Get_Temp_Celsius();
            Check_Protections(cells, curr, t);
        }

        // 每 1000ms 运行一次：Telemetry 数据发送 (低频任务)
        switch (task_counter) {
			case 1: send_packet(1, cells[0]); break;
			case 2: send_packet(2, cells[1]); break;
			case 3: send_packet(3, cells[2]); break;
			case 4: send_packet(4, cells[3]); break;
			case 5: send_packet(5, (uint16_t)curr); break;
			case 6: send_packet(6, (uint16_t)(t * 10.0f)); break;
			case 7: {
				uint16_t min_v = 5000;
				for(int i=0; i<4; i++) if(cells[i] < min_v && cells[i] > 1000) min_v = cells[i];
				send_packet(7, Calculate_SOC(min_v));
				break;
			}
			case 8: send_packet(8, (uint16_t)debug_val); break;
			case 10: task_counter = 0; break; // 1秒周期结束，复位
			default: break;
		}
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timer_100ms_flag = 1; // 触发心跳标志
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 如果能跑到这里，说明 DMA 正常完成了任务，gState 会自动变回 READY
        // 你可以在这里放一个测试代码，比如翻转另一个 LED
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_button_time > 250) { // 简单消抖
            last_button_time = current_time;
            // 只设置一个触发标志
            button_pressed_flag = 1; 
        }
    }
}
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
