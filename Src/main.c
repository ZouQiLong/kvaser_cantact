#include "stm32f0xx_hal.h"  // 包含STM32F0系列的HAL库头文件，提供硬件抽象层相关的函数和定义
#include "xcan_timestamp.h" // 包含xcan时间戳相关的头文件，可能用于时间戳功能的实现
#include "xcan_led.h"       // 包含xcan LED相关的头文件，可能用于LED控制功能的实现
#include "xcan_protocol.h"  // 包含xcan协议相关的头文件，可能用于协议处理功能的实现
#include "xcan_usb.h"       // 包含xcan USB相关的头文件，可能用于USB通信功能的实现

// HAL_MspInit函数，用于初始化MCU的系统和外设时钟
void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();  // 使能系统配置控制器（SYSCFG）的时钟
  __HAL_RCC_PWR_CLK_ENABLE();     // 使能电源控制（PWR）的时钟

  __HAL_RCC_GPIOF_CLK_ENABLE();   // 使能GPIOF端口的时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();   // 使能GPIOA端口的时钟
  __HAL_RCC_GPIOB_CLK_ENABLE();   // 使能GPIOB端口的时钟
}

/* EXTERNAL CLOCK */
#if EXTERNAL_CLOCK  // 如果定义了EXTERNAL_CLOCK宏，则使用外部时钟配置
// SystemClock_Config函数，用于配置系统时钟
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // 定义并初始化RCC振荡器初始化结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 定义并初始化RCC时钟初始化结构体
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0}; // 定义并初始化RCC外设时钟初始化结构体


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 设置振荡器类型为外部高速时钟（HSE）
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;  // 使能外部高速时钟（HSE）
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  // 使能锁相环（PLL）
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // 设置PLL的时钟源为HSE

#if( HSE_VALUE == 16000000u )  // 如果HSE的值为16MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;  // 设置PLL的倍频系数为3
#elif( HSE_VALUE == 8000000u )  // 如果HSE的值为8MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;  // 设置PLL的倍频系数为6
#else
#error invalid HSE_VALUE  // 如果HSE的值不是16MHz或8MHz，报错
#endif
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;  // 设置PLL的预分频系数为1

  HAL_RCC_OscConfig(&RCC_OscInitStruct);  // 根据RCC_OscInitStruct的配置初始化RCC振荡器

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;  // 设置要配置的时钟类型为HCLK、SYSCLK和PCLK1
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 设置系统时钟源为PLL时钟
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 设置AHB时钟分频系数为1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  // 设置APB1时钟分频系数为1

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );  // 根据RCC_ClkInitStruct的配置初始化RCC时钟，并设置FLASH延迟为1

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;  // 选择要配置的外设时钟为USB时钟
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;  // 设置USB时钟源为PLL

  HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );  // 根据PeriphClkInit的配置初始化RCC外设时钟
}
#else  // 如果没有定义EXTERNAL_CLOCK宏，则使用内部时钟配置
/* internal clock with USB SOF sync */
// SystemClock_Config函数，用于配置系统时钟
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;  // 定义RCC振荡器初始化结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct;  // 定义RCC时钟初始化结构体
  RCC_PeriphCLKInitTypeDef PeriphClkInit;  // 定义RCC外设时钟初始化结构体

  RCC_CRSInitTypeDef pInit = {0};  // 定义并初始化RCC时钟恢复系统（CRS）初始化结构体

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;  // 设置振荡器类型为内部48MHz高速时钟（HSI48）
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;  // 使能内部48MHz高速时钟（HSI48）
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;  // 不使用PLL
  HAL_RCC_OscConfig(&RCC_OscInitStruct);  // 根据RCC_OscInitStruct的配置初始化RCC振荡器

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;  // 设置要配置的时钟类型为HCLK、SYSCLK和PCLK1
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;  // 设置系统时钟源为HSI48
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 设置AHB时钟分频系数为1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  // 设置APB1时钟分频系数为1
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);  // 根据RCC_ClkInitStruct的配置初始化RCC时钟，并设置FLASH延迟为1

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;  // 选择要配置的外设时钟为USB和I2C1时钟
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;  // 设置USB时钟源为HSI48
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;  // 设置I2C1时钟源为HSI
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);  // 根据PeriphClkInit的配置初始化RCC外设时钟

  /** Configures CRS */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;  // 设置CRS同步预分频系数为1
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;  // 设置CRS同步源为USB
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;  // 设置CRS同步极性为上升沿
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE( 48000000, 1000 );  // 计算并设置CRS的重载值
  pInit.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;  // 设置CRS的误差限制值为默认值
  pInit.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;  // 设置HSI48的校准值为默认值

  HAL_RCCEx_CRSConfig( &pInit );  // 根据pInit的配置初始化CRS
}
#endif

// SysTick_Handler函数，系统滴答定时器的中断处理函数
void SysTick_Handler( void )
{
  HAL_IncTick();  // 增加系统滴答定时器的计数值
}

// main函数，程序的入口点
int main( void )
{
  HAL_Init();  // 初始化HAL库

  SystemClock_Config();  // 配置系统时钟
  
  xcan_usb_init();  // 初始化xcan USB功能
  xcan_led_init();  // 初始化xcan LED功能
  xcan_timestamp_init();  // 初始化xcan时间戳功能
  xcan_protocol_init();  // 初始化xcan协议功能

  for(;;)  // 无限循环
  {
    xcan_usb_poll();  // 轮询xcan USB功能
    xcan_led_poll();  // 轮询xcan LED功能
    xcan_protocol_poll();  // 轮询xcan协议功能
  }
}
