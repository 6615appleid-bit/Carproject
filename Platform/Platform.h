/* 文件用途：平台计时接口：向上层提供系统毫秒计数，隐藏STM32底层实现。 */
#ifndef PLATFORM_H
#define PLATFORM_H
#include <stdint.h>
/* 初始化统一时基；STM32版配置SysTick，主机测试版仅初始化模拟时间。 */
void Platform_Init(void);
/* 读取当前毫秒计数；这是时刻，不是延时时间，也不会阻塞等待。 */
uint32_t Platform_GetMs(void);
#endif
