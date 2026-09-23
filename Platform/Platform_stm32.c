/* 文件用途：STM32平台计时实现：SysTick每毫秒更新计数，不占用超声波使用的TIM4。 */
#include "stm32f10x.h"
#include "Platform.h"
static volatile uint32_t milliseconds; /* 中断修改、主循环读取；volatile要求实际读取变量。 */
/* 初始化统一时基；STM32版配置SysTick，主机测试版仅初始化模拟时间。 */
void Platform_Init(void)
{
    SystemCoreClockUpdate(); /* 根据时钟寄存器更新主频变量，再计算1毫秒需要多少时钟周期。 */
    milliseconds = 0U;
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) { /* 配置失败时停在下面的循环中，避免在错误时基下运行。 */
        while (1) { /* invalid clock setup */ }
    }
}
/* SysTick中断服务，每1毫秒将系统计数加1；这里不执行循迹或避障。 */
void SysTick_Handler(void) { ++milliseconds; }
/* 读取当前毫秒计数；这是时刻，不是延时时间，也不会阻塞等待。 */
uint32_t Platform_GetMs(void) { return milliseconds; }
//提供统一时钟  CarControl利用这个时间：每隔 10 ms执行一轮控制，判断避障是否超过规定时间。
