/* 文件用途：电机模拟实现：只保存左右轮指令，不配置GPIO、不输出PWM，真实接入时需要替换。 */
#include "Motor.h"
volatile int16_t mock_motor_left; /* 保存左轮模拟需求，不代表真实车轮速度。 */
volatile int16_t mock_motor_right; /* 保存右轮模拟需求，不代表真实车轮速度。 */
/* 初始化电机驱动并保持停止；模拟版仅清零指令变量。 */
void Motor_Init(void) { Motor_Stop(); }
/* 停止左右电机；真实实现应明确制动还是滑行，模拟版只将指令清零。 */
void Motor_Stop(void) { mock_motor_left = 0; mock_motor_right = 0; }
/* 把输入限制在-1000至1000，避免模拟指令超出接口约定范围。 */
static int16_t Limit(int16_t value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return value;
}
/* 设置左右轮归一化需求，范围-1000至1000；正数前进，负数后退，不是实际转速。 */
void Motor_SetDemand(int16_t left, int16_t right)
{
    mock_motor_left = Limit(left);
    mock_motor_right = Limit(right);
}
