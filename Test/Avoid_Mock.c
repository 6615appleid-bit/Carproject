/* 文件用途：避障模拟实现：测试程序手动设置阶段，没有实际测距、绕行路径或找线算法。 */
#include "Avoid.h"
#include "Mock.h"
#include "Motor.h"
volatile AvoidCheck mock_obstacle; /* 模拟测距处理结果：无效、无障碍、有障碍。 */
volatile AvoidStatus mock_avoid_phase; /* 模拟避障当前阶段，可由测试手动推进。 */
volatile uint8_t mock_start_ok; /* 设为0可模拟避障启动失败。 */
volatile uint8_t mock_fail_on_update; /* 设为1可模拟执行避障时发生故障。 */
volatile uint8_t mock_done_on_update;
volatile uint32_t mock_avoid_reset_calls;
volatile uint32_t mock_avoid_calls; /* 记录避障控制调用次数，检查控制权是否互斥。 */
/* 初始化避障模块和内部状态。 */
void Avoid_Init(void)
{
    mock_obstacle = AVOID_CHECK_INVALID;
    mock_start_ok = 1U;
    mock_fail_on_update = 0U;
    mock_avoid_calls = 0U;
    mock_done_on_update = 0U;
    mock_avoid_reset_calls = 0U;
    Avoid_Reset();
}
/* 结束或取消本次避障并回到空闲；不控制电机，停车由上层负责。 */
void Avoid_Reset(void) { ++mock_avoid_reset_calls; mock_avoid_phase = AVOID_IDLE; }
/* 维护测距和障碍检测状态，不控制电机；真实模块内部管理采样周期与有效性。 */
void Avoid_SenseUpdate(void) { /* controlled by test/debugger */ }
/* 读取障碍判断：无效、无障碍或有障碍；不启动避障，也不控制电机。 */
AvoidCheck Avoid_Check(void) { return mock_obstacle; }
/* 开始一次避障并进入绕行阶段；返回非零表示成功，本函数不直接驱动电机。 */
uint8_t Avoid_Start(void)
{
    if (!mock_start_ok) return 0U;
    mock_avoid_phase = AVOID_BYPASSING;
    return 1U;
}
/* 执行一步避障或找线；line指向本轮黑线状态，只读取，不修改；本函数可以控制电机。 */
void Avoid_Update(const TrackStatus *line)
{
    (void)line; /* real module may use state/error to guide search */ /* 模拟版暂不使用黑线参数；真实避障可以据此调整找线动作。 */
    ++mock_avoid_calls;
    if (mock_avoid_phase == AVOID_DONE) return; /* 完成状态锁存，不再运动。 */
    /* 故障注入分支：用于验证CarControl能发现模块失败并锁定停车。 */
    if (mock_fail_on_update) {
        mock_avoid_phase = AVOID_FAILED;
        Motor_Stop();
    } else if (mock_done_on_update && mock_avoid_phase == AVOID_SEARCHING) {
        mock_avoid_phase = AVOID_DONE; /* 模拟负责人已完成稳定回线判断。 */
    } else Motor_SetDemand(180, 80);
}
/* 读取状态不会清除DONE。 */
AvoidStatus Avoid_GetStatus(void) { return mock_avoid_phase; }
