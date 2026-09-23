/* 文件用途：循迹模拟实现：用变量代替红外输入，只验证接口和协调逻辑，没有真正的循迹PID。 */
#include "Track.h"
#include "Motor.h"
#include "Mock.h"
volatile TrackState mock_line_state; /* 模拟红外状态，由测试或调试器设置。 */
volatile int16_t mock_line_error; /* 模拟黑线偏差，负数在左，正数在右。 */
volatile uint32_t mock_track_calls; /* 记录循迹控制调用次数，检查是否被错误调用。 */
volatile uint32_t mock_track_reset_calls;
volatile uint32_t mock_track_history;
static uint32_t sample_id;
static TrackStatus sample; /* 模拟模块对外发布的处理后状态。 */
static uint8_t stable_count; /* 连续正常的模拟样本计数，达到3次才报告正常。 */

/* 只清除控制历史，保留已经确认的感知结果。 */
void Track_Reset(void)
{
    ++mock_track_reset_calls;
    mock_track_history = 0U;
}
/* 初始化循迹模块；模拟版将输入设为无效，等待测试程序提供数据。 */
void Track_Init(void)
{
    mock_line_state = TRACK_INVALID;
    mock_line_error = 0;
    mock_track_calls = 0U;
    mock_track_reset_calls = 0U;
    stable_count = 0U;
    sample_id = 0U;
    sample.state = TRACK_INVALID;
    sample.error = 0;
    Track_Reset();
}
/* 维护红外状态、有效性和稳定性；不控制电机，避障期间也应继续调用。 */
void Track_SenseUpdate(void)
{
    ++sample_id;
    sample.state = mock_line_state;
    sample.error = mock_line_error;
    /* 这里只模拟连续3个样本的确认；真实模块必须确保它们是新采样。 */
    if (sample.state == TRACK_NORMAL) {
        if (stable_count < 3U) ++stable_count;
        if (stable_count < 3U) sample.state = TRACK_AMBIGUOUS;
    } else stable_count = 0U;
}
/* 返回已处理的线状态和偏差；这是读取结果，不代表又采样了一次。 */
TrackStatus Track_GetStatus(void) { return sample; }
/* 执行一步循迹运动控制；只有整车处于正常循迹时才调用。 */
void Track_Update(void)
{
    ++mock_track_calls;
    ++mock_track_history;
    if (sample.state == TRACK_NORMAL) Motor_SetDemand(200, 200);
    else Motor_Stop();
}

uint32_t Track_GetSampleId(void) { return sample_id; }
