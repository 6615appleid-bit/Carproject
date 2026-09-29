/* 航向模块测试替身实现。接口与 Drivers/Heading.h 一一对应，
 * 但内部不读硬件、不算控制，只记录调用并回放测试设定的值。 */
#include "Heading.h"
#include "Heading_Mock.h"
#include "Motor.h"

volatile uint8_t  mock_heading_ready;
volatile uint8_t  mock_heading_fault;
volatile uint8_t  mock_heading_drive;
volatile float    mock_heading_yaw;
volatile uint32_t mock_heading_update_calls;
volatile uint32_t mock_heading_reset_calls;
volatile uint32_t mock_heading_init_calls;
volatile uint32_t mock_heading_stop_calls;
volatile uint32_t mock_heading_odo_resets;
volatile float    mock_heading_target;
volatile int16_t  mock_heading_speed;
volatile int32_t  mock_heading_odo_left;
volatile int32_t  mock_heading_odo_right;

void Heading_MockReset(void)
{
    mock_heading_ready        = 1U;
    mock_heading_fault        = 0U;
    mock_heading_drive        = 0U;
    mock_heading_yaw          = 0.0f;
    mock_heading_update_calls = 0U;
    mock_heading_reset_calls  = 0U;
    mock_heading_init_calls   = 0U;
    mock_heading_stop_calls   = 0U;
    mock_heading_odo_resets   = 0U;
    mock_heading_target       = 0.0f;
    mock_heading_speed        = 0;
    mock_heading_odo_left     = 0;
    mock_heading_odo_right    = 0;
}

void Heading_Init(void)
{
    ++mock_heading_init_calls;
    mock_heading_target = 0.0f;
    mock_heading_speed  = 0;
}

uint8_t Heading_ImuInit(void)      { return 0U; }
uint8_t Heading_CalibrateGyro(void) { return 0U; }
uint8_t Heading_IsImuReady(void)   { return mock_heading_ready; }

void  Heading_SetTarget(float deg)          { mock_heading_target = deg; }
void  Heading_SetTargetRelative(float delta) { mock_heading_target += delta; }
float Heading_GetTarget(void)               { return mock_heading_target; }
float Heading_GetYaw(void)                  { return mock_heading_yaw; }

void Heading_SetSpeed(int16_t speed_cmd) { mock_heading_speed = speed_cmd; }
int16_t Heading_GetSpeed(void)           { return (int16_t)(mock_heading_speed / 2); }

void Heading_Reset(void)
{
    ++mock_heading_reset_calls;
}

/* 与真实实现同一步骤：设定目标角和目标速度，然后更新一次输出。 */
void Heading_Drive(const DriveCmd *cmd)
{
    if (cmd == 0) { Heading_Stop(); return; }
    Heading_SetTarget(cmd->steer_deg);
    Heading_SetSpeed(cmd->speed_cmd);
    Heading_Update();
}

/* 停车并清控制状态。替身用调用计数 + 电机归零来体现。 */
void Heading_Stop(void)
{
    ++mock_heading_stop_calls;
    Motor_Stop();
}

/* 只清累计里程。 */
void Heading_ResetOdometer(void)
{
    ++mock_heading_odo_resets;
    mock_heading_odo_left = 0;
    mock_heading_odo_right = 0;
}

/* 只有测试显式允许时才写电机，这样「谁控制了电机」可以被精确断言。
 * 输出一组固定且左右可辨认的 PWM：左 -100 / 右 +100（原地左转）。 */
void Heading_Update(void)
{
    ++mock_heading_update_calls;
    if (mock_heading_drive != 0U) {
        Motor_SetDemand(-100, 100);
    }
}

uint8_t Heading_HasFault(void)         { return mock_heading_fault; }
int16_t Heading_GetEncoderLeft(void)   { return 1; }
int16_t Heading_GetEncoderRight(void)  { return 2; }
int32_t Heading_GetOdometerLeft(void)  { return mock_heading_odo_left; }
int32_t Heading_GetOdometerRight(void) { return mock_heading_odo_right; }
int16_t Heading_GetSpeedOut(void)      { return 0; }
int16_t Heading_GetTurnOut(void)       { return 0; }
float   Heading_GetGyroZ(void)         { return 0.0f; }
