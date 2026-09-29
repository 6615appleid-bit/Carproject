/* 文件用途：主机自动测试：用可控时间和模拟输入验证真正的CarControl代码，不需要连接单片机。 */
#include <assert.h>
#include <stdio.h>
#include "CarControl.h"
#include "CarConfig.h"
#include "Platform.h"
#include "Mock.h"
#include "Heading_Mock.h"

static uint32_t clock_ms; /* 测试专用时钟，可手动推进，也可以模拟计数回绕。 */
/* 读取当前毫秒计数；这是时刻，不是延时时间，也不会阻塞等待。 */
uint32_t Platform_GetMs(void) { return clock_ms; }
/* 初始化统一时基；STM32版配置SysTick，主机测试版仅初始化模拟时间。 */
void Platform_Init(void) { clock_ms = 0U; }
/* 测试时钟推进一个控制周期，然后调用整车更新。 */
static void tick(void)
{
    clock_ms += CAR_CONTROL_PERIOD_MS;
    CarControl_Update();
}
/* 测试准备：初始化并注入稳定黑线和有效测距，再验证可以启动。
 * 循迹/避障现在都要经过陀螺航向环，所以还要把惯导置成就绪，
 * 并打开替身的电机输出，才能断言「航向模块是唯一写电机的地方」。 */
static void prepare(void)
{
    CarControl_Init();
    assert(CarControl_GetState() == CAR_IDLE);
    assert(!CarControl_Start()); /* invalid initial inputs */
    mock_line_state = TRACK_NORMAL;
    mock_obstacle = AVOID_CHECK_CLEAR;
    mock_heading_ready = 1U;
    mock_heading_fault = 0U;
    mock_heading_yaw = 0.0f;
    mock_heading_drive = 1U;
    tick(); tick(); tick();
    assert(CarControl_Start());
}
/* 测试步骤：注入障碍并检查进入避障，交接当周期必须停车。 */
static void begin_avoid(void)
{
    mock_obstacle = AVOID_CHECK_DETECTED;
    tick();
    assert(CarControl_GetState() == CAR_AVOIDING);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
}
/* 正常线路本身不能触发交接，只有DONE才可交回循迹。 */
static void return_to_line(void)
{
    uint32_t before = mock_track_calls;
    uint32_t resets;
    uint32_t avoid_resets;
    uint32_t avoid_calls;
    TrackStatus sensed;
    mock_avoid_phase = AVOID_SEARCHING;
    mock_obstacle = AVOID_CHECK_CLEAR;
    mock_line_state = TRACK_NORMAL;
    tick(); tick(); tick(); tick();
    assert(Track_GetStatus().state == TRACK_NORMAL);
    assert(CarControl_GetState() == CAR_AVOIDING);
    assert(mock_track_calls == before);
    resets = mock_track_reset_calls;
    avoid_resets = mock_avoid_reset_calls;
    avoid_calls = mock_avoid_calls;
    sensed = Track_GetStatus();
    mock_track_history = 123U;
    mock_avoid_phase = AVOID_DONE;
    assert(Avoid_GetStatus() == AVOID_DONE);
    assert(Avoid_GetStatus() == AVOID_DONE); /* reads do not consume DONE */
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(Avoid_GetStatus() == AVOID_IDLE);
    assert(mock_avoid_calls == avoid_calls); /* no update on latched DONE */
    assert(mock_track_reset_calls == resets + 1U);
    assert(mock_avoid_reset_calls == avoid_resets + 1U);
    assert(mock_track_history == 0U);
    assert(Track_GetStatus().state == sensed.state);
    assert(Track_GetStatus().error == sensed.error);
    assert(mock_track_calls == before); /* no tracking control step during handoff */
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(mock_track_calls == before + 1U);
    assert(mock_track_reset_calls == resets + 1U); /* consume only once */
}
/* 程序入口；应用版循环调度整车，测试版执行各项断言验证。 */
int main(void)
{
    uint32_t before;
    unsigned int i;
    Platform_Init();
    prepare();
    before = mock_track_calls;
    CarControl_Update();
    assert(mock_track_calls == before); /* enforce period */
    /* ★ 循迹不再自己写电机：它只给出「目标航向角 + 目标速度」，
     *   由航向环（陀螺闭环）产生左右轮差速。 */
    tick();
    assert(mock_heading_target == mock_track_steer);
    assert(mock_heading_speed == mock_track_speed);
    assert(mock_motor_left == -100 && mock_motor_right == 100); /* 航向替身的固定差速 */
    assert(mock_heading_odo_resets >= 1U); /* 启动时里程清零 */
    /* 模拟连续4次避障，检查每次结束后可以再次启动。 */
    for (i = 0U; i < 4U; ++i) {
        before = mock_track_calls;
        begin_avoid();
        tick(); tick(); tick(); tick();
        assert(CarControl_GetState() == CAR_AVOIDING); /* line in BYPASS ignored */
        assert(mock_track_calls == before); /* exclusive motor ownership */
        return_to_line();
    }
    /* DONE produced inside Avoid_Update is handled in the same cycle. */
    prepare(); tick(); begin_avoid();
    mock_obstacle = AVOID_CHECK_CLEAR;
    mock_avoid_phase = AVOID_SEARCHING;
    tick(); /* keep the previous avoidance motor demand until handoff */
    assert(mock_motor_left != 0);
    before = mock_track_calls;
    mock_done_on_update = 1U;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(Avoid_GetStatus() == AVOID_IDLE);
    assert(mock_track_calls == before);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    assert(mock_track_history == 0U);
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(mock_track_calls == before + 1U);

    /* 验证终点停车会锁存，之后障碍变化或启动请求不能重新驱动。 */
    CarControl_ConfirmFinish();
    mock_obstacle = AVOID_CHECK_DETECTED;
    tick(); CarControl_Stop();
    assert(CarControl_GetState() == CAR_FINISHED);
    assert(!CarControl_Start());
    assert(mock_motor_left == 0 && mock_motor_right == 0);

    /* Wide black is still a line: tracking continues instead of latching a fault. */
    prepare();
    before = mock_track_calls;
    mock_line_state = TRACK_WIDE;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(CarControl_GetError() == CAR_ERROR_NONE);
    assert(mock_track_calls == before + 1U);

    prepare();
    mock_obstacle = AVOID_CHECK_INVALID;
    tick();
    assert(CarControl_GetError() == CAR_ERROR_RANGE);
    /* A brief loss stops the motors but is tolerated; the fault latches only when
     * CAR_TRACK_LOSS_TOLERANCE_MS passes without a stable line coming back. */
    prepare();
    mock_line_state = TRACK_LOST;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(CarControl_GetError() == CAR_ERROR_NONE);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    /* 循迹给出搜索命令（目标速度非 0）时整车不停车，而是原样转交给航向环，
     * 由它继续驱动车轮找线；搜索超时后循迹会把目标速度置 0，那时才停车。 */
    mock_track_search_steer = 20.0f;
    mock_track_search_speed = 15;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(mock_heading_target == 20.0f && mock_heading_speed == 15);
    assert(mock_motor_left == -100 && mock_motor_right == 100);
    mock_track_search_steer = 0.0f;
    mock_track_search_speed = 0;
    mock_line_state = TRACK_NORMAL;
    tick(); tick(); tick();
    assert(Track_GetStatus().state == TRACK_NORMAL); /* stable line is back */
    mock_line_state = TRACK_LOST;
    tick();
    clock_ms += CAR_TRACK_LOSS_TOLERANCE_MS / 2U;
    CarControl_Update();
    assert(CarControl_GetError() == CAR_ERROR_NONE); /* still inside the window */
    clock_ms += CAR_TRACK_LOSS_TOLERANCE_MS / 2U + CAR_CONTROL_PERIOD_MS;
    CarControl_Update();
    assert(CarControl_GetError() == CAR_ERROR_TRACK); /* the window expired */
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    CarControl_Stop();
    assert(!CarControl_Start()); /* fault is latched */
    prepare();
    mock_start_ok = 0U;
    mock_obstacle = AVOID_CHECK_DETECTED;
    tick();
    assert(CarControl_GetError() == CAR_ERROR_AVOID_START);
    prepare(); begin_avoid();
    mock_fail_on_update = 1U;
    tick();
    assert(CarControl_GetError() == CAR_ERROR_AVOID_FAILED);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    prepare(); begin_avoid();
    mock_line_state = TRACK_LOST;
    mock_avoid_phase = AVOID_SEARCHING;
    tick(); tick();
    assert(CarControl_GetState() == CAR_AVOIDING); /* lost line expected in search */
    clock_ms += CAR_AVOID_TIMEOUT_MS;
    CarControl_Update();
    assert(CarControl_GetError() == CAR_ERROR_AVOID_TIMEOUT);

    /* 将时钟放到32位上限附近，验证计数回绕后仍能正确判断超时。 */
    clock_ms = UINT32_MAX - 60U;
    prepare(); begin_avoid();
    tick(); tick(); tick();
    assert(CarControl_GetState() == CAR_AVOIDING); /* timer wrap handled */
    clock_ms += CAR_AVOID_TIMEOUT_MS;
    CarControl_Update();
    assert(CarControl_GetError() == CAR_ERROR_AVOID_TIMEOUT);

    /* ★ 里程到阈值自动停车并锁存：左右轮累计计数之和 ≥ CAR_ODOMETER_STOP_COUNTS。 */
    prepare();
#if CAR_ODOMETER_STOP_COUNTS > 0
    mock_heading_odo_left = CAR_ODOMETER_STOP_COUNTS / 2;
    mock_heading_odo_right = CAR_ODOMETER_STOP_COUNTS - mock_heading_odo_left - 1;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING); /* 差一个计数还不算到点 */
    assert(mock_motor_left != 0 || mock_motor_right != 0);
    mock_heading_odo_right = CAR_ODOMETER_STOP_COUNTS - mock_heading_odo_left;
    tick();
    assert(CarControl_GetState() == CAR_FINISHED);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    assert(!CarControl_Start()); /* 锁存后不能再启动 */
    CarControl_Init();           /* 只有 Init 才解除锁存 */
    assert(CarControl_GetState() == CAR_IDLE);
#endif

    /* ★ 惯导未就绪时不允许启动：循迹现在也要经过航向环。 */
    prepare();
    CarControl_Stop();
    mock_heading_ready = 0U;
    assert(!CarControl_Start());
    assert(CarControl_GetError() == CAR_ERROR_IMU);

    /* ★ 运行中惯导读取失败 → 立即故障停车，不再继续驱动车轮。 */
    prepare();
    mock_heading_fault = 1U;
    tick();
    assert(CarControl_GetState() == CAR_FAULT);
    assert(CarControl_GetError() == CAR_ERROR_IMU);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    mock_heading_fault = 0U;

    /* ★ 航向模式：顶层只给目标角和目标速度，不驱动循迹/避障。 */
    CarControl_Init();
    mock_heading_drive = 1U;
    assert(CarControl_StartHeading());
    CarControl_SetHeading(90.0f);
    CarControl_SetSpeed(0);
    before = mock_track_calls;
    tick();
    assert(CarControl_GetState() == CAR_HEADING);
    assert(mock_heading_target == 90.0f && mock_heading_speed == 0);
    assert(mock_track_calls == before); /* 航向模式不调循迹 */
    assert(mock_heading_odo_resets >= 2U); /* 进入航向模式也清里程 */
    CarControl_Stop();

    prepare(); tick();
    CarControl_Stop();
    assert(CarControl_GetState() == CAR_IDLE);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    mock_line_state = TRACK_LOST;
    assert(!CarControl_Start()); /* Start samples again; stale NORMAL cannot start */
    mock_line_state = TRACK_NORMAL;
    tick(); tick(); tick();
    assert(CarControl_Start());
    puts("PASS: scheduling, tracking through the heading loop, four avoidance cycles,");
    puts("      DONE handoff before/after update, invalid data, start failure, lost line,");
    puts("      timeout, timer wrap, odometer auto stop, IMU fault, stop/restart,");
    puts("      motor ownership (Heading only), finish/fault latching.");
    return 0;
}
