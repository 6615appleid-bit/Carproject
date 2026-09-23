/* 文件用途：主机自动测试：用可控时间和模拟输入验证真正的CarControl代码，不需要连接单片机。 */
#include <assert.h>
#include <stdio.h>
#include "CarControl.h"
#include "CarConfig.h"
#include "Platform.h"
#include "Mock.h"

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
/* 测试准备：初始化并注入稳定黑线和有效测距，再验证可以启动。 */
static void prepare(void)
{
    CarControl_Init();
    assert(CarControl_GetState() == CAR_IDLE);
    assert(!CarControl_Start()); /* invalid initial inputs */
    mock_line_state = TRACK_NORMAL;
    mock_obstacle = AVOID_CHECK_CLEAR;
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
    assert(mock_track_calls == before); /* no tracking motor write on handoff */
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
    tick();
    assert(mock_motor_left == 200 && mock_motor_right == 200);
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
    /* A brief loss searches at low demand; the fault latches only when
     * CAR_TRACK_LOSS_TOLERANCE_MS passes without a stable line coming back. */
    prepare();
    mock_line_state = TRACK_LOST;
    tick();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(CarControl_GetError() == CAR_ERROR_NONE);
    assert(mock_motor_left == CAR_TRACK_RECOVERY_DEMAND);
    assert(mock_motor_right == CAR_TRACK_RECOVERY_DEMAND);
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
    /* Search keeps the most recent steering sign, including confirmation frames. */
    for (i = 0U; i < 2U; ++i) {
        prepare();
        mock_line_error = i ? 1000 : -1000;
        tick();
        before = mock_track_calls;
        mock_line_state = TRACK_LOST;
        tick();
        assert(mock_track_calls == before);
        assert(mock_motor_left == CAR_TRACK_RECOVERY_DEMAND +
               (i ? CAR_TRACK_RECOVERY_CORRECTION : -CAR_TRACK_RECOVERY_CORRECTION));
        assert(mock_motor_right == CAR_TRACK_RECOVERY_DEMAND -
               (i ? CAR_TRACK_RECOVERY_CORRECTION : -CAR_TRACK_RECOVERY_CORRECTION));
        mock_line_state = TRACK_NORMAL;
        tick(); tick();
        assert(mock_motor_left > 0 && mock_motor_right > 0);
        assert(mock_track_calls == before);
        tick();
        assert(mock_track_calls == before + 1U);
        assert(CarControl_GetError() == CAR_ERROR_NONE);
    }
    /* Invalid data cancels motion for the rest of this window. */
    prepare(); tick();
    mock_line_state = TRACK_LOST;
    tick();
    mock_line_state = TRACK_INVALID;
    tick();
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    mock_line_state = TRACK_LOST;
    tick();
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    clock_ms += CAR_TRACK_LOSS_TOLERANCE_MS;
    CarControl_Update();
    assert(CarControl_GetError() == CAR_ERROR_TRACK);

    prepare(); tick();
    mock_line_state = TRACK_AMBIGUOUS;
    tick();
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    mock_line_state = TRACK_LOST;
    tick();
    assert(mock_motor_left == 0 && mock_motor_right == 0);

    /* Alternating unconfirmed/LOST frames cannot renew the deadline; wrap is safe. */
    clock_ms = UINT32_MAX - 100U;
    prepare(); tick();
    mock_line_state = TRACK_LOST;
    tick();
    clock_ms += CAR_TRACK_LOSS_TOLERANCE_MS - CAR_CONTROL_PERIOD_MS;
    mock_line_state = TRACK_NORMAL;
    CarControl_Update();
    assert(CarControl_GetState() == CAR_TRACKING);
    assert(mock_motor_left > 0);
    mock_line_state = TRACK_LOST;
    tick();
    assert(CarControl_GetError() == CAR_ERROR_TRACK);
    assert(mock_motor_left == 0 && mock_motor_right == 0);

    /* Obstacles and operator stop interrupt recovery immediately. */
    prepare(); tick();
    mock_line_state = TRACK_LOST;
    tick(); begin_avoid();
    prepare(); tick();
    mock_line_state = TRACK_LOST;
    mock_obstacle = AVOID_CHECK_INVALID;
    tick();
    assert(CarControl_GetError() == CAR_ERROR_RANGE);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    prepare(); tick();
    mock_line_state = TRACK_LOST;
    tick(); CarControl_Stop(); tick();
    assert(CarControl_GetState() == CAR_IDLE);
    assert(mock_motor_left == 0 && mock_motor_right == 0);

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

    prepare(); tick();
    CarControl_Stop();
    assert(CarControl_GetState() == CAR_IDLE);
    assert(mock_motor_left == 0 && mock_motor_right == 0);
    mock_line_state = TRACK_LOST;
    assert(!CarControl_Start()); /* Start samples again; stale NORMAL cannot start */
    mock_line_state = TRACK_NORMAL;
    tick(); tick(); tick();
    assert(CarControl_Start());
    puts("PASS: scheduling, tracking, four avoidance cycles, DONE handoff before/after update,");
    puts("      invalid data, start failure, lost line, timeout, timer wrap,");
    puts("      stop/restart, motor ownership, finish/fault latching.");
    return 0;
}
