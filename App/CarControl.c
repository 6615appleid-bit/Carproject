/* 文件用途：整车协调实现：只负责调用谁、何时切换、何时停车；循迹和绕障算法由各自模块实现。 */
#include "CarControl.h"
#include "CarConfig.h"
#include "Track.h"
#include "Avoid.h"
#include "Motor.h"
#include "Platform.h"

static uint32_t line_lost_ms; /* when the current line-loss window started */
static int16_t last_line_error;
static uint8_t recovery_allowed;
static uint8_t line_lost; /* nonzero while the loss tolerance window is open */

static CarState state = CAR_IDLE; /* 当前整车状态，仅在本文件内维护。 */
static CarError error_code = CAR_ERROR_NONE; /* 保存最近的故障原因，便于查询。 */
static uint32_t last_step_ms; /* 上一次整车协调执行的时刻。 */
static uint32_t avoidance_started_ms; /* 本次避障的开始时刻，用于总超时判断。 */

/* 内部故障处理：先停车，再重置模块，保存原因并进入故障锁存状态。 */
static void Fail(CarError reason)
{
    Motor_Stop();
    Avoid_Reset();
    Track_Reset();
    error_code = reason;
    state = CAR_FAULT;
}

/* 避障模块已确认稳定回线：停止运动，按顺序复位，再交回循迹。
 * Track_Reset只清理控制历史，保留已确认的感知结果。
 */
static void CompleteAvoidance(void)
{
    Motor_Stop();
    Avoid_Reset();
    Track_Reset();
    line_lost = 0U;
    recovery_allowed = 0U;
    last_line_error = 0;
    state = CAR_TRACKING;
}

/* 初始化模块并停止电机，进入等待状态；也用于非比赛期间解除终点或故障锁存。 */
void CarControl_Init(void)
{
    Motor_Init();
    Motor_Stop();
    Track_Init();
    Avoid_Init();
    error_code = CAR_ERROR_NONE;
    state = CAR_IDLE;
    line_lost = 0U;
    recovery_allowed = 0U;
    last_line_error = 0;
    last_step_ms = Platform_GetMs();
}

/* 请求启动：只有等待状态、稳定黑线和有效测距才成功；返回1成功，0拒绝。 */
uint8_t CarControl_Start(void)
{
    TrackStatus line;
    if (state != CAR_IDLE) return 0U;
    /* Sense calls do not move the car. Start requires usable, stable data. */
    Track_SenseUpdate();
    Avoid_SenseUpdate();
    line = Track_GetStatus();
    /* Wide black is a line as well: only lost/ambiguous/invalid line data or an
     * unusable range reading keeps the car from starting. */
    if ((line.state != TRACK_NORMAL && line.state != TRACK_WIDE) ||
        Avoid_Check() == AVOID_CHECK_INVALID)
        return 0U;
    error_code = CAR_ERROR_NONE;
    state = CAR_TRACKING;
    line_lost = 0U;
    recovery_allowed = 0U;
    last_line_error = line.error;
    last_step_ms = Platform_GetMs();
    return 1U;
}

/* 立即停车并重置模块；运行状态回到等待，但不会解除终点或故障锁存。 */
void CarControl_Stop(void)
{
    Motor_Stop();
    Avoid_Reset();
    Track_Reset();
    line_lost = 0U;
    recovery_allowed = 0U;
    last_line_error = 0;
    if (state != CAR_FAULT && state != CAR_FINISHED) state = CAR_IDLE;
}

/* 通知整车已经确认到达终点并锁定停车；本函数不负责识别终点。 */
void CarControl_ConfirmFinish(void)
{
    if (state != CAR_TRACKING && state != CAR_AVOIDING) return;
    CarControl_Stop();
    state = CAR_FINISHED;
}

/* 执行一轮整车协调；内部限制为每10毫秒一步，未到时间立即返回。 */
void CarControl_Update(void)

{
    uint32_t now = Platform_GetMs();
    TrackStatus line;
    AvoidCheck obstacle;
    AvoidStatus phase;
    /* 未到周期不重复控制；无符号差值可处理正常范围内的计数回绕。 */
    if ((uint32_t)(now - last_step_ms) < CAR_CONTROL_PERIOD_MS) return;
    last_step_ms = now; /* do not replay missed control steps on stale data */

    /* 终点或故障后持续停车，不再调用运动模块，防止重新驱动车轮。 */
    if (state == CAR_FINISHED || state == CAR_FAULT) {
        Motor_Stop();
        return;
    }
    Track_SenseUpdate();
    Avoid_SenseUpdate();
    /* 等待启动时可以更新感知，但不能运动。 */
    if (state == CAR_IDLE) {
        Motor_Stop();
        return;
    }
    line = Track_GetStatus();
    obstacle = Avoid_Check();
    /* 测距失效不能当成前方畅通，运行中进入故障停车。 */
    if (obstacle == AVOID_CHECK_INVALID) {
        Fail(CAR_ERROR_RANGE);
        return;
    }
    /* 循迹阶段：先检查数据和障碍，再决定是否执行循迹控制。 */
    if (state == CAR_TRACKING) {
        /* Obstacles take priority even while recovering a lost line. */
        if (obstacle == AVOID_CHECK_DETECTED) {
            line_lost = 0U;
            recovery_allowed = 0U;
            Motor_Stop();
            Track_Reset();
            if (!Avoid_Start()) {
                Fail(CAR_ERROR_AVOID_START);
                return;
            }
            avoidance_started_ms = now;
            state = CAR_AVOIDING;
            return;
        }
        if (line.state != TRACK_NORMAL && line.state != TRACK_WIDE) {
            if (!line_lost) {
                line_lost = 1U;
                line_lost_ms = now;
                recovery_allowed = (line.state == TRACK_LOST);
                Track_Reset();
            }
            if ((uint32_t)(now - line_lost_ms) >= CAR_TRACK_LOSS_TOLERANCE_MS) {
                Fail(CAR_ERROR_TRACK);
                return;
            }
            /* INVALID must stop immediately. AMBIGUOUS also covers the first
             * two returning line samples, so it may continue an existing search
             * within the SAME deadline. It never starts a search by itself. */
            if (line.state == TRACK_INVALID) recovery_allowed = 0U;
            if (recovery_allowed &&
                (line.state == TRACK_LOST || line.state == TRACK_AMBIGUOUS)) {
                int32_t correction = (int32_t)last_line_error *
                    CAR_TRACK_RECOVERY_CORRECTION / 1000;
                Motor_SetDemand((int16_t)(CAR_TRACK_RECOVERY_DEMAND + correction),
                                (int16_t)(CAR_TRACK_RECOVERY_DEMAND - correction));
            } else Motor_Stop();
            return;
        }
        line_lost = 0U;
        recovery_allowed = 0U;
        last_line_error = line.error;
        Track_Update();
        return;
    }

    /* 绕行和找线合计超时就停车，避免长时间找不到线仍继续运动。 */
    if ((uint32_t)(now - avoidance_started_ms) >= CAR_AVOID_TIMEOUT_MS) {
        Fail(CAR_ERROR_AVOID_TIMEOUT);
        return;
    }
    phase = Avoid_GetStatus();
    if (phase == AVOID_FAILED || phase == AVOID_IDLE) {
        Fail(CAR_ERROR_AVOID_FAILED);
        return;
    }
    /* 完成事件由避障模块锁存，整车不再自行判断回线。 */
    if (phase == AVOID_DONE) {
        CompleteAvoidance();
        return;
    }
    /* 寻线时允许暂时丢线，但数据不可用仍需故障停车。 */
    if (phase == AVOID_SEARCHING && line.state == TRACK_INVALID) {
        Fail(CAR_ERROR_TRACK);
        return;
    }
    /* 传入本轮线状态的地址；避障模块可以读取它执行一步动作。 */
    Avoid_Update(&line);
    phase = Avoid_GetStatus();
    /* 本轮更新可能刚刚完成避障，立即停车交接。 */
    if (phase == AVOID_DONE) {
        CompleteAvoidance();
        return;
    }
    /* 避障运行阶段报告失败或意外空闲，都视为异常。 */
    if (phase == AVOID_FAILED || phase == AVOID_IDLE)
        Fail(CAR_ERROR_AVOID_FAILED);
}

/* 读取整车当前状态，不改变状态，也不控制电机。 */
CarState CarControl_GetState(void) { return state; }
/* 读取故障原因，用于调试定位；无故障时返回CAR_ERROR_NONE。 */
CarError CarControl_GetError(void) { return error_code; }
