/* 文件用途：整车协调实现：只负责调用谁、何时切换、何时停车；循迹和绕障算法由各自模块实现。 */
#include "CarControl.h"
#include "CarConfig.h"
#include "Track.h"
#include "Avoid.h"
#include "Motor.h"
#include "Platform.h"
#include "Heading.h"

static uint32_t line_lost_ms; /* when the current line-loss window started */
static uint8_t line_lost; /* nonzero while the loss tolerance window is open */

static CarState state = CAR_IDLE; /* 当前整车状态，仅在本文件内维护。 */
static CarError error_code = CAR_ERROR_NONE; /* 保存最近的故障原因，便于查询。 */
static uint32_t last_step_ms; /* 上一次整车协调执行的时刻。 */
static uint32_t avoidance_started_ms; /* 本次避障的开始时刻，用于总超时判断。 */
static DriveCmd last_drive; /* 最近一次下发给航向环的转向命令，仅用于观察。 */

/* 内部故障处理：先停车，再重置模块，保存原因并进入故障锁存状态。 */
static void Fail(CarError reason)
{
    Motor_Stop();
    Avoid_Reset();
    Track_Reset();
    error_code = reason;
    state = CAR_FAULT;
}

/* ★ 全车唯一的驱动出口：把运动模块给出的「目标航向角 + 目标速度」交给航向环。
 * 差速、转向环、速度环都在 Heading 里，本文件只负责选择谁在说话。 */
static void ApplyDrive(const DriveCmd *cmd)
{
    last_drive = *cmd;
    Heading_Drive(cmd);
    if (Heading_HasFault() != 0U) { Fail(CAR_ERROR_IMU); }
}

/* 避障模块已确认稳定回线：停止运动，按顺序复位，再交回循迹。
 * Track_Reset只清理控制历史，保留已确认的感知结果。
 */
static void CompleteAvoidance(void)
{
    Heading_Stop(); /* 停车并清速度环积分，下一段循迹从零输出重新起步 */
    Avoid_Reset();
    Track_Reset();
    line_lost = 0U;
    state = CAR_TRACKING;
}

/* 初始化模块并停止电机，进入等待状态；也用于非比赛期间解除终点或故障锁存。 */
void CarControl_Init(void)
{
    Motor_Init();
    Motor_Stop();
    Heading_Init(); /* 编码器 + 航向环状态；MPU6050 的初始化和标定由 main 单独触发 */
    Heading_ResetOdometer(); /* 本次运行的里程从这里开始算 */
    Track_Init();
    Avoid_Init();
    error_code = CAR_ERROR_NONE;
    state = CAR_IDLE;
    line_lost = 0U;
    last_step_ms = Platform_GetMs();
}

/* 请求启动：只有等待状态、稳定黑线、有效测距和就绪的惯导才成功；返回1成功，0拒绝。
 * 循迹现在也经过陀螺航向环，所以没有惯性单元时不允许启动。 */
uint8_t CarControl_Start(void)
{
    TrackStatus line;
    if (state != CAR_IDLE) return 0U;
    if (Heading_IsImuReady() == 0U) {
        error_code = CAR_ERROR_IMU;
        return 0U;
    }
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
    Heading_Stop();          /* 清速度环积分，避免上一段运行的残留 */
    Heading_ResetOdometer(); /* 本次运行的里程从启动开始算 */
    state = CAR_TRACKING;
    line_lost = 0U;
    last_step_ms = Platform_GetMs();
    return 1U;
}

/* 立即停车并重置模块；运行状态回到等待，但不会解除终点或故障锁存。 */
void CarControl_Stop(void)
{
    Motor_Stop();
    /* 航向环复位：当前朝向重新定义为 0°，清速度环积分和故障。
     * 目标角和目标速度保留，所以「停车→再启动」不会丢掉顶层设定。 */
    Heading_Reset();
    Avoid_Reset();
    Track_Reset();
    line_lost = 0U;
    if (state != CAR_FAULT && state != CAR_FINISHED) state = CAR_IDLE;
}

/* 通知整车已经确认到达终点并锁定停车；本函数不负责识别终点。 */
void CarControl_ConfirmFinish(void)
{
    if (state != CAR_TRACKING && state != CAR_AVOIDING) return;
    CarControl_Stop();
    state = CAR_FINISHED;
}

/* ============================================================================
 * 航向模式
 * ==========================================================================*/

/* 从等待状态进入航向模式。当前朝向被重新定义为 0°。
 * 惯性单元未就绪时返回 0 并把故障码置为 CAR_ERROR_IMU。*/
uint8_t CarControl_StartHeading(void)
{
    if (state != CAR_IDLE) return 0U;
    if (Heading_IsImuReady() == 0U) {
        error_code = CAR_ERROR_IMU;
        return 0U;
    }
    Motor_Stop();
    Heading_Reset();                /* 当前朝向 → 0°，清积分；目标角保留 */
    Heading_ResetOdometer();        /* 本次运行的里程从启动开始算 */
    error_code = CAR_ERROR_NONE;
    state = CAR_HEADING;
    line_lost = 0U;
    last_step_ms = Platform_GetMs();
    return 1U;
}

/* 顶层输入：目标航向角（度），正 = 左转。 */
void CarControl_SetHeading(float degrees) { Heading_SetTarget(degrees); }

void CarControl_SetHeadingRelative(float delta_degrees)
{
    Heading_SetTargetRelative(delta_degrees);
}

/* 顶层输入：目标速度（两轮编码器计数之和 / 10ms）。0 = 原地对准。 */
void CarControl_SetSpeed(int16_t speed_cmd) { Heading_SetSpeed(speed_cmd); }

float   CarControl_GetHeading(void)       { return Heading_GetYaw(); }
float   CarControl_GetHeadingTarget(void) { return Heading_GetTarget(); }
int16_t CarControl_GetEncoderLeft(void)   { return Heading_GetEncoderLeft(); }
int16_t CarControl_GetEncoderRight(void)  { return Heading_GetEncoderRight(); }
int32_t CarControl_GetOdometer(void)
{
    return Heading_GetOdometerLeft() + Heading_GetOdometerRight();
}

/* 最近一次下发给航向环的转向命令；三种模式共用一个出口，便于在线观察调参。 */
void CarControl_GetDrive(DriveCmd *out)
{
    if (out != 0) { *out = last_drive; }
}

/* 执行一轮整车协调；内部限制为每10毫秒一步，未到时间立即返回。 */
void CarControl_Update(void)

{
    uint32_t now = Platform_GetMs();
    TrackStatus line;
    AvoidCheck obstacle;
    AvoidStatus phase;
    DriveCmd cmd;
    /* 未到周期不重复控制；无符号差值可处理正常范围内的计数回绕。 */
    if ((uint32_t)(now - last_step_ms) < CAR_CONTROL_PERIOD_MS) return;
    last_step_ms = now; /* do not replay missed control steps on stale data */

    /* 终点或故障后持续停车，不再调用运动模块，防止重新驱动车轮。 */
    if (state == CAR_FINISHED || state == CAR_FAULT) {
        Motor_Stop();
        return;
    }

#if CAR_ODOMETER_STOP_COUNTS > 0
    /* ★ 里程到点自动停车：本次运行累计里程（左右轮计数之和）达到阈值就锁存
     *   CAR_FINISHED，三种运行模式都适用。阈值见 App/CarConfig.h。 */
    if (state != CAR_IDLE &&
        CarControl_GetOdometer() >= (int32_t)CAR_ODOMETER_STOP_COUNTS) {
        Motor_Stop();
        error_code = CAR_ERROR_NONE;
        state = CAR_FINISHED;
        return;
    }
#endif

    /* ★ 航向模式：独立于循迹和避障，只跑航向环 + 速度环。
     *
     *   必须放在下面 AVOID_CHECK_INVALID 判断之前：航向模式不依赖测距，
     *   如果放到后面就会白白继承超声波依赖，没接超声波时一进航向模式
     *   就直接报 CAR_ERROR_RANGE。
     *
     *   仍然调用 Avoid_SenseUpdate() 是为了让超声波继续刷新测距缓存
     *   （它只读距离、不写电机），顶层需要时可以读 Ultrasound_* 做前方避让。*/
    if (state == CAR_HEADING) {
        if (Heading_HasFault() != 0U) {
            Fail(CAR_ERROR_IMU);
            return;
        }
        Avoid_SenseUpdate();        /* 保持测距新鲜，不控制电机 */
        Heading_Update();
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
        if (Heading_HasFault() != 0U) { Fail(CAR_ERROR_IMU); return; }
        /* Wide black is usable. 丢线、分离黑簇或数据无效时不再立刻停车：
         * Track 会沿最后一次的修正方向继续转向找线（车最后偏右就继续左转，
         * 最后偏左就继续右转），1 秒内找到线就立刻回到正常循迹。
         * 这里只负责转发它的搜索命令，并在容忍窗口耗尽后锁存 CAR_ERROR_TRACK；
         * Track 给不出命令（数据无效、从来没有过非零偏差、搜索超时或搜索被
         * 关闭）时目标速度为 0，此时才真正停车。 */
        if (line.state != TRACK_NORMAL && line.state != TRACK_WIDE) {
            if (!line_lost) {
                line_lost = 1U;
                line_lost_ms = now;
            } else if ((uint32_t)(now - line_lost_ms) >= CAR_TRACK_LOSS_TOLERANCE_MS) {
                Fail(CAR_ERROR_TRACK);
                return;
            }
            Track_Update();
            Track_GetDrive(&cmd);
            if (cmd.speed_cmd == 0) {
                Heading_Stop(); /* 搜索不可用：停转并清速度环积分；航向和目标保留 */
                return;
            }
            ApplyDrive(&cmd);
            return;
        }
        line_lost = 0U;
        /* 先撤销循迹控制并停车，再启动避障；同一轮不会同时执行两种控制。 */
        if (obstacle == AVOID_CHECK_DETECTED) {
            Heading_Stop();
            Track_Reset();
            if (!Avoid_Start()) {
                Fail(CAR_ERROR_AVOID_START);
                return;
            }
            avoidance_started_ms = now;
            state = CAR_AVOIDING;
            return;
        }
        /* ★ 循迹只算出「目标航向角 + 目标速度」，车轮差速由航向环产生。 */
        Track_Update();
        Track_GetDrive(&cmd);
        ApplyDrive(&cmd);
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
    if (Heading_HasFault() != 0U) { Fail(CAR_ERROR_IMU); return; }
    /* 传入本轮线状态的地址；避障模块可以读取它执行一步动作。 */
    Avoid_Update(&line);
    phase = Avoid_GetStatus();
    /* 本轮更新可能刚刚完成避障，立即停车交接。 */
    if (phase == AVOID_DONE) {
        CompleteAvoidance();
        return;
    }
    /* 避障运行阶段报告失败或意外空闲，都视为异常。 */
    if (phase == AVOID_FAILED || phase == AVOID_IDLE) {
        Fail(CAR_ERROR_AVOID_FAILED);
        return;
    }
    /* ★ 避障同样只给出「目标航向角 + 目标速度」，差速由航向环产生。 */
    Avoid_GetDrive(&cmd);
    ApplyDrive(&cmd);
}

/* 读取整车当前状态，不改变状态，也不控制电机。 */
CarState CarControl_GetState(void) { return state; }
/* 读取故障原因，用于调试定位；无故障时返回CAR_ERROR_NONE。 */
CarError CarControl_GetError(void) { return error_code; }
