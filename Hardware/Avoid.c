/* Sensing never moves motors. Manoeuvre control and stable handoff live here.
 *
 * ★ 转向也走统一接口：本模块不再自己拼 PWM 差速，而是每步给出
 *   「目标航向角 + 目标速度」（DriveCmd），差速由 Heading 的转向环产生。
 *   绕障动作因此从「固定时间 + 固定 PWM」变成「转到某个航向角 + 走某段里程」：
 *     序号0 转向空侧：目标航向 = 基准 ∓ AVOID_TURN_DEG，到位判据是航向角
 *     序号1 直行通过：目标航向 = 基准，到位判据是编码器里程
 *     序号2 转回基准：目标航向 = 基准
 *     序号3 寻线：线偏差 → 相对基准的航向修正角，回线时还要求航向接近基准
 *   每一步都保留超时兜底：航向或里程读数异常时不会无限制地走下去。
 */
#include "Avoid.h"
#include "Ultrasound.h"
#include "Heading.h"
#include "Platform.h"

/* Sensing thresholds: unchanged. */
#define AVOID_ENTER_CM 15.0f
#define AVOID_RELEASE_CM 18.0f
#define AVOID_CRITICAL_CM 5.0f

/*--- Manoeuvre geometry / speeds: calibrate on the actual car ----------------*/
#ifndef AVOID_TURN_DEG
#define AVOID_TURN_DEG 45.0f            /* 向空侧让开的角度 */
#endif
#ifndef AVOID_ANGLE_TOL_DEG
#define AVOID_ANGLE_TOL_DEG 5.0f        /* 航向到位判据 */
#endif
#ifndef AVOID_TURN_SPEED
#define AVOID_TURN_SPEED 10             /* 转出时的前进速度（计数/10ms），0 = 原地转 */
#endif
#ifndef AVOID_TURN_TIMEOUT_MS
#define AVOID_TURN_TIMEOUT_MS 2000U
#endif
#ifndef AVOID_PASS_COUNTS
#define AVOID_PASS_COUNTS 900           /* 直行段里程：左右轮编码器计数之和 */
#endif
#ifndef AVOID_PASS_SPEED
#define AVOID_PASS_SPEED 25
#endif
#ifndef AVOID_PASS_TIMEOUT_MS
#define AVOID_PASS_TIMEOUT_MS 4000U
#endif
#ifndef AVOID_RETURN_SPEED
#define AVOID_RETURN_SPEED 12
#endif
#ifndef AVOID_RETURN_TIMEOUT_MS
#define AVOID_RETURN_TIMEOUT_MS 3000U
#endif
#ifndef AVOID_SEARCH_SPEED
#define AVOID_SEARCH_SPEED 18
#endif
#ifndef AVOID_SEARCH_KP_DEG
#define AVOID_SEARCH_KP_DEG 30.0f       /* 每 1000 偏差单位的航向修正角 */
#endif
#ifndef AVOID_SEARCH_STEER_LIMIT_DEG
#define AVOID_SEARCH_STEER_LIMIT_DEG 35.0f
#endif
#ifndef AVOID_RETURN_HEADING_TOL_DEG
#define AVOID_RETURN_HEADING_TOL_DEG 25.0f /* 回线时航向必须还朝基准方向 */
#endif
#define AVOID_SEARCH_TIMEOUT_MS 4000U
#define AVOID_STEP_MS 10U
#define AVOID_STABLE_SAMPLES 5U
#define AVOID_LINE_ERROR_MAX 300

static AvoidStatus phase;
static AvoidCheck obstacle;
static UltrasoundSample ranges;
static uint8_t left_blocked, right_blocked, stable_count, step_valid, bypass_step;
static int8_t away_direction;
static uint32_t phase_ms, last_step_ms;
static uint32_t last_line_id;
static float  base_deg;      /* 进入避障时的航向：绕障前后要对齐的方向 */
static int32_t start_counts; /* 直行段的里程零位 */
static DriveCmd drive;       /* 本模块输出的转向命令，由整车交给 Heading */

static float AbsFloat(float v) { return v < 0.0f ? -v : v; }

/* 两轮累计计数之和：正 = 前进方向走过的里程。 */
static int32_t RunCounts(void)
{
    return Heading_GetOdometerLeft() + Heading_GetOdometerRight();
}

/* 当前航向是否已经到达 target 附近。航向是连续角，不需要解绕。 */
static uint8_t HeadingReached(float target, float tol)
{
    return (uint8_t)(AbsFloat(Heading_GetYaw() - target) <= tol);
}

void Avoid_Reset(void)
{
    phase = AVOID_IDLE;
    stable_count = step_valid = bypass_step = 0U;
    base_deg = 0.0f;
    start_counts = 0;
    drive.steer_deg = 0.0f;
    drive.speed_cmd = 0;
    last_line_id = Track_GetSampleId();
    /* Reset control, preserving fresh acquisition and hysteresis. */
}

void Avoid_Init(void)
{
    Ultrasound_Init();
    obstacle = AVOID_CHECK_INVALID;
    left_blocked = right_blocked = 0U;
    ranges.valid = 0U;
    Avoid_Reset();
}

void Avoid_SenseUpdate(void)
{
    Ultrasound_Update();
    Ultrasound_GetSample(&ranges);
    if (!ranges.valid) { obstacle = AVOID_CHECK_INVALID; return; }
    if (ranges.left_cm < AVOID_ENTER_CM) left_blocked = 1U;
    else if (ranges.left_cm >= AVOID_RELEASE_CM) left_blocked = 0U;
    if (ranges.right_cm < AVOID_ENTER_CM) right_blocked = 1U;
    else if (ranges.right_cm >= AVOID_RELEASE_CM) right_blocked = 0U;
    obstacle = left_blocked || right_blocked ? AVOID_CHECK_DETECTED : AVOID_CHECK_CLEAR;
}

AvoidCheck Avoid_Check(void)
{
    UltrasoundSample current;
    Ultrasound_GetSample(&current);
    return current.valid ? obstacle : AVOID_CHECK_INVALID;
}

uint8_t Avoid_Start(void)
{
    if (phase != AVOID_IDLE || Avoid_Check() != AVOID_CHECK_DETECTED) return 0U;
    away_direction = left_blocked && !right_blocked ? 1 : -1;
    if (left_blocked && right_blocked)
        away_direction = ranges.right_cm >= ranges.left_cm ? 1 : -1;
    /* 当前航向就是绕障全程要对齐的基准方向。 */
    base_deg = Heading_GetYaw();
    start_counts = RunCounts();
    drive.steer_deg = base_deg;
    drive.speed_cmd = 0;
    phase = AVOID_BYPASSING;
    bypass_step = stable_count = step_valid = 0U;
    phase_ms = Platform_GetMs();
    return 1U;
}

static void Fail(void) { phase = AVOID_FAILED; }

void Avoid_Update(const TrackStatus *line)
{
    uint32_t now = Platform_GetMs();
    uint32_t elapsed;
    if (phase != AVOID_BYPASSING && phase != AVOID_SEARCHING) return;
    if (!line || Avoid_Check() == AVOID_CHECK_INVALID) { Fail(); return; }
    if (ranges.left_cm < AVOID_CRITICAL_CM || ranges.right_cm < AVOID_CRITICAL_CM) { Fail(); return; }
    if (step_valid && (uint32_t)(now - last_step_ms) < AVOID_STEP_MS) return;
    if (step_valid && (uint32_t)(now - last_step_ms) > 2U * AVOID_STEP_MS) stable_count = 0U;
    last_step_ms = now;
    step_valid = 1U;
    elapsed = now - phase_ms;

    if (phase == AVOID_BYPASSING) {
        /* Advance at most one step: do not replay missed actions. */
        if (bypass_step == 0U) {
            /* 转出：目标航向 = 基准 ∓ AVOID_TURN_DEG，到位判据是航向角。
             * away_direction > 0 表示「向右让」（左侧有障碍），而航向角正方向
             * 是左转，所以要取负号。 */
            drive.steer_deg = base_deg - (float)away_direction * AVOID_TURN_DEG;
            drive.speed_cmd = AVOID_TURN_SPEED;
            if (HeadingReached(drive.steer_deg, AVOID_ANGLE_TOL_DEG) ||
                elapsed >= AVOID_TURN_TIMEOUT_MS) {
                bypass_step = 1U; phase_ms = now; start_counts = RunCounts();
            }
            return;
        }
        if (bypass_step == 1U) {
            /* 直行通过：保持基准航向，到位判据是编码器里程。 */
            drive.steer_deg = base_deg;
            drive.speed_cmd = AVOID_PASS_SPEED;
            if ((RunCounts() - start_counts) >= AVOID_PASS_COUNTS ||
                elapsed >= AVOID_PASS_TIMEOUT_MS) {
                bypass_step = 2U; phase_ms = now;
            }
            return;
        }
        /* 转回基准方向，再交给寻线。 */
        drive.steer_deg = base_deg;
        drive.speed_cmd = AVOID_RETURN_SPEED;
        if (HeadingReached(base_deg, AVOID_ANGLE_TOL_DEG) ||
            elapsed >= AVOID_RETURN_TIMEOUT_MS) {
            phase = AVOID_SEARCHING; phase_ms = now; stable_count = 0U;
            last_line_id = Track_GetSampleId();
            drive.speed_cmd = 0;
        }
        return;
    }

    if (elapsed >= AVOID_SEARCH_TIMEOUT_MS || line->state == TRACK_INVALID) { Fail(); return; }
    /* 寻线：线偏差 → 相对基准的航向修正角；没看到线就直着朝基准方向找。 */
    drive.steer_deg = base_deg;
    if (line->state == TRACK_NORMAL) {
        float steer = -(AVOID_SEARCH_KP_DEG * (float)line->error) / 1000.0f;
        if (steer > AVOID_SEARCH_STEER_LIMIT_DEG) steer = AVOID_SEARCH_STEER_LIMIT_DEG;
        if (steer < -AVOID_SEARCH_STEER_LIMIT_DEG) steer = -AVOID_SEARCH_STEER_LIMIT_DEG;
        drive.steer_deg = base_deg + steer;
    }
    drive.speed_cmd = AVOID_SEARCH_SPEED;
    /* UART frames may arrive more slowly than the 10 ms control period.
     * Count each consumed frame only once, and never reuse bypass samples.
     * 稳定回线还要判航向：线居中、偏差不大、障碍已解除，并且车头已经回到基准
     * 方向附近 —— 否则找回的可能是另一条线的走向，规则里逆行会直接结束比赛。 */
    if (line->state == TRACK_NORMAL && line->error >= -AVOID_LINE_ERROR_MAX &&
        line->error <= AVOID_LINE_ERROR_MAX && Avoid_Check() == AVOID_CHECK_CLEAR &&
        HeadingReached(base_deg, AVOID_RETURN_HEADING_TOL_DEG)) {
        uint32_t line_id = Track_GetSampleId();
        if (line_id != last_line_id) {
            last_line_id = line_id;
            if (++stable_count >= AVOID_STABLE_SAMPLES) {
                drive.speed_cmd = 0;
                phase = AVOID_DONE;
                return;
            }
        }
    } else { stable_count = 0U; last_line_id = Track_GetSampleId(); }
}

AvoidStatus Avoid_GetStatus(void) { return phase; }

/* 读取本模块最新算出的转向命令；整车每步把它交给 Heading_Drive。 */
void Avoid_GetDrive(DriveCmd *out)
{
    if (out != 0) { *out = drive; }
}
