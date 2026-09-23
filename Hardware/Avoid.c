/* Sensing never moves motors. Timed control and stable handoff live here. */
#include "Avoid.h"
#include "Ultrasound.h"
#include "Motor.h"
#include "Platform.h"

/* Provisional settings: calibrate manoeuvre times and demand on the car. */
#define AVOID_ENTER_CM 15.0f
#define AVOID_RELEASE_CM 18.0f
#define AVOID_CRITICAL_CM 5.0f
#define AVOID_TURN_MS 200U
#define AVOID_PASS_MS 450U
#define AVOID_RETURN_MS 400U
#define AVOID_SEARCH_TIMEOUT_MS 4000U
#define AVOID_STEP_MS 10U
#define AVOID_STABLE_SAMPLES 5U
#define AVOID_LINE_ERROR_MAX 300
#define AVOID_FORWARD_DEMAND 260
#define AVOID_TURN_INNER 100
#define AVOID_TURN_OUTER 280
#define AVOID_SEARCH_DEMAND 180

static AvoidStatus phase;
static AvoidCheck obstacle;
static UltrasoundSample ranges;
static uint8_t left_blocked, right_blocked, stable_count, step_valid, bypass_step;
static int8_t away_direction;
static uint32_t phase_ms, last_step_ms;
static uint32_t last_line_id;

void Avoid_Reset(void)
{
    phase = AVOID_IDLE;
    stable_count = step_valid = bypass_step = 0U;
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
    phase = AVOID_BYPASSING;
    bypass_step = stable_count = step_valid = 0U;
    phase_ms = Platform_GetMs();
    return 1U;
}

static void Turn(int8_t direction)
{
    Motor_SetDemand(direction > 0 ? AVOID_TURN_OUTER : AVOID_TURN_INNER,
                    direction > 0 ? AVOID_TURN_INNER : AVOID_TURN_OUTER);
}

static void Fail(void) { Motor_Stop(); phase = AVOID_FAILED; }

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
        if (bypass_step == 0U && elapsed >= AVOID_TURN_MS) {
            bypass_step = 1U; phase_ms = now;
        } else if (bypass_step == 1U && elapsed >= AVOID_PASS_MS) {
            bypass_step = 2U; phase_ms = now;
        } else if (bypass_step == 2U && elapsed >= AVOID_RETURN_MS) {
            phase = AVOID_SEARCHING; phase_ms = now; stable_count = 0U;
            last_line_id = Track_GetSampleId();
            Motor_Stop(); return;
        }
        if (bypass_step == 0U) Turn(away_direction);
        else if (bypass_step == 1U) Motor_SetDemand(AVOID_FORWARD_DEMAND, AVOID_FORWARD_DEMAND);
        else Turn((int8_t)-away_direction);
        return;
    }

    if (elapsed >= AVOID_SEARCH_TIMEOUT_MS || line->state == TRACK_INVALID) { Fail(); return; }
    /* UART frames may arrive more slowly than the 10 ms control period.
     * Count each consumed frame only once, and never reuse bypass samples. */
    if (line->state == TRACK_NORMAL && line->error >= -AVOID_LINE_ERROR_MAX &&
        line->error <= AVOID_LINE_ERROR_MAX && Avoid_Check() == AVOID_CHECK_CLEAR) {
        uint32_t line_id = Track_GetSampleId();
        if (line_id != last_line_id) {
            last_line_id = line_id;
            if (++stable_count >= AVOID_STABLE_SAMPLES) {
                Motor_Stop(); phase = AVOID_DONE; return;
            }
        }
    } else { stable_count = 0U; last_line_id = Track_GetSampleId(); }
    if (line->state == TRACK_NORMAL) {
        int32_t correction = (int32_t)line->error / 5;
        if (correction > 120) correction = 120;
        if (correction < -120) correction = -120;
        Motor_SetDemand((int16_t)(AVOID_SEARCH_DEMAND + correction),
                        (int16_t)(AVOID_SEARCH_DEMAND - correction));
    } else Motor_SetDemand(AVOID_SEARCH_DEMAND, AVOID_SEARCH_DEMAND);
}

AvoidStatus Avoid_GetStatus(void) { return phase; }
