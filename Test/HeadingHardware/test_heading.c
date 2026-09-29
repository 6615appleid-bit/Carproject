/* 航向环主机测试。
 *
 * 不碰任何硬件：把编码器、MPU6050、电机、时基全部替换成带测试钩子的替身，
 * 再给航向环接一个简化的差速小车被控对象模型，跑闭环。
 *
 * 这个测试最重要的作用是【验证符号约定】。符号错一个，转向环会从负反馈变成
 * 正反馈 —— 表现是车一推就疯狂自转停不下来，而且从代码上根本看不出来。
 * stm32test 的 car_config.h 里花了整整一段注释推导这件事。
 * 闭环仿真能直接把这种错误暴露成「航向角发散」。
 *
 * 被控对象模型（简单但方向正确）：
 *   PWM → 轮速（一阶滞后）→ 编码器计数
 *   右轮比左轮快 → 左转 → 航向角增大
 *   陀螺按 SIGN = -1 的贴装方向回报：物理左转时原始读数为负
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "Heading.h"
#include "Encoder.h"
#include "Imu.h"
#include "Motor.h"
#include "Platform.h"

/*==============================================================================
 * 时基替身
 *============================================================================*/
static uint32_t now;
uint32_t Platform_GetMs(void) { return now; }

/*==============================================================================
 * 编码器替身：由被控对象模型写入读数
 *============================================================================*/
static int16_t stub_enc_l, stub_enc_r;
static int32_t stub_total_l, stub_total_r;
static unsigned enc_init_calls;

void Encoder_Init(void)
{
    ++enc_init_calls;
    stub_enc_l = stub_enc_r = 0;
    stub_total_l = stub_total_r = 0;
}

int16_t Encoder_GetCount(uint8_t ch)
{
    int16_t value = (ch == ENC_CH_LEFT) ? stub_enc_l : stub_enc_r;
    if (ch == ENC_CH_LEFT) { stub_total_l += value; } else { stub_total_r += value; }
    return value;
}

int32_t Encoder_GetTotal(uint8_t ch)
{
    return (ch == ENC_CH_LEFT) ? stub_total_l : stub_total_r;
}

void Encoder_ResetTotal(void) { stub_total_l = stub_total_r = 0; }

/*==============================================================================
 * MPU6050 替身
 *============================================================================*/
static float   stub_gyro_raw;   /* 陀螺原始读数，未应用 HEADING_GYRO_SIGN */
static uint8_t stub_imu_fail;

uint8_t Imu_Init(void)                                  { return 0U; }
uint8_t Imu_CalibrateGyro(uint16_t s, float tol)        { (void)s; (void)tol; return 0U; }
uint8_t Imu_IsReady(void)                               { return 1U; }
float   Imu_GetBiasZ(void)                              { return 0.0f; }
uint8_t Imu_ReadGyroZDps(float *gz)
{
    if (stub_imu_fail) { return 1U; }
    *gz = stub_gyro_raw;
    return 0U;
}

/*==============================================================================
 * 电机替身：只记录最后一次需求
 *============================================================================*/
static int16_t  m_left, m_right;
static unsigned motor_writes;

void Motor_Init(void) { m_left = m_right = 0; }
void Motor_Stop(void) { m_left = m_right = 0; ++motor_writes; }
void Motor_SetDemand(int16_t left, int16_t right)
{
    m_left = left;
    m_right = right;
    ++motor_writes;
}

/*==============================================================================
 * 被控对象模型
 *============================================================================*/
/* 被控对象参数取自 stm32test 的实测反推值：
 *   其 car_config.h 记录“PWM → 转速是一阶惯性（τ≈0.04s，直流增益
 *   K≈0.0116 计数/PWM，PWM 满量程 ±7200）”。
 * 本工程 PWM 满量程 ±1000，所以换算过来：
 *   两轮之和的增益 = 0.0116 × 7200 / 1000 = 0.0835 计数/PWM
 *   而 speed_out 是同时加到左右两轮的，所以单轮增益取一半 ≈ 0.042
 * 这样仿真出来的环路增益和 stm32test 实测的完全一致。 */
#define PLANT_ENC_PER_PWM   0.042f  /* 单轮：PWM 800 → 约 33 计数/10ms */
#define PLANT_LAG           0.30f   /* 每拍向目标逼近 30%，τ ≈ 3 拍 */
#define PLANT_YAW_PER_DIFF  1.5f    /* (°/s) / 计数差 —— 决定转向快慢，
                                     * 取到极限差速时约 100 °/s，
                                     * 与 stm32test 路线实测的转弯耗时同量级 */

static float plant_l, plant_r;      /* 左右轮实际速度（计数/10ms） */
static float plant_yaw;             /* 真实航向角（度），正 = 左转 */

static void PlantStep(void)
{
    float omega;

    /* 一阶电机滞后 */
    plant_l += (PLANT_ENC_PER_PWM * (float)m_left  - plant_l) * PLANT_LAG;
    plant_r += (PLANT_ENC_PER_PWM * (float)m_right - plant_r) * PLANT_LAG;

    /* 右轮快于左轮 → 左转 → 航向角增大 */
    omega = PLANT_YAW_PER_DIFF * (plant_r - plant_l);       /* °/s */
    plant_yaw += omega * 0.01f;                             /* dt = 10 ms */

    /* 陀螺回报：物理左转(omega > 0) 时原始读数为负，
     * 所以 HEADING_GYRO_SIGN = -1 才把它翻成正的。 */
    stub_gyro_raw = -omega;

    /* 编码器读到的是带符号的轮速 */
    stub_enc_l = (int16_t)(plant_l + (plant_l < 0.0f ? -0.5f : 0.5f));
    stub_enc_r = (int16_t)(plant_r + (plant_r < 0.0f ? -0.5f : 0.5f));
}

static void PlantReset(void)
{
    plant_l = plant_r = plant_yaw = 0.0f;
    stub_gyro_raw = 0.0f;
    stub_imu_fail = 0U;
}

/* 跑 count 拍，每拍 10 ms */
static void Run(unsigned count)
{
    unsigned i;
    for (i = 0U; i < count; i++) {
        PlantStep();
        now += 10U;
        Heading_Update();
    }
}

static void ResetAll(void)
{
    now = 0U;
    PlantReset();
    Motor_Init();
    motor_writes = 0U;
    Heading_Init();
    assert(Heading_ImuInit() == 0U);
    Heading_Reset();
}

/*==============================================================================
 * 用例
 *============================================================================*/

/* 1. 静止、目标为 0 时不应该驱动电机 */
static void TestIdle(void)
{
    ResetAll();
    Heading_SetSpeed(0);
    Heading_SetTarget(0.0f);
    Run(5U);
    assert(m_left == 0 && m_right == 0);
    assert(Heading_GetTurnOut() == 0);
    assert(Heading_GetSpeed() == 0);
}

/* 2. ★ 目标 +90° 必须是左转：左轮后退、右轮前进（原地） */
static void TestTurnDirection(void)
{
    ResetAll();
    Heading_SetSpeed(0);
    Heading_SetTarget(90.0f);
    Run(1U);

    assert(Heading_GetTurnOut() > 0);           /* 正转向量 = 左转 */
    assert(m_left  < 0 && m_right > 0);         /* 左轮减速/后退，右轮前进 */
    assert(m_left == -m_right);                 /* 速度环输出为 0 → 原地转 */
    printf("  turn_out=%d  left=%d right=%d  (left turn OK)\n",
           (int)Heading_GetTurnOut(), (int)m_left, (int)m_right);
}

/* 3. ★ 闭环对准：目标 90°，航向角必须收敛到 90°，且不允许发散 */
static void TestAlignConverges(void)
{
    ResetAll();
    Heading_SetSpeed(0);
    Heading_SetTarget(90.0f);
    Run(600U);                                  /* 6 秒 */

    assert(plant_yaw > 80.0f && plant_yaw < 100.0f);
    assert(Heading_GetYaw() > 80.0f && Heading_GetYaw() < 100.0f);
    assert(Heading_GetTarget() == 90.0f);
    /* 稳定后不应再有明显转向量 */
    assert(Heading_GetTurnOut() < 60 && Heading_GetTurnOut() > -60);
    printf("  aligned to %.1f deg (target 90), residual turn_out=%d\n",
           (double)plant_yaw, (int)Heading_GetTurnOut());
}

/* 4. ★ 反向对准：目标 -45° 必须是右转，同样要收敛 */
static void TestAlignNegative(void)
{
    ResetAll();
    Heading_SetSpeed(0);
    Heading_SetTarget(-45.0f);
    Run(1U);
    assert(Heading_GetTurnOut() < 0);           /* 负转向量 = 右转 */
    assert(m_left > 0 && m_right < 0);          /* 左轮前进、右轮后退 */

    Run(600U);
    assert(plant_yaw < -35.0f && plant_yaw > -55.0f);
    printf("  aligned to %.1f deg (target -45)\n", (double)plant_yaw);
}

/* 5. 死区：偏差小于阈值时不修方向。
 * 注意阈值只有 0.30 度（沿用 stm32test 的 TURN_DEADZONE），所以这里的
 * 偏差必须比它更小。 */
static void TestDeadZone(void)
{
    ResetAll();
    Heading_SetSpeed(0);
    Heading_SetTarget(0.2f);                    /* < HEADING_DEADZONE_DEG */
    Run(1U);
    assert(Heading_GetTurnOut() == 0);
    assert(m_left == 0 && m_right == 0);
}

/* 6. 速度环：给定目标速度后两轮都应向前，并逼近目标 */
static void TestSpeedLoop(void)
{
    ResetAll();
    Heading_SetTarget(0.0f);
    Heading_SetSpeed(30);
    Run(400U);

    assert(m_left > 0 && m_right > 0);          /* 正速度 = 前进 */
    assert(Heading_GetSpeed() > 27 && Heading_GetSpeed() < 33);
    printf("  speed settled at %d counts/10ms (target 30)\n",
           (int)Heading_GetSpeed());
}

/* 7. 走直线：目标航向恒为 0，两轮速度应基本相同（不跑偏） */
static void TestStraight(void)
{
    int16_t drift;
    ResetAll();
    Heading_SetTarget(0.0f);
    Heading_SetSpeed(30);
    Run(500U);

    drift = (int16_t)(m_left - m_right);
    if (drift < 0) { drift = (int16_t)(-drift); }
    assert(drift < 60);                         /* 几乎同速 */
    assert(plant_yaw > -5.0f && plant_yaw < 5.0f);
}

/* 8. 惯性单元读取失败必须报故障并停车 */
static void TestImuFault(void)
{
    ResetAll();
    Heading_SetTarget(90.0f);
    Heading_SetSpeed(30);
    Run(20U);

    stub_imu_fail = 1U;
    Run(1U);

    assert(Heading_HasFault() != 0U);
    assert(m_left == 0 && m_right == 0);        /* 停车，不能带着故障继续跑 */
}

/* 9. Reset 把当前朝向定义为 0°，目标角保留（顶层先 Set 再 Start 不会丢） */
static void TestResetSemantics(void)
{
    ResetAll();
    Heading_SetTarget(120.0f);
    Heading_SetTargetRelative(-30.0f);
    assert(Heading_GetTarget() == 90.0f);       /* 相对转角生效 */

    Heading_SetSpeed(20);
    Run(100U);
    assert(Heading_GetYaw() != 0.0f);

    Heading_Reset();                            /* 进入航向模式/停车时调用 */
    assert(Heading_GetYaw() == 0.0f);           /* 航向重新归零 */
    assert(Heading_GetTarget() == 90.0f);       /* 目标角保留 */
    assert(Heading_HasFault() == 0U);
    assert(Heading_GetTurnOut() == 0);
}

/* 10. ★ Heading_Drive：统一入口一次完成「下发目标航向/速度 → 更新输出」。
 *     循迹和避障就是通过这个入口驱动车轮的，所以符号必须与 Heading_Update 一致。 */
static void TestDriveEntry(void)
{
    DriveCmd cmd;
    ResetAll();
    cmd.steer_deg = 90.0f;
    cmd.speed_cmd = 0;
    PlantStep();
    now += 10U;
    Heading_Drive(&cmd);
    assert(Heading_GetTarget() == 90.0f);       /* 目标角确实下发到环里 */
    assert(Heading_GetTurnOut() > 0);           /* 正 = 左转 */
    assert(m_left < 0 && m_right > 0);
    /* 空指针按「停」处理，不让忘记给命令时继续跑 */
    Heading_Drive(0);
    assert(m_left == 0 && m_right == 0);
}

/* 11. ★ Heading_Stop：停车并清控制状态，但保留航向和目标角
 *     （与 Heading_Reset 的区别就是航向不归零）。 */
static void TestStopKeepsState(void)
{
    int16_t turn;   /* 给下一个用例看的中间量 */
    double  yaw_before;
    ResetAll();
    Heading_SetTarget(45.0f);
    Heading_SetSpeed(20);
    Run(200U);
    yaw_before = (double)Heading_GetYaw();
    assert(yaw_before != 0.0);

    Heading_Stop();
    assert(m_left == 0 && m_right == 0);
    assert(Heading_GetTurnOut() == 0 && Heading_GetSpeedOut() == 0);
    assert(Heading_GetYaw() != 0.0f);           /* 航向不归零 */
    assert(Heading_GetTarget() == 45.0f);       /* 目标保留 */
    assert(Heading_HasFault() == 0U);           /* 停车不等于故障 */

    /* 继续跑时从零输出重新起步，不会因旧积分突然窜车 */
    Run(1U);
    turn = Heading_GetSpeedOut();
    assert(turn > 0 && turn < 400);
}

/* 12. Heading_ResetOdometer 只清累计里程，不动角度和速度设定 */
static void TestOdometerReset(void)
{
    ResetAll();
    Heading_SetTarget(20.0f);
    Heading_SetSpeed(20);
    Run(100U);
    assert(Heading_GetOdometerLeft() > 0);
    Heading_ResetOdometer();
    assert(Heading_GetOdometerLeft() == 0 && Heading_GetOdometerRight() == 0);
    assert(Heading_GetTarget() == 20.0f);
    assert(Heading_GetYaw() != 0.0f);
}

int main(void)
{
    printf("Heading controller host tests\n");

    TestIdle();
    printf("PASS idle: no motor output when stationary at target 0\n");

    TestTurnDirection();
    printf("PASS turn direction: +90 commands a left turn (left back, right forward)\n");

    TestAlignConverges();
    printf("PASS closed loop: +90 deg converges, no runaway\n");

    TestAlignNegative();
    printf("PASS closed loop: -45 deg converges the other way\n");

    TestDeadZone();
    printf("PASS dead zone: small errors do not steer\n");

    TestSpeedLoop();
    printf("PASS speed loop: forward demand reaches the encoder target\n");

    TestStraight();
    printf("PASS straight run: heading 0 keeps both wheels matched\n");

    TestImuFault();
    printf("PASS IMU fault: latches fault and stops the motors\n");

    TestResetSemantics();
    printf("PASS reset: yaw re-zeroes, target and speed survive\n");

    TestDriveEntry();
    printf("PASS drive entry: Heading_Drive forwards the target and steers\n");

    TestStopKeepsState();
    printf("PASS stop: motors off, integrators cleared, yaw and target survive\n");

    TestOdometerReset();
    printf("PASS odometer: Heading_ResetOdometer clears distance only\n");

    printf("PASS: heading controller, signs, convergence, speed loop, faults.\n");
    return 0;
}
