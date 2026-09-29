/* 航向对准 + 速度闭环本体。
 *
 * 顶层的用法只有两步：
 *     CarControl_StartHeading();          // 把当前朝向定义为 0°
 *     CarControl_SetHeading(90.0f);       // 左转 90 度后保持
 *     CarControl_SetSpeed(0);             // 0 = 原地转
 *
 * ★ 本模块同时是全车唯一的驱动出口：循迹和避障不再自己拼 PWM，而是各自给出
 *   「目标航向角 + 目标速度」，由 Heading_Drive() 转成左右轮差速。因此
 *   Motor_SetDemand 在本工程里只应该出现于本文件（和 Motor_Stop 的停车路径）。
 *
 * 与 stm32test 的实现差异，全部是环境原因，算法一致：
 *   1. 控制环挂在主循环的 10 ms 调度上，不在 MPU 中断里，
 *      因此 dt 用 Platform_GetMs() 实测值并做限幅。
 *      stm32test 因为本板 DWT->CYCCNT 会偶发「倒退」，反而只能用固定 dt；
 *      这里没有 DWT 依赖，实测 dt 更准。
 *   2. 编码器用标准外设库（TIM_GetCounter）而不是 HAL 句柄。
 *   3. 陀螺每个周期只读 Z 轴 2 个字节，省 I2C 时间。
 */
#include "Heading.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Platform.h"

/*------------------------------------------------------------------------------
 * 内部状态
 *--------------------------------------------------------------------------*/
static float    yaw_deg;            /* 连续航向角，正 = 左转            */
static float    target_deg;         /* 顶层下发的目标航向角             */
static int16_t  speed_cmd;          /* 目标速度（两轮计数之和 / 10ms）  */
static int16_t  speed_out;          /* 速度环输出 PWM                   */
static int16_t  turn_out;           /* 转向环输出 PWM                   */
static int16_t  enc_l, enc_r;       /* 本周期左右轮增量（已带符号）     */
static float    gyro_z;             /* 已应用符号的 Z 轴角速度 (°/s)    */
static float    vel_err_lpf;        /* 速度偏差一阶低通状态             */
static float    vel_integral;       /* 速度环积分累加器                 */
static uint32_t last_ms;            /* 上一拍的时间戳                   */
static uint8_t  imu_ok;
static uint8_t  fault;

static float LimitF(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

static int16_t LimitI(int32_t v, int32_t lim)
{
    if (v >  lim) { return (int16_t) lim; }
    if (v < -lim) { return (int16_t)-lim; }
    return (int16_t)v;
}

/*------------------------------------------------------------------------------
 * 初始化 / 复位
 *--------------------------------------------------------------------------*/

void Heading_Init(void)
{
    Encoder_Init();

    /* 沿用 Imu_IsReady() 而不是直接清 0：CarControl_Init() 可以在运行中
     * 被再次调用（调试命令 4），那时 MPU6050 早就初始化好了，
     * 清掉 imu_ok 会让航向模式白白不可用，还得重新标定一次。 */
    imu_ok = Imu_IsReady();

    target_deg = 0.0f;
    speed_cmd  = 0;
    Heading_Reset();
}

uint8_t Heading_ImuInit(void)
{
    imu_ok = (Imu_Init() == 0U) ? 1U : 0U;
    return (uint8_t)(imu_ok ? 0U : 1U);
}

uint8_t Heading_CalibrateGyro(void)
{
    /* 标定必须静止，先确保电机没在转 */
    Motor_Stop();
    return Imu_CalibrateGyro(200U, 3.0f);
}

uint8_t Heading_IsImuReady(void)
{
    return imu_ok;
}

void Heading_Reset(void)
{
    /* 上电先把两个计数器清干净，避免残留计数被算进第一拍 */
    (void)Encoder_GetCount(ENC_CH_LEFT);
    (void)Encoder_GetCount(ENC_CH_RIGHT);

    yaw_deg      = 0.0f;        /* 「当前朝向」重新定义为 0° */
    gyro_z       = 0.0f;
    enc_l        = 0;
    enc_r        = 0;
    speed_out    = 0;
    turn_out     = 0;
    vel_err_lpf  = 0.0f;
    vel_integral = 0.0f;
    fault        = 0U;
    last_ms      = Platform_GetMs();

    /* target_deg 和 speed_cmd 故意不清：顶层先 Set 再 Start 也不会丢。
     * 安全性由「速度默认 0」保证 —— 不显式给速度就不会自己跑起来。 */
}

/*------------------------------------------------------------------------------
 * 顶层接口
 *--------------------------------------------------------------------------*/

void Heading_SetTarget(float deg)
{
    target_deg = deg;
}

void Heading_SetTargetRelative(float delta)
{
    target_deg += delta;
}

float Heading_GetTarget(void) { return target_deg; }
float Heading_GetYaw(void)    { return yaw_deg; }

void Heading_SetSpeed(int16_t speed)
{
    if (speed >  HEADING_SPEED_CMD_MAX) { speed =  HEADING_SPEED_CMD_MAX; }
    if (speed < -HEADING_SPEED_CMD_MAX) { speed = -HEADING_SPEED_CMD_MAX; }
    speed_cmd = speed;
}

int16_t Heading_GetSpeed(void)      { return (int16_t)(enc_l + enc_r); }
uint8_t Heading_HasFault(void)      { return fault; }

int16_t Heading_GetEncoderLeft(void)   { return enc_l; }
int16_t Heading_GetEncoderRight(void)  { return enc_r; }
int32_t Heading_GetOdometerLeft(void)  { return Encoder_GetTotal(ENC_CH_LEFT); }
int32_t Heading_GetOdometerRight(void) { return Encoder_GetTotal(ENC_CH_RIGHT); }
int16_t Heading_GetSpeedOut(void)      { return speed_out; }
int16_t Heading_GetTurnOut(void)       { return turn_out; }
float   Heading_GetGyroZ(void)         { return gyro_z; }

/*------------------------------------------------------------------------------
 * 统一转向入口 / 停车 / 里程清零
 *--------------------------------------------------------------------------*/

/* 一步运行：设定目标航向角和目标速度，然后按当前 dt 更新一次输出。
 * 这是全车唯一写电机的入口 —— 循迹和避障都只产出 DriveCmd，不再直接写 PWM。 */
void Heading_Drive(const DriveCmd *cmd)
{
    if (cmd == 0) {
        /* 空指针当作「停」处理，不让上层忘记给命令时继续跑 */
        Heading_Stop();
        return;
    }
    Heading_SetTarget(cmd->steer_deg);
    Heading_SetSpeed(cmd->speed_cmd);
    Heading_Update();
}

/* 立即停转并清控制状态；保留 yaw / 目标角 / 目标速度 / 故障标志。
 * 同时读走编码器计数器，避免停车期间累积的计数算进恢复后的第一拍。 */
void Heading_Stop(void)
{
    Motor_Stop();
    enc_l        = 0;
    enc_r        = 0;
    speed_out    = 0;
    turn_out     = 0;
    vel_err_lpf  = 0.0f;
    vel_integral = 0.0f;
    last_ms      = Platform_GetMs();
    (void)Encoder_GetCount(ENC_CH_LEFT);
    (void)Encoder_GetCount(ENC_CH_RIGHT);
}

/* 只清累计里程，不动角度和速度设定（用于「本次运行路程」重新计数）。 */
void Heading_ResetOdometer(void)
{
    Encoder_ResetTotal();
}

/*------------------------------------------------------------------------------
 * 控制环本体 —— 每 10 ms 一次（由 CarControl_Update 调度）
 *--------------------------------------------------------------------------*/
void Heading_Update(void)
{
    uint32_t now;
    float    dt;
    float    err;
    float    out;
    int32_t  pwm_l, pwm_r;

    if (imu_ok == 0U) {
        fault = 1U;
        Motor_Stop();
        return;
    }

    now = Platform_GetMs();
    dt  = (float)(uint32_t)(now - last_ms) * 0.001f;
    last_ms = now;

    /* dt 限幅：下限防止间隔过小时微分项放大噪声，
     * 上限防止某拍被拖延时积分和航向积分被一次拉飞。 */
    if (dt < 0.005f) { dt = 0.005f; }
    if (dt > 0.030f) { dt = 0.030f; }

    /*--- 1. 读编码器（读走本周期增量并清零）--- */
    enc_l = Encoder_GetCount(ENC_CH_LEFT);
    enc_r = Encoder_GetCount(ENC_CH_RIGHT);

    /*--- 2. 读陀螺 Z 轴 + 航向积分 --- */
    if (Imu_ReadGyroZDps(&gyro_z) != 0U) {
        /* 读不到惯性单元就别用脏数据算，直接停车并报故障 */
        fault = 1U;
        Motor_Stop();
        return;
    }
    gyro_z *= (float)HEADING_GYRO_SIGN;     /* 统一成「正 = 左转」 */

    /* 连续角度累加：不做 ±180° 解绕，因此跨圈也不会跳变。
     * 长时间跑担心溢出的场合调 Heading_Reset() 归零即可。 */
    yaw_deg += gyro_z * dt;

    /*--- 3. 速度环：PI + 一阶低通 ---
     * 偏差取 (Target - Speed)：本工程约定「正 = 前进」，
     * 正偏差直接产生正 PWM，不用像原程序那样绕符号。 */
    err = (float)speed_cmd - (float)(enc_l + enc_r);
    vel_err_lpf  = (1.0f - VEL_LPF_A) * err + VEL_LPF_A * vel_err_lpf;
    vel_integral = LimitF(vel_integral + vel_err_lpf, VEL_I_LIMIT);
    out = VEL_KP * vel_err_lpf + VEL_KI * vel_integral;
    speed_out = LimitI((int32_t)out, (int32_t)VEL_OUT_LIMIT);

    /*--- 4. 转向环：航向角 P + 角速度 D ---
     * P 项消除航向偏差；D 项用陀螺角速度做阻尼 —— 负号是因为阻尼要反抗
     * 当前的旋转方向：正在左转(gz > 0)时就往右压一点，防止冲过头摆头。 */
    err = target_deg - yaw_deg;
    if (err < HEADING_DEADZONE_DEG && err > -HEADING_DEADZONE_DEG) {
        err = 0.0f;                         /* 死区：直行时不来回顾 */
    }
    out = HEADING_KP * err - HEADING_KD * gyro_z;
    turn_out = LimitI((int32_t)out, (int32_t)HEADING_OUT_LIMIT);

    /*--- 5. 差速混控 ---
     *   PWM左 = 速度环输出 - 转向输出
     *   PWM右 = 速度环输出 + 转向输出
     * 正转向量让左轮减速、右轮加速，车就左转。
     * 速度环输出为 0 时就是原地对准。 */
    pwm_l = (int32_t)speed_out - (int32_t)turn_out;
    pwm_r = (int32_t)speed_out + (int32_t)turn_out;

    Motor_SetDemand(LimitI(pwm_l, MOTOR_DEMAND_MAX),
                    LimitI(pwm_r, MOTOR_DEMAND_MAX));
}
