/* 航向对准 + 速度闭环。顶层只给一个目标航向角，小车自己转过去并保持。
 *
 * ★ 本模块同时是全车唯一的驱动出口：循迹、避障、顶层航向模式都只给出
 *   「目标航向角 + 目标速度」，车轮差速一律由这里产生（见 Heading_Drive）。
 *   因此“转向”在任何模式下都只有一条路径，模块之间不会再各自拼 PWM。
 *
 * 结构移植自 stm32test 的 control.c，去掉了直立环：
 *
 *   目标航向角 ┐
 *   陀螺仪 → 航向积分 → 转向环(P + 角速度D) ┐
 *                                            ├→ 差速混控 → 电机
 *   目标速度 ┐                               │
 *   编码器 ────→ 速度环(PI) ────────────────┘
 *
 * 与原程序 Turn() 的结构完全一致：
 *     Turn_out = Turn_Kp*(target_yaw - yaw) - Turn_Kd*gyro_z
 * 两项整体符号相反只是因为老代码的航向约定是反的，本质一样：
 * 比例项修航向偏差，微分项用角速度做阻尼。
 * 原来那段 Yaw 跨 ±180° 解绕的代码在这里变成「连续累加角度」，
 * 从根上避免了跳变，不再需要解绕分支。
 *
 * ── 统一符号约定（与 stm32test 一致）──────────────────────────────
 *   正速度   = 前进
 *   正航向角 = 左转（逆时针，俯视）
 *   正转向量 = 左转
 * ────────────────────────────────────────────────────────────────
 */
#ifndef HEADING_H
#define HEADING_H
#include <stdint.h>

/*==============================================================================
 * 参数默认值
 *
 * 放在这里（而不是 App/CarConfig.h）是沿用 Drivers/Motor.h 的既有惯例：
 * 驱动自带头文件给默认值，用 #ifndef 保护，外部想覆盖就在编译选项里 -D。
 *
 * ⚠️ 关于量纲：下面所有 PWM 量纲的增益都是从 stm32test 的 car_config.h 抄来
 *    再按 PWM 量程等比例缩放的。那边的 TIM1 ARR = 7199，满量程 ±7200，
 *    本工程 Motor_SetDemand 是 ±1000，所以缩放系数 = 1000 / 7200 = 0.1389。
 *
 *    缩放后【环路增益不变】，所以闭环带宽、相位裕度、整定时间都与
 *    那边实测的一样（穿越频率约 1.9 Hz，相位裕度约 94°）。
 *
 *    ⚠️ 千万不要直接照抄 stm32test 的数值（80 / 100 / 7200），
 *       那会让转向量一上来就饱和。
 *
 * ⚠️ 但“缩放后正确”仍然不等于“实车已验证”，最终必须在真车上微调。
 *============================================================================*/

/* 陀螺仪 Z 轴符号：让「左转(逆时针)时读数为正」就填 +1，否则填 -1。
 *
 * stm32test 实测该板必须填 -1：它统一了「正 = 左转」的约定，同时翻转了
 * 转向输出和陀螺输入的符号。如果只翻输出不翻输入，P 项和 D 项会双双变成
 * 【正反馈】—— 车左转 → 陀螺读数为负 → D 项 = -Kd*(-|w|) = 正 → 输出更大
 * 的左转 → 越转越猛。实测症状是「一推就疯狂转圈，且停不下来」。
 *
 * 本车的传感器贴装方向必须按 Imu.h 里的自检步骤实测确认。 */
#ifndef HEADING_GYRO_SIGN
#define HEADING_GYRO_SIGN        (-1)
#endif
#if (HEADING_GYRO_SIGN != 1 && HEADING_GYRO_SIGN != -1)
#error HEADING_GYRO_SIGN must be +1 or -1
#endif

/* 转向环：比例项修航向偏差(PWM/度)，微分项用陀螺角速度做阻尼(PWM/(°/s))。
 *
 * 取值来源：stm32test 的 TURN_KP = 100、TURN_KD = 10（PWM/度、PWM/(°/s)，
 * 满量程 ±7200），本工程换算：
 *     TURN_KP 100 × 0.1389 = 13.9
 *     TURN_KD  10 × 0.1389 =  1.39
 * （那边 TURN_KD 原来写 0.6，但它乘的是陀螺原始读数；
 *   换算到 °/s 单位就是 0.6 × 16.4 LSB/(°/s) ≈ 9.8，所以他们取 10。）
 *
 * 整定时先把 HEADING_KD 置 0，把 HEADING_KP 加到开始摆头，再加 KD 压住。 */
#ifndef HEADING_KP
#define HEADING_KP               13.9f
#endif
#ifndef HEADING_KD
#define HEADING_KD               1.39f
#endif

/* 死区：偏差小于这个角度就不修方向，避免直行时来回抖。
 * 沿用 stm32test 的 TURN_DEADZONE = 0.30 度（它那个值是角度量纲，不随量程缩放）。
 * 如果实车直行时来回抽，就适当调大。 */
#ifndef HEADING_DEADZONE_DEG
#define HEADING_DEADZONE_DEG     0.30f
#endif

/* 转向量限幅，即左右轮的差速幅值。stm32test 用 6000（接近满量程），
 * 并注明“实测转弯时一直顶在饱和”，所以那边才从 3000 提到 6000。
 * 这里取 6000 × 0.1389 ≈ 833，保守一点取 800。
 * ⚠️ 受 Motor 满量程 1000 限制：转向量和大速度同时用时，外侧轮会撞顶。 */
#ifndef HEADING_OUT_LIMIT
#define HEADING_OUT_LIMIT        800.0f
#endif

/* 速度环：PI + 一阶低通（结构同 stm32test velocity_loop）。
 * 偏差取 (Target - Speed)，因为本工程约定「正 = 前进」，
 * 正的速度偏差直接产生正的 PWM，不用像原程序那样绕符号。
 *
 * 取值来源（stm32test 的实测整定值，同样乘 0.1389）：
 *     VELOCITY_KP      80.0 → 11.1
 *     VELOCITY_KI       6.0 →  0.83
 *     VELOCITY_LPF_A    0.7 （无量纲，不缩放）
 *
 * VELOCITY_KP 的取值有实测依据：那边从 KP=200 持续自激振荡（6~12Hz）
 * 反推出来，最后定在 80（相位裕度约 94°）。详细推导见其注释。 */
#ifndef VEL_KP
#define VEL_KP                   11.1f
#endif
#ifndef VEL_KI
#define VEL_KI                   0.83f
#endif
#ifndef VEL_LPF_A
#define VEL_LPF_A                0.70f      /* 一阶低通系数，越大越平滑 */
#endif

/* 速度环输出限幅 = PWM 满量程。
 * 等效于 stm32test 的 VELOCITY_OUT_LIMIT = 7200。 */
#ifndef VEL_OUT_LIMIT
#define VEL_OUT_LIMIT            1000.0f
#endif

/* 积分限幅。这里推导很重要，不要随意改小：
 *
 * 去掉直立环后，被控对象是有限直流增益（不是积分器），所以【稳态 PWM
 * 必须全部由积分项承担】。stm32test 的实测直流增益约 0.0116 计数/PWM
 * （换成我们的量纲是 0.0835 计数/PWM）。
 *
 * 要让积分能扛起整个满量程输出，需要 KI × I_LIMIT ≥ OUT_LIMIT，
 * 即 I_LIMIT = OUT_LIMIT / KI = 1000 / 0.83 ≈ 1200。
 * stm32test 的 1200 正好也是 7200/6 —— 同一个比例。
 *
 * ⚠️ 这个值给太小是隐形坑：表现是“怎么调 KP 都有很大的稳态误差”，
 *    本工程的主机测试（Test/run_heading_tests.ps1）就是靠这个抓出来的。 */
#ifndef VEL_I_LIMIT
#define VEL_I_LIMIT              1200.0f
#endif

/* 目标速度的绝对值上限（两轮编码器计数之和 / 10 ms）。
 * 受输出限幅和直流增益限制，能达到的最大速度约 OUT_LIMIT/(1/K) ≈ 84，
 * 超过这个值只会一直饱和。stm32test 的参考量级也是 20~72。 */
#ifndef HEADING_SPEED_CMD_MAX
#define HEADING_SPEED_CMD_MAX    100
#endif

/*==============================================================================
 * 初始化
 *
 * Heading_Init 只初始化编码器和内部状态（快）；
 * MPU6050 的初始化和零偏标定单独调用，因为标定必须静止且会阻塞，
 * 需要由 main 在合适的时机（上电、车没人碰）明确触发。
 *============================================================================*/
void    Heading_Init(void);

/* 初始化 MPU6050，返回 0 = 成功 */
uint8_t Heading_ImuInit(void);

/* 陀螺零偏标定。★ 车必须静止！阻塞约 2.5 秒。返回 0 = 成功 */
uint8_t Heading_CalibrateGyro(void);

/* 1 = 惯性单元已就绪（Imu_Init 成功过） */
uint8_t Heading_IsImuReady(void);

/*==============================================================================
 * 顶层接口
 *============================================================================*/

/**
  * @brief  设置目标航向角
  * @param  deg 单位度，正 = 左转。转向环会把车拧到该角度并一直保持。
  *             0 表示「沿进入航向模式时的车头朝向直行」。
  */
void    Heading_SetTarget(float deg);

/* 在当前目标角基础上再转 delta 度，正 = 左转 */
void    Heading_SetTargetRelative(float delta);

float   Heading_GetTarget(void);
float   Heading_GetYaw(void);

/* 目标速度：两轮编码器计数之和 / 10 ms。0 = 原地对准（默认）。 */
void    Heading_SetSpeed(int16_t speed_cmd);
int16_t Heading_GetSpeed(void);         /* 当前实际速度（本周期读数之和） */

/* 复位：把「当前朝向」重新定义为 0°，清速度环积分和误差历史、清故障。
 * 目标角和目标速度故意【不清】，这样顶层先 Set 再 Start 也不会丢。 */
void    Heading_Reset(void);

/* 每 10 ms 调用一次。内部按 Platform_GetMs 实测 dt，不要并发调用。 */
void    Heading_Update(void);

/*==============================================================================
 * ★ 统一转向接口：目标航向角 + 目标速度（全车唯一写电机的入口）
 *
 * 任何运动模块（循迹 Track、避障 Avoid、顶层航向模式）都只回答两个问题：
 * 「我要朝哪个航向角走」和「我要走多快」，差速不再由各模块自己拼。
 * 这样既保证同一时刻只有一个模块在驱动车轮，也让每一种转向都经过陀螺闭环。
 *
 *   转向环：目标航向角 - 陀螺积分航向 → P 项 + 角速度 D 项 → turn_out
 *   速度环：目标速度 - 编码器增量     → PI                → speed_out
 *   差速混控：左 = speed_out - turn_out，右 = speed_out + turn_out
 *============================================================================*/
typedef struct {
    /* 目标航向角，度，正 = 左转（俯视逆时针）。
     * 语义与 Heading_SetTarget 相同：0 = 「进入本次运行时的车头朝向」。 */
    float   steer_deg;
    /* 目标速度，左右轮编码器计数之和 / 10 毫秒，正 = 前进。
     * 0 = 原地对准：差速仍然有效，所以还能原地转。 */
    int16_t speed_cmd;
} DriveCmd;

/* 一步运行：下发「目标航向角 + 目标速度」，并按当前 dt 更新一次输出。
 * 这是全车唯一写电机的入口。IMU 读取失败时内部会停车并置 Heading_HasFault，
 * 调用方每步检查 Heading_HasFault() 决定是否转入故障。 */
void    Heading_Drive(const DriveCmd *cmd);

/* 立即停转并清控制状态（速度环积分、低通、输出、编码器残留计数），
 * 但【保留】yaw、目标角、目标速度和故障标志。
 * 用于「先停一下、稍后继续」的场合：避免恢复时带着旧积分突然窜车。
 * 与 Heading_Reset 的区别是它不会把当前朝向重新定义为 0°。 */
void    Heading_Stop(void);

/* 只清累计里程（Encoder_GetTotal），不动角度和速度设定。 */
void    Heading_ResetOdometer(void);

/* 1 = 惯导读取失败，上层应转入故障处理 */
uint8_t Heading_HasFault(void);

/*==============================================================================
 * 只读观测量（编码器变量就在这里）
 *============================================================================*/
int16_t Heading_GetEncoderLeft(void);       /* 左轮本周期计数增量 */
int16_t Heading_GetEncoderRight(void);      /* 右轮本周期计数增量 */
int32_t Heading_GetOdometerLeft(void);      /* 左轮上电以来累计 */
int32_t Heading_GetOdometerRight(void);     /* 右轮上电以来累计 */
int16_t Heading_GetSpeedOut(void);          /* 速度环输出 PWM     */
int16_t Heading_GetTurnOut(void);           /* 转向环输出 PWM     */
float   Heading_GetGyroZ(void);             /* 已应用符号的 Z 轴角速度 (°/s) */

#endif /* HEADING_H */
