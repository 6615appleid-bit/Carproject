/* 文件用途：主程序入口，初始化后反复调用 CarControl；当前 debug 变量仍保留，
 * 便于调试器在线观察和下发命令，但不是比赛用的启动入口。
 *
 * ============================================================================
 * ★★ 转向只有一个入口：目标航向角 + 目标速度 ★★
 *
 * 循迹、避障、航向三种模式都不再自己拼 PWM 差速，而是各自给出
 * 「目标航向角 + 目标速度」，由 Hardware/Heading.c 的转向环（陀螺闭环）
 * 变成左右轮差速。所以“转向”在任何模式下都经过同一条路径：
 *
 *     线偏差 → 目标航向角 ─┐
 *                          ├→ 转向环(P + 角速度D) ─┐
 *     陀螺积分航向 ────────┘                       ├→ 差速 → 电机
 *     目标速度 ─┐                                   │
 *     编码器 ────→ 速度环(PI) ────────────────────┘
 *
 * 本周期实际下发的命令可以在调试器里看 debug_steer_deg / debug_speed_cmd。
 * ★ 里程：本次运行累计里程（左右轮计数之和）到 CAR_ODOMETER_STOP_COUNTS
 *   （App/CarConfig.h，默认 100000）自动停车并锁存 CAR_FINISHED。
 *
 * ============================================================================
 * ★★ 顶层航向角接口（航向模式）★★
 *
 * 想「让车自己转到某个方向并保持」，只需要改下面这两个变量：
 *
 *     heading_target_deg   目标航向角（度），正 = 左转（俯视逆时针）
 *                          0  = 沿进入航向模式时的车头朝向直行
 *                          90 = 左转 90 度后锁住
 *     heading_speed_cmd    目标速度（左右轮编码器计数之和 / 10 毫秒）
 *                          0  = 原地对准（对准时最常用）
 *                          20~40 左右就是边走边对准
 *
 * 这两个变量是 volatile 全局量，直接在调试器的 Watch / Expressions 窗口里
 * 改就能实时生效，不需要重新编译烧录。
 *
 * 进入航向模式的方式：把 debug_command 写成 5；或把下面的
 * CAR_START_MODE_HEADING 改成 1，上电就自动进入航向模式。
 *
 * 编码器读数、当前航向角等观测量见文件末尾的 debug_* 变量，
 * 调试时放进 Watch 窗口即可。
 * ============================================================================
 */
#include "Platform.h"
#include "CarControl.h"
#include "Heading.h"
#include "Imu.h"
/* Power-on auto start. It only retries the same guarded entry point the host
 * uses; CarControl_Start stays rejected until Track and Avoid report usable,
 * stable data, so this never moves the car on its own. Debug commands 2 and 3
 * latch the car off, so an operator stop is not undone by the retry. */
#define AUTO_START_RETRY_MS 500U

/* 上电默认进入哪个模式：
 *   0 = 原来的循迹模式（默认，保持参加比赛那套流程不变）
 *   1 = 航向模式（预热结束后自动进入，按 heading_target_deg 对准）
 * 航向模式不依赖循迹模块，所以选 1 时不会再等那 5 秒循迹预热。 */
#define CAR_START_MODE_HEADING 0

/* 陀螺零偏标定的重试次数；标定期间车必须静止。 */
#define GYRO_CALIB_RETRY 5U

/* 1=start, 2=stop, 3=confirmed finish, 4=reset to idle, 5=start heading mode. */
volatile unsigned int debug_command; /* 调试命令：1启动，2停止，3确认终点，4重新初始化，5进入航向模式 */
volatile unsigned int debug_result; /* 最近一次命令的结果：1成功，0未被接受或失败 */
volatile CarState debug_state; /* 在调试器里观察当前整车状态 */
volatile CarError debug_error; /* 在调试器里观察故障原因 */

/* --- 顶层航向接口：调试器里直接改这两个值 --- */
volatile float heading_target_deg;  /* 目标航向角（度），正 = 左转 */
volatile int   heading_speed_cmd;   /* 目标速度（编码器计数/10ms），0 = 原地 */

/* --- 惯导与编码器的只读观测量，方便在线观察 --- */
volatile uint8_t debug_imu_ready;       /* 1 = MPU6050 初始化成功 */
volatile uint8_t debug_imu_who_am_i;    /* WHO_AM_I 原始读数，正常 = 0x68 */
volatile uint8_t debug_imu_scan;        /* I2C 首个应答地址，0xFF = 无人应答 */
volatile float   debug_gyro_bias_z;     /* 标定得到的 Z 轴零偏 (°/s) */
volatile float   debug_heading;         /* 当前航向角（度） */
volatile float   debug_heading_target;  /* 当前目标航向角（度） */
volatile int     debug_enc_left;        /* 左轮本周期编码器增量 */
volatile int     debug_enc_right;       /* 右轮本周期编码器增量 */
volatile int     debug_odometer;        /* 本次运行累计编码器计数（自动停车的依据） */
volatile float   debug_steer_deg;       /* 本周期下发的目标航向角（度），正 = 左转 */
volatile int     debug_speed_cmd;       /* 本周期下发的目标速度（计数 / 10ms） */

/* 初始化 MPU6050 并完成陀螺零偏标定。
 * ★ 调用时必须让小车静止！标定会阻塞约 1.1 秒，失败会重试。
 * 返回 1 = 就绪；失败不影响循迹模式，只是进不了航向模式。 */
static uint8_t InitImu(void)
{
    unsigned int attempt;
    uint8_t ret;

    if (Heading_ImuInit() != 0U) {
        /* 初始化失败：WHO_AM_I 和总线扫描地址可用来区分原因 */
        debug_imu_ready    = 0U;
        debug_imu_who_am_i = Imu_GetWhoAmI();
        debug_imu_scan     = Imu_GetScanAddress();
        return 0U;
    }

    ret = 1U;
    for (attempt = 0U; attempt < GYRO_CALIB_RETRY; attempt++) {
        ret = Heading_CalibrateGyro();
        if (ret == 0U) break;    /* 0 = 成功；3 = 车在动，值得重试 */
    }

    debug_imu_ready    = 1U;
    debug_imu_who_am_i = Imu_GetWhoAmI();
    debug_imu_scan     = Imu_GetScanAddress();
    debug_gyro_bias_z  = Imu_GetBiasZ();
    (void)ret;
    return 1U;
}

int main(void)
{
    unsigned int command;
    uint8_t auto_start = 1U; /* Debug commands 2/3 clear it; 1/4 set it again. */
    uint32_t auto_try_ms;
    DriveCmd drive_cmd;
    Platform_Init();
    CarControl_Init();

    /* ★ 惯性单元初始化 + 零偏标定：此时车必须静止！
     * 放在自动启动之前，保证第一拍控制就拿到有效的零偏。 */
    (void)InitImu();

    auto_try_ms = Platform_GetMs();
    for (;;) {
        command = debug_command;
        debug_command = 0U; /* 有读取即清零，避免同一条命令在循环里被重复执行 */
        switch (command) {
        case 1U: auto_start = 1U; debug_result = CarControl_Start(); break;
        case 2U: auto_start = 0U; CarControl_Stop(); break;
        case 3U: auto_start = 0U; CarControl_ConfirmFinish(); break;
        case 4U: auto_start = 1U; CarControl_Init(); auto_try_ms = Platform_GetMs(); break;
        case 5U: auto_start = 0U; debug_result = CarControl_StartHeading(); break;
        default: break;
        }

        /* ★ 顶层只做两件事：给目标航向角、给目标速度。
         * 状态不是航向模式时 CarControl 会忽略这两个设置。 */
        if (CarControl_GetState() == CAR_HEADING) {
            CarControl_SetSpeed((int16_t)heading_speed_cmd);
            CarControl_SetHeading(heading_target_deg);
        }
        /* Automatic start is only an attempt: the guarded entry point rejects
         * the request during the 5 s warm-up and until the sensors are
         * usable, so the loop keeps waiting instead of forcing motion. */
        if (auto_start && CarControl_GetState() == CAR_IDLE) {
#if (CAR_START_MODE_HEADING > 0)
            /* 航向模式不依赖循迹模块，不用等 5 秒预热，成功一次就够 */
            (void)auto_try_ms;
            if (CarControl_StartHeading() != 0U) {
                auto_start = 0U;
            }
#else
            uint32_t now = Platform_GetMs();
            if ((uint32_t)(now - auto_try_ms) >= AUTO_START_RETRY_MS) {
                auto_try_ms = now;
                (void)CarControl_Start();
            }
#endif
        }
        CarControl_Update();

        /* 每轮结束前刷新观测量，方便在调试器 Watch 窗口里看。 */
        debug_state = CarControl_GetState();
        debug_error = CarControl_GetError();
        debug_heading        = CarControl_GetHeading();
        debug_heading_target = CarControl_GetHeadingTarget();
        debug_enc_left       = CarControl_GetEncoderLeft();
        debug_enc_right      = CarControl_GetEncoderRight();
        debug_odometer       = CarControl_GetOdometer();
        /* 本周期真正下发给转向环的命令：不管是循迹、避障还是航向模式，
         * 转向都只由这两个数决定，调参时盯它们最直接。 */
        CarControl_GetDrive(&drive_cmd);
        debug_steer_deg      = drive_cmd.steer_deg;
        debug_speed_cmd      = drive_cmd.speed_cmd;
    }
}
