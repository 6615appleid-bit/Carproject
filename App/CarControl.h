/* 文件用途：整车控制对外接口：main通过这些函数启动、更新、停止小车并读取状态。
 *
 * ★ 转向接口：三种运行模式（循迹 CAR_TRACKING、避障 CAR_AVOIDING、航向
 *   CAR_HEADING）都只决定「目标航向角 + 目标速度」，车轮差速统一由
 *   Hardware/Heading.c 的转向环（陀螺闭环）产生，模块之间不再各自拼 PWM。
 * ★ 里程停车：本次运行累计里程到 CAR_ODOMETER_STOP_COUNTS 时自动停车并锁存
 *   CAR_FINISHED，阈值在 App/CarConfig.h。 */
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdint.h>
#include "Heading.h" /* DriveCmd：全车统一的「目标航向角 + 目标速度」 */

typedef enum {
    /* 等待、循迹、避障、航向对准、到达终点、故障；终点与故障状态会锁存。 */
    CAR_IDLE, CAR_TRACKING, CAR_AVOIDING, CAR_HEADING, CAR_FINISHED, CAR_FAULT
} CarState;
typedef enum {
    /* 无故障、循迹数据异常、测距数据异常、惯性单元异常。 */
    CAR_ERROR_NONE, CAR_ERROR_TRACK, CAR_ERROR_RANGE,
    /* 避障启动失败、避障执行失败、避障总时间超限。 */
    CAR_ERROR_AVOID_START, CAR_ERROR_AVOID_FAILED, CAR_ERROR_AVOID_TIMEOUT,
    CAR_ERROR_IMU
} CarError;

/* 初始化模块并停止电机，进入等待状态；也用于非比赛期间解除终点或故障锁存。 */
void CarControl_Init(void);
/* 请求启动：只有等待状态、稳定黑线和有效测距才成功；返回1成功，0拒绝。 */
uint8_t CarControl_Start(void);
/* 执行一轮整车协调；内部限制为每10毫秒一步，未到时间立即返回。 */
void CarControl_Update(void);
/* 立即停车并重置模块；运行状态回到等待，但不会解除终点或故障锁存。 */
void CarControl_Stop(void); /* manual stop -> IDLE; terminal states stay latched */
/* 读取整车当前状态，不改变状态，也不控制电机。 */
CarState CarControl_GetState(void);
/* 读取故障原因，用于调试定位；无故障时返回CAR_ERROR_NONE。 */
CarError CarControl_GetError(void);
/* 读取最近一次下发给航向环的「目标航向角 + 目标速度」，用于调试观察。
 * 循迹、避障、航向三种模式都从同一个出口驱动车轮，所以它反映的就是当前转向。 */
void CarControl_GetDrive(DriveCmd *out);
/* Call only after an external, qualified finish decision. WIDE alone is not finish. */
/* 通知整车已经确认到达终点并锁定停车；本函数不负责识别终点。 */
void CarControl_ConfirmFinish(void);

/* ========================================================================
 * 航向模式：顶层只给一个目标航向角，小车自己转到该方向并保持。
 *
 *   进入航向模式时把「车头当前朝向」重新定义为 0°，因此
 *       SetHeading(0.0f)   = 沿当前方向直行
 *       SetHeading(90.0f)  = 左转 90 度后锁住
 *   角度约定：正 = 左转（俯视逆时针），可超出 ±180 度。
 *   目标角和目标速度在 Stop/Init 后仍然保留，但每次进入航向模式
 *   航向角都会重新归零。
 *
 * 航向反馈来自 MPU6050 的陀螺 Z 轴积分，速度反馈来自编码器，
 * 因此本模式不依赖循迹模块和超声波；但测距仍会照常刷新，
 * 供上层做前方避让判断。
 * ======================================================================*/

/* 从等待状态进入航向模式。成功返回 1；状态不是等待、或惯性单元未就绪
 * 时返回 0（故障码置为 CAR_ERROR_IMU）。 */
uint8_t CarControl_StartHeading(void);

/* 设置目标航向角，单位度，正 = 左转。 */
void CarControl_SetHeading(float degrees);

/* 在当前目标角基础上再转 delta 度，正 = 左转。 */
void CarControl_SetHeadingRelative(float delta_degrees);

/* 设置目标速度：左右轮编码器计数之和 / 10 毫秒。
 * 填 0 就是原地对准，这也是对准目标角最常用的方式。 */
void CarControl_SetSpeed(int16_t speed_cmd);

/* 只读观测量，主要用于调试器 Watch 窗口。 */
float   CarControl_GetHeading(void);        /* 当前航向角(度)   */
float   CarControl_GetHeadingTarget(void);  /* 目标航向角(度)   */
int16_t CarControl_GetEncoderLeft(void);    /* 左轮本周期增量   */
int16_t CarControl_GetEncoderRight(void);   /* 右轮本周期增量   */
int32_t CarControl_GetOdometer(void);       /* 两轮累计计数     */
#endif
