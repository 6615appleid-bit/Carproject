/* 文件用途：避障模块接口约定：包含障碍检测、绕行和找线；由CarControl决定控制权交接。 */
#ifndef AVOID_H
#define AVOID_H
#include <stdint.h>
#include "Heading.h" /* DriveCmd: steering = target heading + target speed. */
#include "Track.h" /* 下面接口用到TrackStatus类型，因此包含循迹头文件。 */
typedef enum {
    /* 测距不可用、未发现需避让的障碍、发现障碍。 */
    AVOID_CHECK_INVALID, AVOID_CHECK_CLEAR, AVOID_CHECK_DETECTED
} AvoidCheck;
typedef enum {
    /* 空闲、正在绕障、正在找线、已稳定回线完成、执行失败。 */
    AVOID_IDLE, AVOID_BYPASSING, AVOID_SEARCHING, AVOID_DONE, AVOID_FAILED
} AvoidStatus;
/* 初始化避障模块和内部状态。 */
void Avoid_Init(void);
/* 维护测距和障碍检测状态，不控制电机；真实模块内部管理采样周期与有效性。 */
void Avoid_SenseUpdate(void); /* bounded acquisition; never controls motors */
/* 读取障碍判断：无效、无障碍或有障碍；不启动避障，也不控制电机。 */
AvoidCheck Avoid_Check(void); /* returns processed result; no side effects */
/* 开始一次避障并进入绕行阶段；返回非零表示成功，本函数不直接驱动电机。 */
uint8_t Avoid_Start(void); /* nonzero success; enters BYPASSING; no motor writes */
/* 执行一步避障或找线；line指向本轮黑线状态，只读取，不修改；本函数可以控制电机。 */
/* One control step: sets the steering command only; never writes motors.
 * The vehicle layer hands the DriveCmd to Heading_Drive. */
void Avoid_Update(const TrackStatus *line);
/* Read the latest steering command (target heading + target speed). */
void Avoid_GetDrive(DriveCmd *out);
/* 读取避障当前阶段；读取不会清除DONE，直到整车调用Avoid_Reset。 */
AvoidStatus Avoid_GetStatus(void);
/* 结束或取消本次避障并回到空闲；不控制电机，停车由上层负责。 */
void Avoid_Reset(void); /* cancel state; no motor writes */
/* SEARCHING表示已绕过障碍。模块仅在该阶段确认稳定回线后进入DONE，
 * 必须基于寻线阶段的新鲜样本确认，不能沿用绕障阶段的确认历史。
 * DONE保持到Avoid_Reset；DONE状态调用Avoid_Update不得再输出运动指令。
 * 整车收到DONE后：停车 -> Avoid_Reset -> Track_Reset -> CAR_TRACKING。
 */
#endif
