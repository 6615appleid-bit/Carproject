/* 文件用途：整车控制对外接口：main通过这些函数启动、更新、停止小车并读取状态。 */
#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdint.h>

typedef enum {
    /* 等待、循迹、避障、到达终点、故障；终点与故障状态会锁存。 */
    CAR_IDLE, CAR_TRACKING, CAR_AVOIDING, CAR_FINISHED, CAR_FAULT
} CarState;
typedef enum {
    /* 无故障、循迹数据异常、测距数据异常。 */
    CAR_ERROR_NONE, CAR_ERROR_TRACK, CAR_ERROR_RANGE,
    /* 避障启动失败、避障执行失败、避障总时间超限。 */
    CAR_ERROR_AVOID_START, CAR_ERROR_AVOID_FAILED, CAR_ERROR_AVOID_TIMEOUT
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
/* Call only after an external, qualified finish decision. WIDE alone is not finish. */
/* 通知整车已经确认到达终点并锁定停车；本函数不负责识别终点。 */
void CarControl_ConfirmFinish(void);
#endif
