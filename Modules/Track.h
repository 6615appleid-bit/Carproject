/* 文件用途：循迹模块接口约定：队友提供实现；感知与运动控制分开，便于避障时继续观察黑线。 */
#ifndef TRACK_H
#define TRACK_H
#include <stdint.h>

typedef enum {
    /* 数据无效、丢线、稳定正常、宽黑区域候选、无法可靠判断；宽黑不等于终点。 */
    TRACK_INVALID, TRACK_LOST, TRACK_NORMAL, TRACK_WIDE, TRACK_AMBIGUOUS
} TrackState;
typedef struct {
    TrackState state; /* 红外状态，不是整车运行状态。 */
    int16_t error; /* -1000..1000, line left < 0, line right > 0; NORMAL only */ /* 仅正常状态有效；这是归一化偏差，不是毫米。 */
} TrackStatus;

/* 初始化循迹模块；模拟版将输入设为无效，等待测试程序提供数据。 */
void Track_Init(void);
/* Bounded/nonblocking: sampling, freshness, filtering and stability internally.
 * Called even during avoidance. Must never control motors.
 * NORMAL means confirmed using fresh samples, not repeated reads of old data.
 */
/* 维护红外状态、有效性和稳定性；不控制电机，避障期间也应继续调用。 */
void Track_SenseUpdate(void);
/* 返回已处理的线状态和偏差；这是读取结果，不代表又采样了一次。 */
TrackStatus Track_GetStatus(void);
/* Read-only frame identity for peer modules (e.g. stable avoidance handoff).
 * Changes only when SenseUpdate consumes a new frame. Read status first;
 * an ID alone never implies valid data. CarControl need not inspect this. */
uint32_t Track_GetSampleId(void);
/* 执行一步循迹运动控制；只有整车处于正常循迹时才调用。 */
void Track_Update(void); /* one control step; may control motors; no long delay.
 * TRACK_NORMAL and a single wide black group (TRACK_WIDE) both drive motors. */
/* 清除控制器历史（如积分、上次误差），不写电机。
 * 保留感知结果、有效性及线路确认历史，避免交接后误报丢线。
 * 感知的初始化由Track_Init负责；数据新鲜度仍由SenseUpdate负责。
 */
void Track_Reset(void);
#endif
