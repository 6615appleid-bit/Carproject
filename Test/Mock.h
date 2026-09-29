/* 文件用途：模拟模块的测试变量声明：用于注入传感器状态、观察指令和调用次数，不连接真实传感器。 */
#ifndef MOCK_H
#define MOCK_H
#include "Track.h"
#include "Avoid.h"
/* Debugger-editable inputs. Mock sampling treats each scheduled sense call as
 * one fresh simulated sample. Real drivers must check actual data freshness. */
extern volatile TrackState mock_line_state; /* 模拟红外状态，由测试或调试器设置。 */
extern volatile int16_t mock_line_error; /* 模拟黑线偏差，负数在左，正数在右。 */
extern volatile AvoidCheck mock_obstacle; /* 模拟测距处理结果：无效、无障碍、有障碍。 */
extern volatile AvoidStatus mock_avoid_phase; /* 模拟避障当前阶段，可由测试手动推进。 */
extern volatile uint8_t mock_start_ok; /* 设为0可模拟避障启动失败。 */
extern volatile uint8_t mock_fail_on_update; /* 设为1可模拟执行避障时发生故障。 */
extern volatile uint32_t mock_track_calls; /* 记录循迹控制调用次数，检查是否被错误调用。 */
extern volatile uint32_t mock_avoid_calls; /* 记录避障控制调用次数，检查控制权是否互斥。 */
/* 循迹/避障现在只输出「目标航向角 + 目标速度」，不写电机；测试断言这两个值
 * 被 CarControl 原样转交给了航向环（Heading_Drive）。 */
extern volatile float   mock_track_steer;
extern volatile int16_t mock_track_speed;
/* 丢线时循迹给出的搜索命令（真实模块会沿最后一次的修正方向继续转）。
 * 默认 0/0 = 替身不搜索，也就是「丢线即停车」；测试可以设成非零来验证
 * 整车在丢线期间照常把命令转交给航向环。 */
extern volatile float   mock_track_search_steer;
extern volatile int16_t mock_track_search_speed;
extern volatile float   mock_avoid_steer;
extern volatile int16_t mock_avoid_speed;
extern volatile int16_t mock_motor_left; /* 保存左轮模拟需求，不代表真实车轮速度。 */
extern volatile int16_t mock_motor_right; /* 保存右轮模拟需求，不代表真实车轮速度。 */
extern volatile uint8_t mock_done_on_update;
extern volatile uint32_t mock_avoid_reset_calls;
extern volatile uint32_t mock_track_reset_calls;
extern volatile uint32_t mock_track_history;
#endif
