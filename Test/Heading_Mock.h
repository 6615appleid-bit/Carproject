/* 航向模块的测试替身。
 *
 * 供状态机测试（Test/test_car_control.c）和模块联调测试
 * （Test/ModuleHardware/test_modules.c）链接：二者都编译真实的
 * Track.c / Avoid.c / CarControl.c，但没有 MPU6050 和编码器硬件，
 * 所以用这个替身记录调用情况，让测试能验证：
 *   - 循迹/避障只把「目标航向角 + 目标速度」交给航向环，自己不写电机；
 *   - 航向模块是唯一写电机的地方（mock_heading_drive = 1 时）；
 *   - 启动被拒时故障码是 CAR_ERROR_IMU、运行中读取失败会转 CAR_FAULT；
 *   - 里程到 CAR_ODOMETER_STOP_COUNTS 自动锁存停车。
 */
#ifndef HEADING_MOCK_H
#define HEADING_MOCK_H
#include <stdint.h>
#include "Heading.h"

/* 测试可写：1 = 惯性单元就绪（决定 Start/StartHeading 能否成功） */
extern volatile uint8_t  mock_heading_ready;
/* 测试可写：1 = 航向模块报故障（模拟运行中 IMU 读取失败） */
extern volatile uint8_t  mock_heading_fault;
/* 测试可写：非 0 时 Heading_Update 会写电机，写出的是一组固定可辨认的 PWM */
extern volatile uint8_t  mock_heading_drive;
/* 测试可写：当前航向角（度）。循迹的航向基准、避障的到位判据都读它，
 * 所以测试可以用它模拟「车已经转到目标角度」。 */
extern volatile float    mock_heading_yaw;

/* 测试可读：调用计数与最近一次下发值 */
extern volatile uint32_t mock_heading_update_calls;
extern volatile uint32_t mock_heading_reset_calls;
extern volatile uint32_t mock_heading_init_calls;
extern volatile uint32_t mock_heading_stop_calls;
extern volatile uint32_t mock_heading_odo_resets;
extern volatile float    mock_heading_target;
extern volatile int16_t  mock_heading_speed;
/* 累计里程读数，测试可写以验证透传与自动停车 */
extern volatile int32_t  mock_heading_odo_left;
extern volatile int32_t  mock_heading_odo_right;

/* 断言辅助：把替身的全部状态复位成「刚上电」 */
void Heading_MockReset(void);

#endif /* HEADING_MOCK_H */
