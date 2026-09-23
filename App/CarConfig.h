/* 文件用途：集中配置整车调度周期和避障超时；参数需要实车标定。 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* Maximum recovery window from the first unusable line reading.
 * LOST starts a low-speed search; INVALID stops immediately.
 * 0 disables recovery and faults on the first unusable reading. */
#define CAR_TRACK_LOSS_TOLERANCE_MS 400U
/* Open-loop PWM, not measured speed. Preserve the last steering direction;
 * both wheels stay forward and the outer wheel is capped at 300/1000. */
#define CAR_TRACK_RECOVERY_DEMAND 200
#define CAR_TRACK_RECOVERY_CORRECTION 100
#if CAR_TRACK_RECOVERY_DEMAND <= 0 || CAR_TRACK_RECOVERY_CORRECTION < 0 || \
    CAR_TRACK_RECOVERY_CORRECTION > CAR_TRACK_RECOVERY_DEMAND || \
    CAR_TRACK_RECOVERY_DEMAND + CAR_TRACK_RECOVERY_CORRECTION > 1000
#error Invalid line recovery motor demand
#endif
/* Provisional integration settings; calibrate on the actual vehicle. */
#define CAR_CONTROL_PERIOD_MS 10U     //CarControl每隔10毫秒执行一轮控制
#define CAR_AVOID_TIMEOUT_MS 12000U	 //从开始避障算起，绕障加找线超过12秒就故障停车
#endif
//Car_ctrl 管理的所有参数集中放在一起，方便以后修改。
