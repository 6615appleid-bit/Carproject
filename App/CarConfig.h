/* 文件用途：集中配置整车调度周期和避障超时；参数需要实车标定。 */
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* Line-loss tolerance: a single wide black group still counts as a usable line,
 * and a lost or invalid reading only latches CAR_ERROR_TRACK after this long.
 * 丢线期间不再立刻停车：Track 会沿最后一次的修正方向继续转向找线
 * （TRACK_SEARCH_TIMEOUT_MS = 1000 ms，见 Hardware/track.c），
 * 找到线就立刻回到正常循迹，超过 1 秒没找到才停止搜索。
 * 本值必须大于 TRACK_SEARCH_TIMEOUT_MS，否则搜索还没结束就被判故障。
 * 0 restores the old one-frame fail behaviour. */
#define CAR_TRACK_LOSS_TOLERANCE_MS 1300U
/* Provisional integration settings; calibrate on the actual vehicle. */
#define CAR_CONTROL_PERIOD_MS 10U     //CarControl每隔10毫秒执行一轮控制
#define CAR_AVOID_TIMEOUT_MS 12000U	 //从开始避障算起，绕障加找线超过12秒就故障停车
/* Run-distance auto stop: when the odometer (left + right encoder counts
 * accumulated since the current run started) reaches this value, the car
 * stops and latches CAR_FINISHED. 0 disables the check.
 * 100000 counted edges is a provisional value: convert it to a real distance
 * with the measured counts-per-mm (see README, odometer calibration). */
#define CAR_ODOMETER_STOP_COUNTS 100000
#endif
//Car_ctrl 管理的所有参数集中放在一起，方便以后修改。
