/* 正交编码器读取。硬件接线沿用原平衡车，与 stm32test 完全一致：
 *
 *   左编码器 : TIM2  CH1 = PA0, CH2 = PA1
 *   右编码器 : TIM4  CH1 = PB6, CH2 = PB7
 *
 * 注意 TIM4：stm32test 里 TIM4 是右编码器的，本工程原来把它给超声波做时基。
 * 为了不动接线，超声波已改为 TIM3（它只把定时器当时基用，不占任何引脚），
 * TIM4 还给右编码器。见 Hardware/Ultrasound.c 顶部注释。
 *
 * 计数方式同样沿用 stm32test：编码器模式 TI12（A、B 相都计上升沿，4 倍频）、
 * 输入滤波 10（压电机换向噪声）、ARR = 65535。
 */
#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

#define ENC_CH_LEFT   0U
#define ENC_CH_RIGHT  1U

/* 某轮「前进」时计数读数为正 → +1；读出负数 → -1。
 *
 * ⚠️ 只能翻这里，绝对不要「编码器和电机一起翻」：两次反相互相抵消，
 *    速度环的增益符号变成正反馈，PWM 会一路顶到饱和、转速乱飘。
 *    详见 stm32test 的 car_config.h 里 SIGN_ENC_L / SIGN_ENC_R 那段推导。
 * 默认值与 stm32test 实测一致（左 +1 / 右 -1），是否适用于本车必须实测。 */
#ifndef ENCODER_SIGN_LEFT
#define ENCODER_SIGN_LEFT   (+1)
#endif
#ifndef ENCODER_SIGN_RIGHT
#define ENCODER_SIGN_RIGHT  (-1)
#endif
#if (ENCODER_SIGN_LEFT != 1 && ENCODER_SIGN_LEFT != -1) || \
    (ENCODER_SIGN_RIGHT != 1 && ENCODER_SIGN_RIGHT != -1)
#error Encoder sign must be +1 or -1
#endif

/* 初始化 TIM2 / TIM4 为编码器接口模式，并把计数器清零。 */
void    Encoder_Init(void);

/* 读走本周期计数增量并清零，返回值已乘上对应的符号。
 * 正 = 该轮前进。100 Hz 控制周期下每周期只有几十个计数，所以
 * 「读 + 清零」之间丢掉一个脉冲的影响可以忽略，做法与 stm32test 一致。 */
int16_t Encoder_GetCount(uint8_t ch);

/* 上电（或上次 Encoder_ResetTotal）以来的累计计数，正 = 前进。
 * 速度环用 Encoder_GetCount 的周期增量；累计值用于里程：
 * CarControl_GetOdometer()（左右之和）和「本次运行到 CAR_ODOMETER_STOP_COUNTS
 * 自动停车」（App/CarConfig.h）。单位是编码器计数，不是毫米。 */
int32_t Encoder_GetTotal(uint8_t ch);

/* 只清累计里程，不动计数器；用于标定/清零里程。 */
void    Encoder_ResetTotal(void);

#endif /* ENCODER_H */
