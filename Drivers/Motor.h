/* Shared interface for the hardware driver and the host-test mock. */
#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#define MOTOR_DEMAND_MAX 1000
/* Original balance-car setting: TIM1 PSC=0, ARR=7199.
 * PWM frequency is 10 kHz when the TIM1 clock is 72 MHz. */
#define MOTOR_PWM_PERIOD_COUNTS 7200U

/* +1 preserves the original Load() polarity. Change a wheel to -1 if
 * its installed wiring makes a positive demand rotate it backwards. */
#ifndef MOTOR_LEFT_POLARITY
#define MOTOR_LEFT_POLARITY 1
#endif
#ifndef MOTOR_RIGHT_POLARITY
#define MOTOR_RIGHT_POLARITY 1
#endif
#if (MOTOR_LEFT_POLARITY != 1 && MOTOR_LEFT_POLARITY != -1) || \
    (MOTOR_RIGHT_POLARITY != 1 && MOTOR_RIGHT_POLARITY != -1)
#error Motor polarity must be +1 or -1
#endif

/* Left: PA8/TIM1_CH1, direction PB14/PB15.
 * Right: PA11/TIM1_CH4, direction PB13/PB12.
 * Driver STBY is tied high on the supplied schematic; no STBY GPIO.
 * Call from the main control context, not concurrently from interrupts.
 * Init owns TIM1 and initializes both GPIO and PWM with outputs stopped. */
void Motor_Init(void);

/* Immediate zero PWM and all four direction pins low. Physical stopping
 * distance depends on the bridge, motor and vehicle; no speed feedback here.
 * Calls before Init are ignored. A subsequent SetDemand resumes motion. */
void Motor_Stop(void);

/* Signed open-loop PWM demand, clamped to -1000..1000.
 * +1000 = 100% duty forwards, -1000 = 100% backwards, 0 = zero PWM.
 * This is NOT a wheel-speed command in mm/s. Calls before Init are ignored. */
void Motor_SetDemand(int16_t left, int16_t right);
#endif
