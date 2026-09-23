#ifndef ULTRASOUND_H
#define ULTRASOUND_H
#include <stdint.h>

#define ULTRASOUND_DISTANCE_INVALID (-1.0f)
/* No rising Echo edge within the acquisition window means no target was found
 * in the useful range. This positive sentinel is deliberately above the
 * largest accepted measured distance (510 cm), so threshold comparisons treat
 * it as clear while callers can still distinguish it from a real distance. */
#define ULTRASOUND_DISTANCE_NO_ECHO (1000.0f)
typedef struct {
    float left_cm;
    float right_cm;
    uint8_t valid; /* Both channels are fresh: measured distance or NO_ECHO. */
} UltrasoundSample;

/* Left trigger PB10 / echo PB0; right trigger PB11 / echo PB1.
 * Owns TIM4, EXTI0 and EXTI1. SysTick remains owned by Platform.
 * Init/Update/getters run in main context; ISRs only capture pulse timing. */
void Ultrasound_Init(void);
void Ultrasound_Update(void); /* Nonblocking; call regularly (10 ms). */
void Ultrasound_GetSample(UltrasoundSample *sample);
/* Compatibility getters: cached data only, never start a measurement. */
float Ultrasound_GetLeftDistance(void);
float Ultrasound_GetRightDistance(void);
void Ultrasound_GetDistances(float *left, float *right);
#endif
