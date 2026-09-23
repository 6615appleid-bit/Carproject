/* STM32F103C8 motor driver. Wiring and polarity follow the balance-car
 * HARDWARE/MOTOR/motor.c and HARDWARE/PWM/pwm.c reference implementation.
 * TIM1 is exclusively owned by this driver; no encoder or balance loop needed.
 */
#include "Motor.h"
#include "stm32f10x.h"

#define MOTOR_DIRECTION_PINS (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

static uint8_t initialized;
static int8_t previous_left_direction;
static int8_t previous_right_direction;

static int16_t ClampDemand(int16_t demand)
{
    if (demand > MOTOR_DEMAND_MAX) return MOTOR_DEMAND_MAX;
    if (demand < -MOTOR_DEMAND_MAX) return -MOTOR_DEMAND_MAX;
    return demand;
}

static int8_t Direction(int16_t demand)
{
    if (demand > 0) return 1;
    if (demand < 0) return -1;
    return 0;
}

static uint16_t DutyCounts(int16_t demand)
{
    /* Cast before negation; even INT16_MIN input is first clamped. */
    uint32_t magnitude = (uint32_t)(demand < 0 ? -(int32_t)demand : demand);
    return (uint16_t)((magnitude * MOTOR_PWM_PERIOD_COUNTS) / MOTOR_DEMAND_MAX);
}

static void CommitPwm(uint16_t left, uint16_t right)
{
    TIM_SetCompare1(TIM1, left);
    TIM_SetCompare4(TIM1, right);
    /* Latch both preloaded values now. In particular, Stop must not wait for
     * the next timer overflow while the old nonzero duty remains active. */
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef output;

    initialized = 0U;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_DeInit(TIM1);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, DISABLE);

    GPIO_ResetBits(GPIOB, MOTOR_DIRECTION_PINS);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = MOTOR_DIRECTION_PINS;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* Configure zero duty before connecting PA8/PA11 to the PWM peripheral. */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 0U;
    timer.TIM_Period = MOTOR_PWM_PERIOD_COUNTS - 1U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &timer);

    /* StructInit also disables complementary outputs on PB13/PB14/PB15,
     * which this board uses as ordinary direction GPIOs. */
    TIM_OCStructInit(&output);
    output.TIM_OCMode = TIM_OCMode_PWM1;
    output.TIM_OutputState = TIM_OutputState_Enable;
    output.TIM_OCPolarity = TIM_OCPolarity_High;
    output.TIM_Pulse = 0U;
    TIM_OC1Init(TIM1, &output);
    TIM_OC4Init(TIM1, &output);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    CommitPwm(0U, 0U);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    previous_left_direction = 0;
    previous_right_direction = 0;
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    initialized = 1U;
}

void Motor_Stop(void)
{
    if (!initialized) return;
    CommitPwm(0U, 0U);
    GPIO_ResetBits(GPIOB, MOTOR_DIRECTION_PINS);
    previous_left_direction = 0;
    previous_right_direction = 0;
}

void Motor_SetDemand(int16_t left, int16_t right)
{
    int8_t left_direction;
    int8_t right_direction;
    uint16_t pins = 0U;

    if (!initialized) return;
    left = (int16_t)(ClampDemand(left) * MOTOR_LEFT_POLARITY);
    right = (int16_t)(ClampDemand(right) * MOTOR_RIGHT_POLARITY);
    left_direction = Direction(left);
    right_direction = Direction(right);

    if (left_direction != previous_left_direction ||
        right_direction != previous_right_direction) {
        /* Remove drive before changing direction. This is not a mechanical
         * deceleration ramp; higher-level code must choose its motion profile. */
        CommitPwm(0U, 0U);
        GPIO_ResetBits(GPIOB, MOTOR_DIRECTION_PINS);
        /* Original Load(): positive left -> PB14=0, PB15=1;
         * positive right -> PB13=0, PB12=1. */
        if (left_direction > 0) pins |= GPIO_Pin_15;
        if (left_direction < 0) pins |= GPIO_Pin_14;
        if (right_direction > 0) pins |= GPIO_Pin_12;
        if (right_direction < 0) pins |= GPIO_Pin_13;
        GPIO_SetBits(GPIOB, pins);
        previous_left_direction = left_direction;
        previous_right_direction = right_direction;
    }
    CommitPwm(DutyCounts(left), DutyCounts(right));
}
