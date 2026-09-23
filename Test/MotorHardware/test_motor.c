#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "Motor.h"

GPIO_TypeDef test_gpio_a;
GPIO_TypeDef test_gpio_b;
TIM_TypeDef test_tim1;
static uint16_t active_left, active_right;
static uint32_t clocks;
static int pwm_enabled, timer_enabled, gpio_ready, pwm_ready;
static int preload_left, preload_right;
static unsigned int commit_count;

void RCC_APB2PeriphClockCmd(uint32_t mask, FunctionalState state)
{
    assert(state == ENABLE);
    clocks |= mask;
}
void TIM_CtrlPWMOutputs(TIM_TypeDef *tim, FunctionalState state)
{
    assert(tim == TIM1);
    if (state == ENABLE) {
        assert(active_left == 0 && active_right == 0);
        assert(gpio_ready && pwm_ready && timer_enabled);
    }
    pwm_enabled = state == ENABLE;
}
void TIM_DeInit(TIM_TypeDef *tim)
{
    assert(tim == TIM1 && !pwm_enabled);
    memset(tim, 0, sizeof(*tim));
    active_left = active_right = 0;
    preload_left = preload_right = 0;
}
void GPIO_PinRemapConfig(uint32_t remap, FunctionalState state)
{
    assert(remap == GPIO_FullRemap_TIM1 && state == DISABLE);
}
void GPIO_ResetBits(GPIO_TypeDef *gpio, uint16_t pins)
{
    assert(gpio == GPIOB);
    /* Direction must never change while the old duty is still active. */
    assert(active_left == 0 && active_right == 0);
    gpio->ODR &= ~(uint32_t)pins;
}
void GPIO_SetBits(GPIO_TypeDef *gpio, uint16_t pins)
{
    assert(gpio == GPIOB);
    assert(active_left == 0 && active_right == 0);
    gpio->ODR |= pins;
}
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert(gpio->GPIO_Speed == GPIO_Speed_50MHz);
    if (port == GPIOB) {
        assert(gpio->GPIO_Pin == 0xf000U && gpio->GPIO_Mode == GPIO_Mode_Out_PP);
        assert((port->ODR & 0xf000U) == 0);
        gpio_ready = 1;
    } else {
        assert(port == GPIOA && gpio->GPIO_Pin == (GPIO_Pin_8 | GPIO_Pin_11));
        assert(gpio->GPIO_Mode == GPIO_Mode_AF_PP);
        assert(active_left == 0 && active_right == 0);
        pwm_ready = 1;
    }
}
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *timer)
{ memset(timer, 0, sizeof(*timer)); }
void TIM_TimeBaseInit(TIM_TypeDef *tim, TIM_TimeBaseInitTypeDef *timer)
{
    assert(tim == TIM1 && timer->TIM_Prescaler == 0);
    assert(timer->TIM_Period == 7199 && timer->TIM_CounterMode == TIM_CounterMode_Up);
    assert(timer->TIM_RepetitionCounter == 0);
    tim->ARR = timer->TIM_Period;
}
void TIM_OCStructInit(TIM_OCInitTypeDef *output)
{ memset(output, 0, sizeof(*output)); }
static void CheckOutput(const TIM_OCInitTypeDef *output)
{
    assert(output->TIM_OCMode == TIM_OCMode_PWM1);
    assert(output->TIM_OutputState == TIM_OutputState_Enable);
    assert(output->TIM_OutputNState == TIM_OutputNState_Disable);
    assert(output->TIM_OCPolarity == TIM_OCPolarity_High && output->TIM_Pulse == 0);
}
void TIM_OC1Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *output)
{ assert(tim == TIM1); CheckOutput(output); tim->CCR1 = output->TIM_Pulse; }
void TIM_OC4Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *output)
{ assert(tim == TIM1); CheckOutput(output); tim->CCR4 = output->TIM_Pulse; }
void TIM_OC1PreloadConfig(TIM_TypeDef *tim, uint16_t state)
{ assert(tim == TIM1 && state == TIM_OCPreload_Enable); preload_left = 1; }
void TIM_OC4PreloadConfig(TIM_TypeDef *tim, uint16_t state)
{ assert(tim == TIM1 && state == TIM_OCPreload_Enable); preload_right = 1; }
void TIM_ARRPreloadConfig(TIM_TypeDef *tim, FunctionalState state)
{ assert(tim == TIM1 && state == ENABLE); }
void TIM_SetCompare1(TIM_TypeDef *tim, uint16_t value)
{ assert(tim == TIM1); tim->CCR1 = value; }
void TIM_SetCompare4(TIM_TypeDef *tim, uint16_t value)
{ assert(tim == TIM1); tim->CCR4 = value; }
void TIM_GenerateEvent(TIM_TypeDef *tim, uint16_t event)
{
    assert(tim == TIM1 && event == TIM_EventSource_Update);
    assert(preload_left && preload_right);
    active_left = (uint16_t)tim->CCR1;
    active_right = (uint16_t)tim->CCR4;
    ++commit_count;
}
void TIM_ClearFlag(TIM_TypeDef *tim, uint16_t flag)
{ assert(tim == TIM1 && flag == TIM_FLAG_Update); }
void TIM_Cmd(TIM_TypeDef *tim, FunctionalState state)
{ assert(tim == TIM1); timer_enabled = state == ENABLE; }

static void Check(int left, int right, unsigned int left_duty, unsigned int right_duty)
{
    uint32_t expected = GPIO_Pin_2; /* Unrelated GPIO must remain untouched. */
    left *= MOTOR_LEFT_POLARITY;
    right *= MOTOR_RIGHT_POLARITY;
    if (left > 0) expected |= GPIO_Pin_15;
    if (left < 0) expected |= GPIO_Pin_14;
    if (right > 0) expected |= GPIO_Pin_12;
    if (right < 0) expected |= GPIO_Pin_13;
    assert(GPIOB->ODR == expected);
    assert(active_left == left_duty && active_right == right_duty);
}

int main(void)
{
    unsigned int before;
    Motor_Stop();
    Motor_SetDemand(1000, 1000);
    assert(commit_count == 0 && clocks == 0);
    GPIOB->ODR = GPIO_Pin_2;
    Motor_Init();
    Check(0, 0, 0, 0);
    assert(pwm_enabled && timer_enabled);
    assert(clocks == (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                      RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1));
    Motor_SetDemand(1000, 500); Check(1, 1, 7200, 3600);
    before = commit_count;
    Motor_SetDemand(250, 750); Check(1, 1, 1800, 5400);
    assert(commit_count == before + 1); /* Same direction: no forced zero step. */
    Motor_SetDemand(-500, 250); Check(-1, 1, 3600, 1800);
    Motor_SetDemand(250, -500); Check(1, -1, 1800, 3600);
    Motor_SetDemand(-1000, -1000); Check(-1, -1, 7200, 7200);
    Motor_SetDemand(INT16_MIN, INT16_MAX); Check(-1, 1, 7200, 7200);
    Motor_SetDemand(0, 1); Check(0, 1, 0, 7);
    Motor_SetDemand(-1, 0); Check(-1, 0, 7, 0);
    Motor_SetDemand(0, 0); Check(0, 0, 0, 0);
    Motor_SetDemand(600, 600);
    Motor_Stop(); Check(0, 0, 0, 0);
    Motor_Stop(); Check(0, 0, 0, 0);
    Motor_SetDemand(300, 400); Check(1, 1, 2160, 2880);
    Motor_Init(); Check(0, 0, 0, 0);
    puts("PASS: motor wiring, PWM, polarity, limits, reversal ordering, immediate stop, re-init.");
    return 0;
}
