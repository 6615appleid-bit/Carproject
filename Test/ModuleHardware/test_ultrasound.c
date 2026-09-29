#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "Ultrasound.h"

GPIO_TypeDef module_gpio_a, module_gpio_b, module_gpio_c;
TIM_TypeDef module_tim3;
static uint32_t now, primask, exti_pending;
static uint16_t tim_pending, interrupt_enable;
static unsigned int nvic_count;
void TIM3_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
uint32_t Platform_GetMs(void) { return now; }
uint32_t Module_GetPrimask(void) { return primask; }
void Module_SetPrimask(uint32_t value) { primask = value; }
void Module_DisableIrq(void) { primask = 1U; }
void RCC_APB2PeriphClockCmd(uint32_t mask, FunctionalState state) { (void)mask; assert(state == ENABLE); }
void RCC_APB1PeriphClockCmd(uint32_t mask, FunctionalState state) { assert(mask == RCC_APB1Periph_TIM3 && state == ENABLE); }
void RCC_GetClocksFreq(RCC_ClocksTypeDef *clocks)
{ memset(clocks, 0, sizeof(*clocks)); clocks->PCLK1_Frequency = 36000000; clocks->HCLK_Frequency = 72000000; }
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{ assert(port == GPIOB); assert(gpio->GPIO_Mode == GPIO_Mode_Out_PP || gpio->GPIO_Mode == GPIO_Mode_IPD); }
void GPIO_ResetBits(GPIO_TypeDef *port, uint16_t pins) { assert(port == GPIOB); port->ODR &= ~(uint32_t)pins; }
void GPIO_SetBits(GPIO_TypeDef *port, uint16_t pins) { assert(port == GPIOB); port->ODR |= pins; }
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *port, uint16_t pin) { assert(port == GPIOB); return (port->IDR & pin) != 0; }
void GPIO_EXTILineConfig(uint8_t port, uint8_t pin) { assert(port == GPIO_PortSourceGPIOB && pin <= GPIO_PinSource1); }
void TIM_DeInit(TIM_TypeDef *tim) { assert(tim == TIM3); memset(tim, 0, sizeof(*tim)); interrupt_enable = tim_pending = 0; }
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *timer) { memset(timer, 0, sizeof(*timer)); }
void TIM_TimeBaseInit(TIM_TypeDef *tim, TIM_TimeBaseInitTypeDef *timer)
{ assert(tim == TIM3 && timer->TIM_Prescaler == 71 && timer->TIM_Period == 65535); }
void TIM_OCStructInit(TIM_OCInitTypeDef *output) { memset(output, 0, sizeof(*output)); }
void TIM_OC1Init(TIM_TypeDef *tim, TIM_OCInitTypeDef *output)
{ assert(tim == TIM3 && output->TIM_OCMode == TIM_OCMode_Timing); }
void TIM_OC1PreloadConfig(TIM_TypeDef *tim, uint16_t state) { assert(tim == TIM3 && state == TIM_OCPreload_Disable); }
void TIM_Cmd(TIM_TypeDef *tim, FunctionalState state) { assert(tim == TIM3 && state == ENABLE); }
void TIM_ITConfig(TIM_TypeDef *tim, uint16_t mask, FunctionalState state)
{ assert(tim == TIM3); if (state == ENABLE) interrupt_enable |= mask; else interrupt_enable &= (uint16_t)~mask; }
ITStatus TIM_GetITStatus(TIM_TypeDef *tim, uint16_t mask) { assert(tim == TIM3); return (tim_pending & mask & interrupt_enable) ? SET : RESET; }
void TIM_ClearITPendingBit(TIM_TypeDef *tim, uint16_t mask) { assert(tim == TIM3); tim_pending &= (uint16_t)~mask; }
uint16_t TIM_GetCounter(TIM_TypeDef *tim) { assert(tim == TIM3); return (uint16_t)tim->CNT; }
void TIM_SetCompare1(TIM_TypeDef *tim, uint16_t value) { assert(tim == TIM3); tim->CCR1 = value; }
void EXTI_StructInit(EXTI_InitTypeDef *exti) { memset(exti, 0, sizeof(*exti)); }
void EXTI_Init(EXTI_InitTypeDef *exti) { assert(exti->EXTI_Line == 3 && exti->EXTI_Trigger == EXTI_Trigger_Rising_Falling); }
void EXTI_ClearITPendingBit(uint32_t mask) { exti_pending &= ~mask; }
ITStatus EXTI_GetITStatus(uint32_t mask) { return (exti_pending & mask) ? SET : RESET; }
void NVIC_Init(NVIC_InitTypeDef *nvic)
{ assert(nvic->NVIC_IRQChannelCmd == ENABLE); ++nvic_count; }

static UltrasoundSample Read(void) { UltrasoundSample result; Ultrasound_GetSample(&result); return result; }
static void Reset(uint32_t time)
{ now = time; GPIOB->IDR = 0; GPIOB->ODR = GPIO_Pin_7; primask = 0; Ultrasound_Init(); assert(!primask && !Read().valid); }
static void Trigger(uint8_t sensor)
{
    Ultrasound_Update();
    assert(GPIOB->ODR & (sensor ? GPIO_Pin_11 : GPIO_Pin_10));
    assert((uint16_t)(TIM3->CCR1 - TIM3->CNT) == 20U);
    tim_pending |= TIM_IT_CC1; TIM3_IRQHandler();
    assert((GPIOB->ODR & (GPIO_Pin_10 | GPIO_Pin_11)) == 0);
    assert(GPIOB->ODR & GPIO_Pin_7);
}
static void Edge(uint8_t sensor, uint8_t high)
{
    if (high) GPIOB->IDR |= 1U << sensor; else GPIOB->IDR &= ~(1U << sensor);
    exti_pending |= 1U << sensor;
    if (sensor) EXTI1_IRQHandler(); else EXTI0_IRQHandler();
}
static void Echo(uint8_t sensor, uint16_t width)
{
    Edge(sensor, 1);
    TIM3->CNT = (uint16_t)(TIM3->CNT + width);
    now += width / 1000U;
    Edge(sensor, 0);
}

int main(void)
{
    UltrasoundSample sample;
    Reset(0); assert(nvic_count == 3);
    TIM3->CNT = 65530U; Trigger(0);
    Edge(1, 1); Edge(1, 0); /* Inactive sensor must not complete acquisition. */
    Echo(0, 1000); Ultrasound_Update(); assert(!Read().valid);
    assert(Ultrasound_GetLeftDistance() > 16.9f && Ultrasound_GetLeftDistance() < 17.1f);
    now += 59; Ultrasound_Update(); assert((GPIOB->ODR & 0xc00U) == 0);
    ++now; Trigger(1); Echo(1, 2000); Ultrasound_Update();
    sample = Read(); assert(sample.valid && sample.right_cm > 33.9f && sample.right_cm < 34.1f);
    now += 401; assert(!Read().valid);

    Reset(UINT32_MAX - 20U); Trigger(0); Echo(0, 1000); Ultrasound_Update();
    now += 60; Trigger(1); Echo(1, 1000); Ultrasound_Update(); assert(Read().valid);
    now += 60; Trigger(0); now += 35; Ultrasound_Update();
    assert(Read().valid && Ultrasound_GetLeftDistance() == ULTRASOUND_DISTANCE_NO_ECHO);
    Edge(0, 1); Edge(0, 0); assert(Read().valid); /* Late echo cannot change NO_ECHO. */
    now += 60; Trigger(1); Echo(1, 1000); Ultrasound_Update();
    now += 60; Trigger(0); Echo(0, 1000); Ultrasound_Update(); assert(Read().valid);

    /* An empty scene is valid and clear after both channels time out without
     * an Echo rise; stale data remains invalid. */
    Reset(0); Trigger(0); now += 35; Ultrasound_Update();
    now += 60; Trigger(1); now += 35; Ultrasound_Update();
    sample = Read();
    assert(sample.valid && sample.left_cm == ULTRASOUND_DISTANCE_NO_ECHO &&
           sample.right_cm == ULTRASOUND_DISTANCE_NO_ECHO);
    now += 401; assert(!Read().valid);

    Reset(0); Trigger(0); Echo(0, 50); Ultrasound_Update();
    assert(Ultrasound_GetLeftDistance() == ULTRASOUND_DISTANCE_INVALID);
    Reset(0); GPIOB->IDR = GPIO_Pin_0; Ultrasound_Update();
    assert((GPIOB->ODR & 0xc00U) == 0 && !Read().valid);
    Reset(0); Trigger(0); Echo(0, 1000); now += 401; Ultrasound_Update();
    assert(Ultrasound_GetLeftDistance() == ULTRASOUND_DISTANCE_INVALID); /* Processing old IRQ data must not refresh its age. */
    Reset(0); Trigger(0); Edge(0, 1); now += 35; Ultrasound_Update();
    assert(Ultrasound_GetLeftDistance() == ULTRASOUND_DISTANCE_INVALID);
    primask = 1; Ultrasound_Update(); assert(primask == 1); primask = 0;
    puts("PASS: async ultrasound, NO_ECHO clear, signal faults, recovery, freshness, timer/millisecond wrap.");
    return 0;
}
