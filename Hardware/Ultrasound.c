/* Asynchronous dual ultrasound acquisition.
 * TIM3: free-running 1 MHz counter, CC1 interrupt ends the trigger pulse.
 * TIM4 is reserved for the right wheel encoder (PB6/PB7). This module only uses
 * a timer as a time base with no pins at all, so the move changes nothing
 * electrically: the trigger/echo pins PB10/PB11/PB0/PB1 stay where they were.
 * EXTI0/1: latch PB0/PB1 echo edges. No motor commands, Delay or busy waits.
 * Update is called by Avoid_SenseUpdate; getters only read cached samples.
 */
#include "Ultrasound.h"
#include "Platform.h"
#include "stm32f10x.h"

#define ECHO_TIMEOUT_MS 35U
#define QUIET_MS 60U
#define SAMPLE_STALE_MS 400U
#define MIN_ECHO_US 100U
#define MAX_ECHO_US 30000U
#define TRIGGER_US 20U

enum { CAP_IDLE, CAP_RISE, CAP_FALL, CAP_DONE };
static volatile uint8_t capture_state, active_sensor;
static volatile uint16_t rise_us, pulse_us;
static volatile uint32_t started_ms, completed_ms;
static uint8_t initialized, have_sample[2], next_sensor, have_finished;
static float distance_cm[2];
static uint32_t sample_ms[2], finished_ms;
static const uint16_t trig_pin[2] = { GPIO_Pin_10, GPIO_Pin_11 };
static const uint16_t echo_pin[2] = { GPIO_Pin_0, GPIO_Pin_1 };

static void EchoEdge(uint8_t sensor)
{
    uint8_t high;
    uint16_t counter;
    if (sensor != active_sensor || !initialized) return;
    if ((uint32_t)(Platform_GetMs() - started_ms) >= ECHO_TIMEOUT_MS) return;
    high = GPIO_ReadInputDataBit(GPIOB, echo_pin[sensor]);
    counter = TIM_GetCounter(TIM3);
    if (capture_state == CAP_RISE && high) {
        rise_us = counter;
        capture_state = CAP_FALL;
    } else if (capture_state == CAP_FALL && !high) {
        pulse_us = (uint16_t)(counter - rise_us); /* handles 16-bit wrap */
        completed_ms = Platform_GetMs();
        capture_state = CAP_DONE;
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        EchoEdge(0U);
    }
}

void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line1);
        EchoEdge(1U);
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        GPIO_ResetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    }
}

void Ultrasound_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef compare;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_hz;
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    initialized = 0U;
    capture_state = CAP_IDLE;
    active_sensor = next_sensor = have_finished = 0U;
    have_sample[0] = have_sample[1] = 0U;
    distance_cm[0] = distance_cm[1] = ULTRASOUND_DISTANCE_INVALID;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_DeInit(TIM3);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &gpio);

    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_hz *= 2U;
    if (timer_hz < 1000000U || timer_hz % 1000000U != 0U) {
        __set_PRIMASK(saved_irq);
        return;
    }
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000U - 1U);
    timer.TIM_Period = 65535U;
    TIM_TimeBaseInit(TIM3, &timer);
    TIM_OCStructInit(&compare);
    compare.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OC1Init(TIM3, &compare);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_Cmd(TIM3, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    EXTI_StructInit(&exti);
    exti.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1);

    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = TIM3_IRQn; NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI0_IRQn; NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel = EXTI1_IRQn; NVIC_Init(&nvic);
    initialized = 1U;
    __set_PRIMASK(saved_irq);
}

void Ultrasound_Update(void)
{
    uint32_t now = Platform_GetMs();
    uint32_t saved_irq;
    uint8_t sensor;
    if (!initialized) return;
    saved_irq = __get_PRIMASK();
    __disable_irq();
    if (capture_state != CAP_IDLE) {
        if (capture_state == CAP_DONE || (uint32_t)(now - started_ms) >= ECHO_TIMEOUT_MS) {
            uint8_t finished_state = capture_state;
            sensor = active_sensor;
            if (finished_state == CAP_DONE && pulse_us >= MIN_ECHO_US && pulse_us <= MAX_ECHO_US)
                distance_cm[sensor] = (float)pulse_us * 0.017f;
            else if (finished_state == CAP_RISE)
                /* The line stayed low for the full window: no target in range. */
                distance_cm[sensor] = ULTRASOUND_DISTANCE_NO_ECHO;
            else
                /* A rise without a fall, or an invalid pulse, is a signal fault. */
                distance_cm[sensor] = ULTRASOUND_DISTANCE_INVALID;
            sample_ms[sensor] = finished_state == CAP_DONE ? completed_ms : now;
            have_sample[sensor] = 1U;
            capture_state = CAP_IDLE;
            TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
            GPIO_ResetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
            next_sensor = (uint8_t)(sensor ^ 1U);
            finished_ms = now;
            have_finished = 1U;
        }
        __set_PRIMASK(saved_irq);
        return;
    }
    if (have_finished && (uint32_t)(now - finished_ms) < QUIET_MS) {
        __set_PRIMASK(saved_irq);
        return;
    }
    sensor = next_sensor;
    if (GPIO_ReadInputDataBit(GPIOB, echo_pin[sensor]) != 0U) {
        distance_cm[sensor] = ULTRASOUND_DISTANCE_INVALID;
        sample_ms[sensor] = now;
        have_sample[sensor] = 1U;
        next_sensor ^= 1U;
        finished_ms = now;
        have_finished = 1U;
        __set_PRIMASK(saved_irq);
        return;
    }
    active_sensor = sensor;
    started_ms = now;
    capture_state = CAP_RISE;
    EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1);
    GPIO_SetBits(GPIOB, trig_pin[sensor]);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_SetCompare1(TIM3, (uint16_t)(TIM_GetCounter(TIM3) + TRIGGER_US));
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    __set_PRIMASK(saved_irq);
}

static float ReadDistance(uint8_t sensor)
{
    if (!initialized || !have_sample[sensor] ||
        (uint32_t)(Platform_GetMs() - sample_ms[sensor]) > SAMPLE_STALE_MS)
        return ULTRASOUND_DISTANCE_INVALID;
    return distance_cm[sensor];
}

void Ultrasound_GetSample(UltrasoundSample *sample)
{
    if (!sample) return;
    sample->left_cm = ReadDistance(0U);
    sample->right_cm = ReadDistance(1U);
    sample->valid = (uint8_t)(sample->left_cm > 0.0f && sample->right_cm > 0.0f);
}

float Ultrasound_GetLeftDistance(void) { return ReadDistance(0U); }
float Ultrasound_GetRightDistance(void) { return ReadDistance(1U); }
void Ultrasound_GetDistances(float *left, float *right)
{
    if (left) *left = ReadDistance(0U);
    if (right) *right = ReadDistance(1U);
}
