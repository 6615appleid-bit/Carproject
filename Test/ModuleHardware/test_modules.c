#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "Track.h"
#include "Avoid.h"
#include "Ultrasound.h"
#include "CarControl.h"
#include "CarConfig.h"
#include "Motor.h"
#include "Platform.h"

GPIO_TypeDef module_gpio_a, module_gpio_b, module_gpio_c;
TIM_TypeDef module_tim4;
USART_TypeDef module_usart2;
static uint32_t now, primask;
static uint8_t black_mask;
static int16_t left_demand, right_demand;
static unsigned int writes, tx_count;
static uint8_t tx_enabled;
static char tx_bytes[256];
static UltrasoundSample sample;
void USART2_IRQHandler(void);
uint32_t Platform_GetMs(void) { return now; }
void Platform_Init(void) { now = 0; }
uint32_t Module_GetPrimask(void) { return primask; }
void Module_SetPrimask(uint32_t value) { primask = value; }
void Module_DisableIrq(void) { primask = 1; }
void Motor_Init(void) { left_demand = right_demand = 0; }
void Motor_Stop(void) { left_demand = right_demand = 0; ++writes; }
void Motor_SetDemand(int16_t left, int16_t right)
{
    assert(left >= 0 && left <= MOTOR_DEMAND_MAX);
    assert(right >= 0 && right <= MOTOR_DEMAND_MAX);
    left_demand = left; right_demand = right; ++writes;
}
void Ultrasound_Init(void) { sample.left_cm = sample.right_cm = 100.0f; sample.valid = 1; }
void Ultrasound_Update(void) { }
void Ultrasound_GetSample(UltrasoundSample *out) { *out = sample; }
void RCC_APB2PeriphClockCmd(uint32_t mask, FunctionalState state)
{ assert(mask == (RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO) && state == ENABLE); }
void RCC_APB1PeriphClockCmd(uint32_t mask, FunctionalState state)
{ assert(mask == RCC_APB1Periph_USART2 && state == ENABLE); }
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOA);
    if (gpio->GPIO_Pin == GPIO_Pin_2) assert(gpio->GPIO_Mode == GPIO_Mode_AF_PP);
    else assert(gpio->GPIO_Pin == GPIO_Pin_3 && gpio->GPIO_Mode == GPIO_Mode_IPU);
}
void GPIO_PinRemapConfig(uint32_t remap, FunctionalState state)
{ assert(remap == GPIO_Remap_USART2 && state == DISABLE); }
void USART_DeInit(USART_TypeDef *uart)
{ assert(uart == USART2); memset(uart, 0, sizeof(*uart)); tx_enabled = 0; tx_count = 0; }
void USART_StructInit(USART_InitTypeDef *uart) { memset(uart, 0, sizeof(*uart)); }
void USART_Init(USART_TypeDef *uart, USART_InitTypeDef *config)
{
    assert(uart == USART2 && config->USART_BaudRate == 115200);
    assert(config->USART_WordLength == USART_WordLength_8b && config->USART_StopBits == USART_StopBits_1);
    assert(config->USART_Parity == USART_Parity_No && config->USART_HardwareFlowControl == USART_HardwareFlowControl_None);
    assert(config->USART_Mode == (USART_Mode_Tx | USART_Mode_Rx));
}
void USART_Cmd(USART_TypeDef *uart, FunctionalState state) { assert(uart == USART2 && state == ENABLE); }
void USART_ITConfig(USART_TypeDef *uart, uint16_t interrupt, FunctionalState state)
{
    assert(uart == USART2);
    if (interrupt == USART_IT_TXE) tx_enabled = state == ENABLE;
    else assert(interrupt == USART_IT_RXNE || interrupt == USART_IT_ERR || interrupt == USART_IT_PE);
}
ITStatus USART_GetITStatus(USART_TypeDef *uart, uint16_t interrupt)
{ assert(uart == USART2 && interrupt == USART_IT_TXE); return tx_enabled ? SET : RESET; }
uint16_t USART_ReceiveData(USART_TypeDef *uart)
{
    assert(uart == USART2);
    uart->SR = 0;
    return (uint16_t)uart->DR;
}
void USART_SendData(USART_TypeDef *uart, uint16_t value)
{ assert(uart == USART2 && tx_count < sizeof(tx_bytes)); tx_bytes[tx_count++] = (char)value; }
void NVIC_Init(NVIC_InitTypeDef *nvic) { assert(nvic->NVIC_IRQChannel == USART2_IRQn && nvic->NVIC_IRQChannelCmd == ENABLE); }

static void Receive(const char *text)
{
    unsigned int before = writes;
    for (; *text; ++text) {
        USART2->DR = (uint8_t)*text;
        USART2->SR = USART_FLAG_RXNE;
        USART2_IRQHandler();
    }
    assert(writes == before); /* ISR must never move motors. */
}
static void Frame(uint8_t mask)
{
    char text[] = "$D,x1:1,x2:1,x3:1,x4:1,x5:1,x6:1,x7:1,x8:1#";
    unsigned int i;
    for (i = 0; i < 8; ++i) text[6 + 5 * i] = (char)('0' + ((mask & (1U << i)) ? TRACK_BLACK_LEVEL : !TRACK_BLACK_LEVEL));
    Receive(text);
}
static void DrainTx(void)
{
    unsigned int i;
    for (i = 0; i < 10 && tx_enabled; ++i) USART2_IRQHandler();
    assert(!tx_enabled);
}
static void Warmup(void)
{
    unsigned int before = writes;
    now += 20000; Track_SenseUpdate(); DrainTx();
    assert(tx_count == 7 && memcmp(tx_bytes, "$0,0,1#", 7) == 0 && writes == before);
}
static TrackStatus Sense(uint8_t mask)
{
    unsigned int before = writes;
    black_mask = mask; now += 10U; Frame(mask);
    Track_SenseUpdate();
    assert(writes == before);
    return Track_GetStatus();
}
static TrackStatus Stable(uint8_t mask) { Sense(mask); Sense(mask); return Sense(mask); }
static void AvoidStep(uint32_t ms, TrackStatus *line)
{ now += ms; Frame(0x18); Track_SenseUpdate(); Avoid_SenseUpdate(); Avoid_Update(line); }
static void ToSearch(TrackStatus *line)
{
    AvoidStep(10, line); assert(Avoid_GetStatus() == AVOID_BYPASSING);
    AvoidStep(200, line); assert(Avoid_GetStatus() == AVOID_BYPASSING);
    AvoidStep(450, line); assert(Avoid_GetStatus() == AVOID_BYPASSING);
    AvoidStep(400, line); assert(Avoid_GetStatus() == AVOID_SEARCHING);
}
static void CarStep(void) { now += 10; Frame(black_mask); CarControl_Update(); }

static int Base(void) { return (left_demand + right_demand) / 2; }
static int CorrectionMagnitude(void)
{
    int difference = left_demand - right_demand;
    return (difference < 0 ? -difference : difference) / 2;
}
static void Accelerate(void)
{
    unsigned int i;
    for (i = 0; i < 35; ++i) { Sense(0x18); Track_Update(); }
    assert(left_demand == 650 && right_demand == 650);
}
static void TestDynamicPd(void)
{
    unsigned int i, before;
    now = 0; Track_Init(); Warmup(); Stable(0x18);
    Track_Update(); assert(Base() == 300);
    for (i = 1; i <= 26; ++i) {
        Sense(0x18); Track_Update(); assert(Base() == 300 + (int)i * 13);
        before = writes; Track_Update(); assert(writes == before);
    }
    Sense(0x18); Track_Update(); assert(Base() == 650);
    before = writes; now += 10; Track_Update(); assert(writes == before);

    /* New frame inside the 10 ms gate is consumed only when control is due. */
    Track_Reset(); Track_Update(); assert(Base() == 300);
    now += 5; Frame(0x18); Track_SenseUpdate(); before = writes;
    Track_Update(); assert(writes == before);
    now += 5; Track_Update(); assert(Base() == 313);
    Accelerate();
    Sense(0x04); Track_Update(); assert(Base() == 517); /* |error|=429. */
    assert(CorrectionMagnitude() == 180); /* P + D at 10 ms. */
    Sense(0x04); Track_Update(); assert(Base() == 517);
    assert(CorrectionMagnitude() == 128); /* Settled P only. */
    Sense(0x02); Track_Update(); assert(Base() == 351); /* |error|=714. */
    Sense(0x01); Track_Update(); assert(Base() == 300 && CorrectionMagnitude() == 280);
    Sense(0x80); Track_Update(); assert(Base() == 300 && CorrectionMagnitude() == 280);

    /* Small position alone is insufficient: rapid changes block acceleration. */
    Sense(0x10); Track_Update(); assert(Base() == 300);
    Sense(0x10); Track_Update(); assert(Base() == 313); /* |error|=143, D=0. */
    Sense(0x08); Track_Update(); assert(Base() == 313); /* D magnitude=286. */
    Sense(0x18); Track_Update(); assert(Base() == 313); /* D magnitude=143. */
    Sense(0x18); Track_Update(); assert(Base() == 326);

    /* Same 143 change over 30 ms normalizes to 47: below the threshold 50. */
    now += 30; Frame(0x10); Track_SenseUpdate(); Track_Update();
    assert(Base() == 339 && CorrectionMagnitude() == 48);
    /* Over 20 ms it is 71: acceleration is blocked, with unchanged D scaling. */
    now += 20; Frame(0x18); Track_SenseUpdate(); Track_Update();
    assert(Base() == 339 && CorrectionMagnitude() == 8);
    Accelerate();
    Sense(0xff); Track_Update(); assert(Base() == 300 && CorrectionMagnitude() == 0);
    Sense(0x3f); Track_Update(); assert(Base() == 300 && CorrectionMagnitude() > 0);
    Accelerate();

    /* SenseUpdate must reset even if CarControl skips Track_Update on loss. */
    Sense(0x00); Stable(0x18); Track_Update(); assert(Base() == 300);
    Accelerate(); Sense(0x81); Stable(0x18); Track_Update(); assert(Base() == 300);
    Accelerate(); now += 151; before = writes; Track_SenseUpdate();
    assert(writes == before && Track_GetStatus().state == TRACK_INVALID);
    Stable(0x18); Track_Update(); assert(Base() == 300);
    Accelerate(); USART2->SR = USART_FLAG_ORE | USART_FLAG_RXNE; USART2_IRQHandler();
    Track_SenseUpdate(); Stable(0x18); Track_Update(); assert(Base() == 300);
    Accelerate(); Track_Reset(); Track_Update(); assert(Base() == 300);
    Accelerate(); now += 151; Track_Update(); assert(Base() == 0);
    Stable(0x18); Track_Update(); assert(Base() == 300);

    /* Fresh sensing during a control hiatus must not preserve acceleration. */
    Accelerate(); for (i = 0; i < 16; ++i) Sense(0x18);
    Track_Update(); assert(Base() == 300);
}

int main(void)
{
    TrackStatus line;
    unsigned int before, i;
    uint32_t id;
    TestDynamicPd();
    now = 0; Track_Init();
    Frame(0x18); Track_SenseUpdate(); assert(Track_GetStatus().state == TRACK_INVALID);
    now = 4999; Track_SenseUpdate(); assert(!tx_enabled && tx_count == 0);
    now = 0; Warmup();
    assert(Track_GetStatus().state == TRACK_INVALID);
    assert(Sense(0x18).state == TRACK_AMBIGUOUS);
    id = Track_GetSampleId();
    for (i = 0; i < 10; ++i) { now += 10; Track_SenseUpdate(); }
    assert(Track_GetStatus().state == TRACK_AMBIGUOUS && Track_GetSampleId() == id);
    assert(Sense(0x18).state == TRACK_AMBIGUOUS);
    assert(Sense(0x18).state == TRACK_NORMAL);
    assert(Track_GetStatus().error == 0);
    Track_Update(); assert(left_demand == right_demand && left_demand > 0);
    Track_Reset(); assert(Track_GetStatus().state == TRACK_NORMAL);
    line = Stable(0x02); Track_Update();
    assert(TRACK_REVERSE_ORDER ? line.error > 0 : line.error < 0);
    assert(TRACK_REVERSE_ORDER ? left_demand > right_demand : left_demand < right_demand);
    line = Stable(0x40); Track_Update();
    assert(TRACK_REVERSE_ORDER ? line.error < 0 : line.error > 0);
    assert(TRACK_REVERSE_ORDER ? left_demand < right_demand : left_demand > right_demand);
    before = writes; now += 10; Track_Update(); assert(writes == before);
    assert(Sense(0x81).state == TRACK_AMBIGUOUS);
    assert(Sense(0xff).state == TRACK_WIDE);
    assert(Sense(0x00).state == TRACK_LOST);
    Track_Update(); assert(left_demand == 0 && right_demand == 0);
    Stable(0x18);

    /* Reject wrong labels, lengths, values, analog messages and overflow. */
    id = Track_GetSampleId();
    Receive("noise$D,x1:0#$A,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#");
    Receive("$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x7:0#");
    Receive("$D,x1:0,x2:0,x3:0,x4:0,x5:2,x6:0,x7:0,x8:0#");
    Receive("$D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0!#");
    Receive("$xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx#");
    Track_SenseUpdate(); assert(Track_GetSampleId() == id);
    Receive("$D,x1:0"); now += 51; Receive(",x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#");
    Track_SenseUpdate(); assert(Track_GetSampleId() == id);
    Receive("$D,x1:0"); now += 10; Receive(",x2:0,x3:0"); now += 11;
    Receive(",x4:0,x5:0,x6:0,x7:0,x8:0#");
    Track_SenseUpdate(); assert(Track_GetSampleId() == id); /* Total frame age, not just inter-byte gap. */
    Receive("$D,broken"); Sense(0x18); assert(Track_GetSampleId() != id);
    now += 151; assert(Track_GetStatus().state == TRACK_INVALID);
    Track_Update(); assert(left_demand == 0 && right_demand == 0);
    assert(Sense(0x18).state == TRACK_AMBIGUOUS);
    Stable(0x18); Frame(0x18); now += 151; Track_SenseUpdate();
    assert(Track_GetStatus().state == TRACK_INVALID); /* Processing does not renew age. */
    Stable(0x18);
    USART2->SR = USART_FLAG_ORE | USART_FLAG_RXNE; USART2_IRQHandler();
    assert(Track_GetStatus().state == TRACK_INVALID);
    Track_SenseUpdate(); assert(Track_GetStatus().state == TRACK_INVALID);
    assert(Stable(0x18).state == TRACK_NORMAL);
    now += 1001; Track_SenseUpdate(); DrainTx(); assert(tx_count == 14);
    now += 10; Track_SenseUpdate(); DrainTx(); assert(tx_count == 14);
    now = UINT32_MAX - 20015U; Track_Init(); Warmup(); assert(Stable(0x18).state == TRACK_NORMAL);

    Avoid_Init(); before = writes; Avoid_SenseUpdate(); Avoid_Reset(); assert(writes == before);
    assert(Avoid_Start() == 0);
    sample.left_cm = sample.right_cm = ULTRASOUND_DISTANCE_NO_ECHO;
    Avoid_SenseUpdate(); assert(Avoid_Check() == AVOID_CHECK_CLEAR);
    sample.left_cm = 10; Avoid_SenseUpdate(); assert(Avoid_Check() == AVOID_CHECK_DETECTED);
    before = writes; assert(Avoid_Start()); assert(writes == before); assert(!Avoid_Start());
    line.state = TRACK_NORMAL; line.error = 0;
    AvoidStep(10, &line); assert(left_demand > right_demand);
    sample.left_cm = 17; Avoid_SenseUpdate(); assert(Avoid_Check() == AVOID_CHECK_DETECTED);
    sample.left_cm = 20; Avoid_SenseUpdate(); assert(Avoid_Check() == AVOID_CHECK_CLEAR);
    ToSearch(&line);
    /* Neither cached frames on later control ticks nor same-tick calls count. */
    for (i = 0; i < 8; ++i) { now += 10; Avoid_SenseUpdate(); Avoid_Update(&line); }
    assert(Avoid_GetStatus() == AVOID_SEARCHING);
    line.error = 600; for (i = 0; i < 6; ++i) AvoidStep(10, &line);
    assert(Avoid_GetStatus() == AVOID_SEARCHING);
    line.error = 0; AvoidStep(10, &line); before = writes;
    for (i = 0; i < 10; ++i) Avoid_Update(&line);
    assert(Avoid_GetStatus() == AVOID_SEARCHING && writes == before);
    line.state = TRACK_LOST; AvoidStep(10, &line); line.state = TRACK_NORMAL;
    for (i = 0; i < 4; ++i) { AvoidStep(10, &line); assert(Avoid_GetStatus() == AVOID_SEARCHING); }
    AvoidStep(10, &line); assert(Avoid_GetStatus() == AVOID_DONE);
    before = writes; Avoid_Update(&line); assert(writes == before && Avoid_GetStatus() == AVOID_DONE);
    assert(left_demand == 0 && right_demand == 0);
    Avoid_Reset(); assert(Avoid_GetStatus() == AVOID_IDLE);
    sample.right_cm = 10; Avoid_SenseUpdate(); assert(Avoid_Start());
    AvoidStep(10, &line); assert(left_demand < right_demand);
    sample.valid = 0; AvoidStep(10, &line); assert(Avoid_GetStatus() == AVOID_FAILED);
    Avoid_Init(); sample.left_cm = 10; Avoid_SenseUpdate(); assert(Avoid_Start());
    ToSearch(&line); line.state = TRACK_LOST; AvoidStep(4000, &line);
    assert(Avoid_GetStatus() == AVOID_FAILED);

    /* Actual CarControl + Track/Avoid, only peripherals and motors replaced. */
    now = UINT32_MAX - 20030U; black_mask = 0x18; CarControl_Init(); Warmup();
    for (i = 0; i < 3; ++i) CarStep();
    assert(CarControl_Start()); CarStep(); assert(CarControl_GetState() == CAR_TRACKING);
    for (i = 0; i < 35; ++i) CarStep();
    assert(Base() == 650);
    black_mask = 0; CarStep(); assert(Base() == CAR_TRACK_RECOVERY_DEMAND);
    assert(CarControl_GetState() == CAR_TRACKING);
    black_mask = 0x18;
    CarStep(); assert(Base() == CAR_TRACK_RECOVERY_DEMAND);
    CarStep(); assert(Base() == CAR_TRACK_RECOVERY_DEMAND);
    CarStep(); assert(Base() == 300);
    for (i = 0; i < 35; ++i) CarStep();
    assert(Base() == 650);
    sample.left_cm = 10; CarStep(); assert(CarControl_GetState() == CAR_AVOIDING);
    sample.left_cm = 100;
    for (i = 0; i < 200 && CarControl_GetState() == CAR_AVOIDING; ++i) CarStep();
    assert(CarControl_GetState() == CAR_TRACKING && left_demand == 0 && right_demand == 0);
    CarStep(); assert(left_demand == 300 && right_demand == 300);
    for (i = 0; i < 35; ++i) CarStep();
    assert(Base() == 650);
    CarControl_Stop(); assert(Base() == 0);
    assert(CarControl_Start()); CarStep(); assert(Base() == 300);
    now += 151; CarControl_Update();
    assert(CarControl_GetState() == CAR_TRACKING && left_demand == 0 && right_demand == 0);
    now += CAR_TRACK_LOSS_TOLERANCE_MS; CarControl_Update();
    assert(CarControl_GetState() == CAR_FAULT && left_demand == 0 && right_demand == 0);
    puts("PASS: USART2 setup/protocol/warmup/retry, malformed/stale/error frames, PD, fresh-frame DONE, CarControl handoff/fault.");
    return 0;
}
