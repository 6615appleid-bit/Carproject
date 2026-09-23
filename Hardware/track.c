/* Yahboom eight-probe module: USART2, PA2(TX)->RX, PA3(RX)<-TX, 115200 8N1.
 * Protocol, sampling, classification and PD stay encapsulated in this file.
 * Source: supplied module manual + STM32 serial example, not the ZE IO BSP.
 */
#include "Track.h"
#include "Motor.h"
#include "Platform.h"
#include "stm32f10x.h"

/* Every power-on requires >=20 s stabilization per the module manual.
 * Calibration remains a physical KEY operation at the actual mounting height.
 * This car steers away from the line with order 0: keep TRACK_REVERSE_ORDER 1.
 */
#define TRACK_WARMUP_MS 5000U
#define TRACK_RETRY_MS 1000U
#define TRACK_FRAME_TIMEOUT_MS 20U /* 43 bytes take about 3.7 ms at 115200. */
#define TRACK_STALE_MS 150U
#define TRACK_SAMPLE_MS 10U
#define TRACK_NORMAL_SAMPLES 3U
#define TRACK_FRAME_LENGTH 43U
#ifndef TRACK_REVERSE_ORDER
#define TRACK_REVERSE_ORDER 1
#endif
#ifndef TRACK_BLACK_LEVEL
#define TRACK_BLACK_LEVEL 0U
#endif
/* Open-loop PWM demand: 1000 = 100% duty, not measured wheel speed. */
#define TRACK_STRAIGHT_MAX_DEMAND 650 /* Straight-line base ceiling: first trial 60%. */
#define TRACK_MIN_DEMAND 300          /* Base at large error and after reset. */
#define TRACK_WIDE_MAX_DEMAND 300     /* Wide black must not trigger high speed. */
#define TRACK_SLOW_START_ERROR 200     /* Full target at or below this absolute error. */
#define TRACK_SLOW_FULL_ERROR 800   /* Minimum target at or above this error. */
#define TRACK_ACCEL_STEP 13          /* Maximum base increase per new control sample. */
#define TRACK_ACCEL_DERIVATIVE_MAX 50 /* Absolute error change per normalized 10 ms. */
#define TRACK_KP 300
#define TRACK_KD 120
#define TRACK_MAX_CORRECTION 280
#if (TRACK_REVERSE_ORDER != 0 && TRACK_REVERSE_ORDER != 1) || (TRACK_BLACK_LEVEL != 0 && TRACK_BLACK_LEVEL != 1)
#error Invalid track mounting or black-level configuration
#endif

static const char stream_command[] = "$0,0,1#";
static char rx_frame[TRACK_FRAME_LENGTH];
static uint8_t rx_length;
static uint32_t rx_start_ms;
static volatile uint8_t warmed, received_levels, tx_index;
static volatile uint32_t received_id, received_ms, rx_errors;
static uint32_t consumed_errors, sample_id, sample_ms, init_ms, command_ms, normal_ms;
static uint32_t control_ms, control_sample_ms, control_sample_id;
static uint8_t initialized, sampled, requested, normal_samples, control_valid;
static int16_t previous_error;
static int32_t base_demand;
static TrackStatus status;

/* IRQ-only bounded parser. Validate every label, separator, bit and length.
 * No checksum exists in this protocol; complete-frame syntax is all it offers.
 */
static void ReceiveByte(uint8_t value)
{
    unsigned int i;
    uint8_t levels = 0U;
    uint32_t now = Platform_GetMs();
    if (!warmed) { rx_length = 0U; return; }
    if (rx_length && (uint32_t)(now - rx_start_ms) > TRACK_FRAME_TIMEOUT_MS) rx_length = 0U;
    if (value == '$') { rx_frame[0] = '$'; rx_length = 1U; rx_start_ms = now; return; }
    if (!rx_length) return;
    rx_frame[rx_length++] = (char)value;
    if (value != '#' && rx_length < TRACK_FRAME_LENGTH) return;
    if (value == '#' && rx_length == TRACK_FRAME_LENGTH && rx_frame[1] == 'D') {
        for (i = 0U; i < 8U; ++i) {
            unsigned int offset = 2U + i * 5U;
            if (rx_frame[offset] != ',' || rx_frame[offset + 1U] != 'x' ||
                rx_frame[offset + 2U] != (char)('1' + i) || rx_frame[offset + 3U] != ':' ||
                (rx_frame[offset + 4U] != '0' && rx_frame[offset + 4U] != '1')) break;
            if (rx_frame[offset + 4U] == '1') levels |= (uint8_t)(1U << i);
        }
        if (i == 8U) {
            received_levels = levels;
            received_ms = now;
            ++received_id; /* Only new, complete valid frames get a new ID. */
        }
    }
    rx_length = 0U;
}

void USART2_IRQHandler(void)
{
    /* Reading SR then DR clears RXNE and ORE/FE/NE/PE on STM32F1. */
    uint16_t flags = (uint16_t)USART2->SR;
    if (flags & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) {
        uint8_t value = (uint8_t)USART_ReceiveData(USART2);
        if (flags & (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) {
            rx_length = 0U;
            ++rx_errors;
        } else ReceiveByte(value);
    }
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if (tx_index < sizeof(stream_command) - 1U)
            USART_SendData(USART2, (uint16_t)stream_command[tx_index++]);
        if (tx_index >= sizeof(stream_command) - 1U) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}

void Track_Reset(void)
{
    /* Preserve stream, current sensing and confirmation during handoff. */
    control_valid = 0U;
    previous_error = 0;
    base_demand = TRACK_MIN_DEMAND;
    control_ms = control_sample_ms = control_sample_id = 0U;
}

void Track_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    initialized = warmed = sampled = requested = normal_samples = rx_length = 0U;
    received_id = received_ms = rx_errors = consumed_errors = sample_id = 0U;
    init_ms = Platform_GetMs();
    tx_index = sizeof(stream_command) - 1U;
    status.state = TRACK_INVALID;
    status.error = 0;
    Track_Reset();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_DeInit(USART2);
    GPIO_PinRemapConfig(GPIO_Remap_USART2, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200U;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
    USART_ITConfig(USART2, USART_IT_PE, ENABLE);
    USART_Cmd(USART2, ENABLE);
    initialized = 1U;
    __set_PRIMASK(saved_irq);
}

static void RequestStream(uint32_t now)
{
    uint32_t saved_irq;
    if (requested && (uint32_t)(now - command_ms) < TRACK_RETRY_MS) return;
    if (sampled && (uint32_t)(now - sample_ms) <= TRACK_STALE_MS) return;
    saved_irq = __get_PRIMASK();
    __disable_irq();
    if (tx_index >= sizeof(stream_command) - 1U) {
        tx_index = 0U;
        command_ms = now;
        requested = 1U;
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    }
    __set_PRIMASK(saved_irq);
}

static void Classify(uint8_t levels, uint32_t frame_ms)
{
    unsigned int i, count = 0U, groups = 0U;
    uint8_t previous_black = 0U;
    int32_t sum = 0;
    /* Symmetric weights avoid the old integer-division centre bias. */
    static const int16_t weights[8] = {-1000,-714,-429,-143,143,429,714,1000};
    for (i = 0U; i < 8U; ++i) {
        unsigned int probe = TRACK_REVERSE_ORDER ? 7U - i : i;
        uint8_t black = (uint8_t)(((levels >> probe) & 1U) == TRACK_BLACK_LEVEL);
        if (black) {
            ++count;
            if (!previous_black) ++groups;
            sum += weights[i];
        }
        previous_black = black;
    }
    status.error = 0;
    if (!count) status.state = TRACK_LOST;
    else if (count >= 6U) {
        /* One wide black group is still one line. Report its centroid so the
         * controller can steer through crossings and wide curves instead of
         * treating the pattern as unusable. */
        status.state = TRACK_WIDE;
        status.error = groups == 1U ? (int16_t)(sum / (int32_t)count) : 0;
    }
    else if (groups != 1U) status.state = TRACK_AMBIGUOUS;
    else {
        if (!normal_samples || (uint32_t)(frame_ms - normal_ms) >= TRACK_SAMPLE_MS) {
            if (normal_samples < TRACK_NORMAL_SAMPLES) ++normal_samples;
            normal_ms = frame_ms;
        }
        status.state = normal_samples >= TRACK_NORMAL_SAMPLES ? TRACK_NORMAL : TRACK_AMBIGUOUS;
        status.error = (int16_t)(sum / (int32_t)count);
        return;
    }
    normal_samples = 0U;
}

void Track_SenseUpdate(void)
{
    uint32_t now = Platform_GetMs(), frame_id, frame_ms, errors, saved_irq;
    uint8_t levels;
    if (!initialized) return;
    if (!warmed) {
        if ((uint32_t)(now - init_ms) < TRACK_WARMUP_MS) return;
        warmed = 1U;
    }
    RequestStream(now);
    saved_irq = __get_PRIMASK();
    __disable_irq();
    frame_id = received_id; frame_ms = received_ms;
    levels = received_levels; errors = rx_errors;
    __set_PRIMASK(saved_irq);
    now = Platform_GetMs(); /* The IRQ snapshot can be newer than entry time. */
    if (errors != consumed_errors) {
        consumed_errors = errors;
        sample_id = frame_id; /* Discard mailbox associated with a receive error. */
        status.state = TRACK_INVALID; status.error = 0;
        normal_samples = 0U;
        Track_Reset(); /* CarControl can stop without calling Track_Update. */
        return;
    }
    if (sampled && (uint32_t)(now - sample_ms) > TRACK_STALE_MS) {
        status.state = TRACK_INVALID; status.error = 0; normal_samples = 0U;
        Track_Reset();
    }
    if (frame_id == sample_id) return; /* Never count a cached frame again. */
    sample_id = frame_id;
    if ((uint32_t)(now - frame_ms) > TRACK_STALE_MS) return;
    Classify(levels, frame_ms);
    /* Reset control history only; sensing/classification and motor ownership stay intact. */
    if (status.state != TRACK_NORMAL && status.state != TRACK_WIDE) Track_Reset();
    sample_ms = frame_ms; /* Receive time, not processing time. */
    sampled = 1U;
}

TrackStatus Track_GetStatus(void)
{
    TrackStatus result = status;
    if (!initialized || !warmed || !sampled || rx_errors != consumed_errors ||
        (uint32_t)(Platform_GetMs() - sample_ms) > TRACK_STALE_MS) {
        result.state = TRACK_INVALID; result.error = 0;
    }
    return result;
}

uint32_t Track_GetSampleId(void) { return sample_id; }

void Track_Update(void)
{
    TrackStatus line = Track_GetStatus();
    uint32_t now = Platform_GetMs();
    uint32_t elapsed = sample_ms - control_sample_ms;
    int32_t derivative = 0, correction, abs_error, abs_derivative, target, headroom;
    /* A single wide black group is still a line: keep steering from its centroid. */
    if (line.state != TRACK_NORMAL && line.state != TRACK_WIDE) { Motor_Stop(); Track_Reset(); return; }
    if (control_valid && (control_sample_id == sample_id || (uint32_t)(now - control_ms) < TRACK_SAMPLE_MS)) return;
    if (control_valid && elapsed > TRACK_STALE_MS) Track_Reset();
    if (control_valid && elapsed && elapsed <= TRACK_STALE_MS)
        derivative = ((int32_t)line.error - previous_error) * (int32_t)TRACK_SAMPLE_MS / (int32_t)elapsed;
    correction = ((int32_t)TRACK_KP * line.error + TRACK_KD * derivative) / 1000;
    if (correction > TRACK_MAX_CORRECTION) correction = TRACK_MAX_CORRECTION;
    if (correction < -TRACK_MAX_CORRECTION) correction = -TRACK_MAX_CORRECTION;
    abs_error = line.error < 0 ? -(int32_t)line.error : line.error;
    abs_derivative = derivative < 0 ? -derivative : derivative;
    if (abs_error <= TRACK_SLOW_START_ERROR) target = TRACK_STRAIGHT_MAX_DEMAND;
    else if (abs_error >= TRACK_SLOW_FULL_ERROR) target = TRACK_MIN_DEMAND;
    else target = TRACK_STRAIGHT_MAX_DEMAND -
        (TRACK_STRAIGHT_MAX_DEMAND - TRACK_MIN_DEMAND) *
        (abs_error - TRACK_SLOW_START_ERROR) / (TRACK_SLOW_FULL_ERROR - TRACK_SLOW_START_ERROR);
    if (line.state == TRACK_WIDE && target > TRACK_WIDE_MAX_DEMAND)
        target = TRACK_WIDE_MAX_DEMAND;

    if (target < base_demand) base_demand = target; /* Reduce PWM immediately. */
    else if (control_valid && elapsed && elapsed <= TRACK_STALE_MS &&
             line.state == TRACK_NORMAL && abs_error <= TRACK_SLOW_START_ERROR &&
             abs_derivative <= TRACK_ACCEL_DERIVATIVE_MAX) {
        base_demand += TRACK_ACCEL_STEP;
        if (base_demand > target) base_demand = target;
    }
    /* Keep the requested differential instead of clipping the outer wheel alone.
     * Store the applied base so subsequent acceleration is still bounded. */
    headroom = MOTOR_DEMAND_MAX - (correction < 0 ? -correction : correction);
    if (base_demand > headroom) base_demand = headroom;
    Motor_SetDemand((int16_t)(base_demand + correction), (int16_t)(base_demand - correction));
    previous_error = line.error;
    control_ms = now; control_sample_ms = sample_ms; control_sample_id = sample_id;
    control_valid = 1U;
}
