/* Yahboom eight-probe module: USART2, PA2(TX)->RX, PA3(RX)<-TX, 115200 8N1.
 * Protocol, sampling, classification and PD stay encapsulated in this file.
 * Source: supplied module manual + STM32 serial example, not the ZE IO BSP.
 */
#include "Track.h"
#include "Heading.h"
#include "Platform.h"
#include "stm32f10x.h"

/* The module is only polled after TRACK_WARMUP_MS (5 s) from power-on.
 * Calibration remains a physical KEY operation at the actual mounting height
 * (the module manual asks for a longer warm-up before that KEY step; it is a
 * bench procedure, not this firmware's wait).
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
/* ★ 转向统一走「目标航向角 + 目标速度」，本文件不写电机。
 *
 * 偏差不再直接变成 PWM 差速，而是变成相对于「航向基准」的一个修正角：
 *     目标航向角 = 基准角 + 修正角
 * 基准角 = 线路居中时车头朝向（进循迹时取当时的 yaw），偏差通过 trim 慢速把
 * 弯道学进去；差速和转角由 Heading 的转向环（陀螺闭环）产生。
 */
/* 线偏差 → 航向修正角：每 1000 偏差单位对应多少度（error ±1000 为满幅）。
 * 取值依据：旧版本在 |error| = 1000 时给出 300/1000 占空比的差速，而本工程转向环
 * 是 HEADING_KP = 13.9 PWM/度，300 / 13.9 ≈ 22 度，所以默认 22 与原来的转向
 * 力度相当；实车觉得「拐不过来」就加大，「画龙」就减小。 */
#ifndef TRACK_STEER_KP_DEG
#define TRACK_STEER_KP_DEG 22
#endif
/* 偏差微分项，默认 0：航向环自带的角速度 D 项已经在做阻尼，不必重复。 */
#ifndef TRACK_STEER_KD_DEG
#define TRACK_STEER_KD_DEG 0
#endif
/* 修正角限幅：偏差满幅时也不允许车头偏离基准太多。 */
#ifndef TRACK_STEER_LIMIT_DEG
#define TRACK_STEER_LIMIT_DEG 45.0f
#endif
/* 航向基准跟随偏差的速率（度 / 每步）与单步限幅。
 * 直道上偏差归零后基准不再动 一 锁定航向；弯道上偏差持续存在，
 * 基准就匀速跟过去，这样不用靠一个永久存在的航向偏差来维持转弯。 */
#ifndef TRACK_HEADING_TRIM_GAIN_DEG
#define TRACK_HEADING_TRIM_GAIN_DEG 0.0015f
#endif
#ifndef TRACK_HEADING_TRIM_MAX_DEG
#define TRACK_HEADING_TRIM_MAX_DEG 0.5f
#endif
/* 基准角相对「进循迹时的朝向」最多偏离多少度，防卡住时慢性绕圈。 */
#ifndef TRACK_HEADING_TRIM_LIMIT_DEG
#define TRACK_HEADING_TRIM_LIMIT_DEG 120.0f
#endif
/* ★ 丢线搜索（2026-09-25）：只有「上一次有效检测已经严重偏离」时，丢线后才沿
 * 最后一次的修正方向继续转向找线 —— 车最后偏右（线在左侧，error < 0，刚才在
 * 左转）就继续左转，最后偏左就继续右转。
 * 判据是最后一次有效偏差的【大小】：|error| >= TRACK_SEARCH_MIN_ABS_ERROR 才算
 * 「严重偏离」。例如只剩最外侧一路 00000001（|error| = 1000）会继续找线；
 * 而 00000110 这类还没到极限的偏差，丢线就只停车、不找线。
 * 数据无效（没有新帧/接收错误）也不搜索：那时没有任何可信偏差。
 * 找到线（NORMAL/WIDE）立刻回到正常循迹；搜索超时只把目标速度置 0，
 * 是否停车由整车负责。0 = 关闭，恢复「丢线即停车」的旧行为。 */
#ifndef TRACK_SEARCH_ENABLE
#define TRACK_SEARCH_ENABLE 1
#endif
/* 「严重偏离」的门槛：最后一次有效偏差的绝对值达到这个值，丢线后才继续转向找线。
 * 默认 800 与 TRACK_SLOW_FULL_ERROR 一致 —— 偏差到 800 时目标速度已经压到最低
 * 25，再丢线就是「线已经快滑出探头」的情形。
 * 量级参考（单路 / 相邻两路的 |error|）：最外侧一路 = 1000、最外侧两路 = 857、
 * 第二路 = 714、第二+第三路 = 571。取 1000 = 只有最外侧一路才算极限偏离。
 * 0 = 任何非零偏差都算（最宽松）。 */
#ifndef TRACK_SEARCH_MIN_ABS_ERROR
#define TRACK_SEARCH_MIN_ABS_ERROR 800
#endif
#if (TRACK_SEARCH_MIN_ABS_ERROR < 0) || (TRACK_SEARCH_MIN_ABS_ERROR > 1000)
#error TRACK_SEARCH_MIN_ABS_ERROR must be within 0..1000
#endif
/* 搜索时的目标速度（计数 / 10ms）：比正常循迹慢，边走边找。 */
#ifndef TRACK_SEARCH_SPEED
#define TRACK_SEARCH_SPEED 15
#endif
/* 搜索时目标航向角的旋转速率（度 / 每步；10ms 一步，1.0 ≈ 100°/s）。 */
#ifndef TRACK_SEARCH_RATE_DEG
#define TRACK_SEARCH_RATE_DEG 1.0f
#endif
/* 单次搜索最多转多少度，防止原地无限绕圈。 */
#ifndef TRACK_SEARCH_MAX_DEG
#define TRACK_SEARCH_MAX_DEG 150.0f
#endif
/* 单次搜索最长时间：超过 1 秒还没找到线就停止搜索（只把目标速度置 0，
 * 随后由整车按丢线容忍窗口报故障）。必须小于 App/CarConfig.h 的
 * CAR_TRACK_LOSS_TOLERANCE_MS。 */
#ifndef TRACK_SEARCH_TIMEOUT_MS
#define TRACK_SEARCH_TIMEOUT_MS 1000U
#endif
/* 目标速度：左、右轮编码器计数之和 / 10ms。
 * 600/300 PWM 经实测直流增益（0.0835 计数/PWM）换算约 50/25 计数每 10ms。 */
#ifndef TRACK_SPEED_MAX
#define TRACK_SPEED_MAX 50            /* 直道（偏差小）的目标速度 */
#endif
#ifndef TRACK_SPEED_MIN
#define TRACK_SPEED_MIN 25            /* 偏差满幅的目标速度，也是复位后的起点 */
#endif
#ifndef TRACK_WIDE_SPEED_MAX
#define TRACK_WIDE_SPEED_MAX 25       /* 宽黑不允许用高速冲过 */
#endif
#define TRACK_SLOW_START_ERROR 200    /* Full target at or below this absolute error. */
#define TRACK_SLOW_FULL_ERROR 800   /* Minimum target at or above this error. */
#define TRACK_SPEED_ACCEL_STEP 1     /* 每个新样本最多增加的目标速度（计数/10ms） */
#define TRACK_ACCEL_DERIVATIVE_MAX 50 /* Absolute error change per normalized 10 ms. */
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
static uint8_t base_valid;      /* 0 = 还没取过航向基准 */
static float   base_origin_deg; /* 进循迹（或 Reset）时的车头朝向，用于限幅 */
static float   base_deg;        /* 航向基准：线路居中时车头朝向 */
static int16_t last_turn_error; /* 最后一次有效偏差（带符号）；0 = 丢线前没偏 */
/* 丢线搜索状态。它【不】被 ResetControl 清除：SenseUpdate 每丢一帧都会清一次
 * 控制历史，搜索必须跨帧累积；Track_Reset（整车停车/交接/故障）和 Track_Init
 * 才清它。 */
static uint8_t  search_active;     /* 1 = 本次丢线已经开过一次搜索 */
static uint8_t  search_exhausted;  /* 1 = 搜索已超时或转到限幅，本次丢线不再转 */
static uint32_t search_start_ms;   /* 搜索开始时刻 */
static uint32_t search_ms;         /* 上一次推进搜索角的时刻 */
static float    search_deg;        /* 搜索的目标航向角，逐步旋转 */
static float    search_origin_deg; /* 进入搜索时的车头朝向，用于转角限幅 */
static int8_t   search_dir;        /* +1 = 左转，-1 = 右转 */
static DriveCmd drive;          /* 本模块输出的转向命令，由整车交给 Heading */
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

/* 只清控制历史（航向基准、偏差微分、速度斜坡）。SenseUpdate 每丢一帧都会调用它，
 * 所以它【不能】清丢线搜索状态，否则搜索会在每一帧从头开始。 */
static void ResetControl(void)
{
    control_valid = 0U;
    previous_error = 0;
    /* 下一拍重新把当前车头朝向当作航向基准，避免用旧路段的朝向去拐新路段。 */
    base_valid = 0U;
    base_deg = 0.0f;
    base_origin_deg = 0.0f;
    drive.steer_deg = 0.0f;
    drive.speed_cmd = (int16_t)TRACK_SPEED_MIN;
    control_ms = control_sample_ms = control_sample_id = 0U;
}

/* 结束一次丢线搜索：线路回来、整车停车/交接/故障、初始化时调用。
 * last_turn_error 故意保留：它是「上一次有效检测偏多少」的记忆，只由有效帧
 * 刷新、只在 Track_Init 里清零。 */
static void ResetSearch(void)
{
    search_active = 0U;
    search_exhausted = 0U;
    search_start_ms = search_ms = 0U;
    search_deg = search_origin_deg = 0.0f;
    search_dir = 0;
}

void Track_Reset(void)
{
    /* Preserve stream, current sensing and confirmation during handoff. */
    ResetControl();
    ResetSearch();
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
    last_turn_error = 0;
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
        ResetControl(); /* CarControl can stop without calling Track_Update. */
        return;
    }
    if (sampled && (uint32_t)(now - sample_ms) > TRACK_STALE_MS) {
        status.state = TRACK_INVALID; status.error = 0; normal_samples = 0U;
        ResetControl();
    }
    if (frame_id == sample_id) return; /* Never count a cached frame again. */
    sample_id = frame_id;
    if ((uint32_t)(now - frame_ms) > TRACK_STALE_MS) return;
    Classify(levels, frame_ms);
    /* Reset control history only; sensing/classification, motor ownership and any
     * ongoing loss search stay intact. */
    if (status.state != TRACK_NORMAL && status.state != TRACK_WIDE) ResetControl();
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

/* 丢线搜索：只有「上一次有效检测已经严重偏离」时，才沿最后一次的修正方向继续
 * 旋转目标航向角（车最后偏右就继续左转，最后偏左就继续右转），并把目标速度设成
 * 搜索速度。
 * 返回 1 = 已经给出搜索命令；0 = 不搜索（调用方把目标速度置 0，由整车停车）。
 * 不搜索的情况：搜索被关闭 / 上一次偏差没到 TRACK_SEARCH_MIN_ABS_ERROR（轻微
 * 偏离，或者一直压中线）/ 本次丢线已经超时或转到限幅 /
 * 数据无效（TRACK_INVALID，没有任何可信偏差）。 */
static uint8_t SearchStep(TrackState state, uint32_t now)
{
#if TRACK_SEARCH_ENABLE
    float turned;
    int16_t magnitude;
    if (state != TRACK_LOST && state != TRACK_AMBIGUOUS) return 0U;
   
    /* 只有「上一次有效检测已经严重偏离」才继续找线；轻微偏差丢线宁可停车。 */
    magnitude = last_turn_error < 0 ? (int16_t)-last_turn_error : last_turn_error;
    if (magnitude < TRACK_SEARCH_MIN_ABS_ERROR) return 0U;
    if (!search_active) {
        /* 第一次丢线：从当前车头朝向开始，朝最后一次的修正方向继续转，
         * 而不是先拐回上一次的指令角。 */
        search_active = 1U;
        search_exhausted = 0U;
        search_start_ms = now;
        search_ms = now - TRACK_SAMPLE_MS; /* 本拍就先转一步 */
        search_dir = last_turn_error > 0 ? -1 : 1; /* error > 0 = 线在右 = 刚才右转 */
        search_deg = Heading_GetYaw();
        search_origin_deg = search_deg;
    }
    if (!search_exhausted) {
        if ((uint32_t)(now - search_start_ms) >= TRACK_SEARCH_TIMEOUT_MS) {
            search_exhausted = 1U; /* 超过 1 秒没找到线：停止搜索，交给整车报故障 */
        } else if ((uint32_t)(now - search_ms) >= TRACK_SAMPLE_MS) {
            search_ms = now;
            search_deg += (float)search_dir * TRACK_SEARCH_RATE_DEG;
            turned = search_dir > 0 ? search_deg - search_origin_deg
                                    : search_origin_deg - search_deg;
            if (turned >= TRACK_SEARCH_MAX_DEG) search_exhausted = 1U;
        }
    }
    if (search_exhausted) return 0U;
    drive.steer_deg = search_deg;
    drive.speed_cmd = (int16_t)TRACK_SEARCH_SPEED;
    return 1U;
#else
    (void)state; (void)now;
    return 0U;
#endif
}

void Track_Update(void)
{
    TrackStatus line = Track_GetStatus();
    uint32_t now = Platform_GetMs();
    uint32_t elapsed = sample_ms - control_sample_ms;
    int32_t derivative = 0, abs_error, abs_derivative, target;
    float steer, trim;
    /* 丢线 / 分离黑簇 / 数据无效：先试「沿最后一次的修正方向继续转」的搜索。
     * 搜索给不出命令时把目标速度置 0，是否停车由整车决定；
     * 本模块任何情况下都不写电机。 */
    if (line.state != TRACK_NORMAL && line.state != TRACK_WIDE) {
        if (SearchStep(line.state, now) == 0U) { drive.speed_cmd = 0; }
        return;
    }
    /* 线路回来了（宽黑单簇仍算线路）：结束本次搜索，并按「从下限重新起步」的
     * 语义回到循迹最低速度。 */
    if (search_active) {
        ResetSearch();
        drive.speed_cmd = (int16_t)TRACK_SPEED_MIN;
    }
    if (control_valid && (control_sample_id == sample_id || (uint32_t)(now - control_ms) < TRACK_SAMPLE_MS)) return;
    if (control_valid && elapsed > TRACK_STALE_MS) Track_Reset();
    if (control_valid && elapsed && elapsed <= TRACK_STALE_MS)
        derivative = ((int32_t)line.error - previous_error) * (int32_t)TRACK_SAMPLE_MS / (int32_t)elapsed;
    abs_error = line.error < 0 ? -(int32_t)line.error : line.error;
    abs_derivative = derivative < 0 ? -derivative : derivative;
    /* 记住最后一次有效偏差（含符号），供丢线搜索判断「上一次是不是严重偏离」：
     * 大小和 TRACK_SEARCH_MIN_ABS_ERROR 比，方向取符号。
     * 线居中（error == 0）会把它清成 0 —— 那是「丢线前没偏」，丢线就停车。 */
    last_turn_error = line.error;

    /*--- 1. 航向基准 ---------------------------------------------------------
     * 进循迹时把当前车头朝向当作基准，之后由偏差慢速修正，方向与修正角一致
     * （等效于线偏差的积分项）：
     * 直道上偏差归零，基准不再移动 → 锁定航向，不会来回画龙；
     * 弯道上偏差持续存在，基准就跟着弯一直转过去，于是「维持转弯」由基准旋转
     * 承担，而不是靠一个永久存在的航向偏差。限幅防止卡住时慢性绕圈。 */
    if (!base_valid) {
        base_deg = Heading_GetYaw();
        base_origin_deg = base_deg;
        base_valid = 1U;
    }
    trim = -(TRACK_HEADING_TRIM_GAIN_DEG * (float)line.error);
    if (trim > TRACK_HEADING_TRIM_MAX_DEG) trim = TRACK_HEADING_TRIM_MAX_DEG;
    if (trim < -TRACK_HEADING_TRIM_MAX_DEG) trim = -TRACK_HEADING_TRIM_MAX_DEG;
    base_deg += trim;
    if (base_deg > base_origin_deg + TRACK_HEADING_TRIM_LIMIT_DEG)
        base_deg = base_origin_deg + TRACK_HEADING_TRIM_LIMIT_DEG;
    if (base_deg < base_origin_deg - TRACK_HEADING_TRIM_LIMIT_DEG)
        base_deg = base_origin_deg - TRACK_HEADING_TRIM_LIMIT_DEG;

    /*--- 2. 线偏差 → 相对基准的航向修正角（度，正 = 左转）-------------------
     * error > 0 = 线在右侧 → 要往右拐 → 修正角取负。 */
    steer = -((float)TRACK_STEER_KP_DEG * (float)line.error +
              (float)TRACK_STEER_KD_DEG * (float)derivative) / 1000.0f;
    if (steer > TRACK_STEER_LIMIT_DEG) steer = TRACK_STEER_LIMIT_DEG;
    if (steer < -TRACK_STEER_LIMIT_DEG) steer = -TRACK_STEER_LIMIT_DEG;
    drive.steer_deg = base_deg + steer;

    /*--- 3. 速度规划：偏差越大目标速度越低 --------------------------------- */
    if (abs_error <= TRACK_SLOW_START_ERROR) target = TRACK_SPEED_MAX;
    else if (abs_error >= TRACK_SLOW_FULL_ERROR) target = TRACK_SPEED_MIN;
    else target = TRACK_SPEED_MAX -
        (TRACK_SPEED_MAX - TRACK_SPEED_MIN) *
        (abs_error - TRACK_SLOW_START_ERROR) / (TRACK_SLOW_FULL_ERROR - TRACK_SLOW_START_ERROR);
    if (line.state == TRACK_WIDE && target > TRACK_WIDE_SPEED_MAX)
        target = TRACK_WIDE_SPEED_MAX;

    if (target < drive.speed_cmd) drive.speed_cmd = (int16_t)target; /* 减速立即生效 */
    else if (control_valid && elapsed && elapsed <= TRACK_STALE_MS &&
             line.state == TRACK_NORMAL && abs_error <= TRACK_SLOW_START_ERROR &&
             abs_derivative <= TRACK_ACCEL_DERIVATIVE_MAX) {
        drive.speed_cmd = (int16_t)(drive.speed_cmd + TRACK_SPEED_ACCEL_STEP);
        if (drive.speed_cmd > target) drive.speed_cmd = (int16_t)target;
    }
    previous_error = line.error;
    control_ms = now; control_sample_ms = sample_ms; control_sample_id = sample_id;
    control_valid = 1U;
}

/* 读取本模块最新算出的转向命令；整车每步把它交给 Heading_Drive。 */
void Track_GetDrive(DriveCmd *out)
{
    if (out != 0) { *out = drive; }
}
