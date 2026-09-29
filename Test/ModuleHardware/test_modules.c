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
#include "Heading_Mock.h" /* 航向替身：循迹/避障的命令最终由它变成差速 */

GPIO_TypeDef module_gpio_a, module_gpio_b, module_gpio_c;
TIM_TypeDef module_tim3;
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
/* 现在唯一的写电机者是航向环（替身），它会输出 ±100 的固定差速，
 * 所以这里允许有符号需求；量程仍然是驱动层的 ±MOTOR_DEMAND_MAX。 */
void Motor_SetDemand(int16_t left, int16_t right)
{
    assert(left >= -MOTOR_DEMAND_MAX && left <= MOTOR_DEMAND_MAX);
    assert(right >= -MOTOR_DEMAND_MAX && right <= MOTOR_DEMAND_MAX);
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
static int Near(float a, float b)
{
    float d = a - b;
    return (d < 0.0f ? -d : d) < 1.0f; /* 度，留 1° 容差 */
}
static DriveCmd TrackCmd(void) { DriveCmd d; Track_GetDrive(&d); return d; }
static DriveCmd AvoidCmd(void) { DriveCmd d; Avoid_GetDrive(&d); return d; }
static int32_t TrackSpeed(void) { return TrackCmd().speed_cmd; }
static float   TrackSteer(void) { return TrackCmd().steer_deg; }

static void AvoidStep(uint32_t ms, TrackStatus *line)
{ now += ms; Frame(0x18); Track_SenseUpdate(); Avoid_SenseUpdate(); Avoid_Update(line); }
/* 推进到寻线阶段：序号0 转出 → 序号1 直行到里程 → 序号2 转回基准。
 * 用 mock_heading_yaw 模拟车真的转到了目标航向，用里程读数模拟前进。 */
static void ToSearch(TrackStatus *line)
{
    float target;
    mock_heading_yaw = 0.0f;
    mock_heading_odo_left = mock_heading_odo_right = 0;
    AvoidStep(10, line);                      /* 序号0：算出转出目标 */
    target = AvoidCmd().steer_deg;
    assert(target > 1.0f || target < -1.0f);
    mock_heading_yaw = target;               /* 车已经转到位 */
    AvoidStep(10, line);                      /* 序号0 → 序号1 */
    AvoidStep(10, line);
    assert(Near(AvoidCmd().steer_deg, 0.0f)); /* 直行段对着基准航向 */
    mock_heading_odo_left = 1000;            /* 直行里程到点 */
    AvoidStep(10, line);                      /* 序号1 → 序号2 */
    AvoidStep(10, line);
    assert(Near(AvoidCmd().steer_deg, 0.0f)); /* 转回基准 */
    mock_heading_yaw = 0.0f;                 /* 已经转回基准 */
    AvoidStep(10, line);                      /* 序号2 → 寻线 */
    assert(Avoid_GetStatus() == AVOID_SEARCHING);
}
static void CarStep(void) { now += 10; Frame(black_mask); CarControl_Update(); }

/* 取一个样本单独算出的命令：先清控制器历史，航向基准重新取当前 yaw。 */
static DriveCmd OneShot(uint8_t mask)
{
    Track_Reset();
    Sense(mask);
    Track_Update();
    return TrackCmd();
}
/* 线居中时目标速度会从下限（25）斜坡升到上限（50）。 */

static void TestDynamicSteering(void)
{
    unsigned int i, before;
    TrackStatus line;
    DriveCmd cmd;
    float first;
    now = 0; Track_Init(); Warmup(); Stable(0x18);
    /* 1. 复位后的起点：线居中 → 目标航向 = 基准 = 当前 yaw，速度在下限，
     *    而且循迹绝不写电机。 */
    mock_heading_yaw = 0.0f;
    before = writes;
    Track_Update();
    assert(writes == before);
    assert(TrackSpeed() == 25); /* TRACK_SPEED_MIN */
    assert(Near(TrackSteer(), 0.0f));

    /* 2. 速度斜坡：每个新样本只抬 1，封顶 50。 */
    for (i = 1; i <= 24; ++i) { Sense(0x18); Track_Update(); assert(TrackSpeed() == 25 + (int)i); }
    for (i = 0; i < 5; ++i) { Sense(0x18); Track_Update(); assert(TrackSpeed() == 50); }

    /* 3. 同一样本不会被重复消费：没有新帧时命令不变。 */
    cmd = TrackCmd();
    Track_Update();
    assert(TrackCmd().speed_cmd == cmd.speed_cmd && Near(TrackCmd().steer_deg, cmd.steer_deg));

    /* 4. 偏差 → 航向修正角：线偏向哪侧，目标航向就朝哪侧偏（正 = 左转）。
     *    TRACK_STEER_KP_DEG = 22 → |error| 714 / 1000 分别约 15.7° / 22°。 */
    mock_heading_yaw = 0.0f;
    cmd = OneShot(0x02); /* |error| = 714 */
    assert(TRACK_REVERSE_ORDER ? (cmd.steer_deg < -14.0f && cmd.steer_deg > -18.0f)
                               : (cmd.steer_deg > 14.0f && cmd.steer_deg < 18.0f));
    cmd = OneShot(0x40); /* 另一侧，符号相反 */
    assert(TRACK_REVERSE_ORDER ? (cmd.steer_deg > 14.0f && cmd.steer_deg < 18.0f)
                               : (cmd.steer_deg < -14.0f && cmd.steer_deg > -18.0f));
    cmd = OneShot(0x01); /* |error| = 1000 */
    assert(TRACK_REVERSE_ORDER ? (cmd.steer_deg < -20.0f && cmd.steer_deg > -24.0f)
                               : (cmd.steer_deg > 20.0f && cmd.steer_deg < 24.0f));
    cmd = OneShot(0x10); /* |error| = 143 */
    assert(TRACK_REVERSE_ORDER ? (cmd.steer_deg > 2.0f && cmd.steer_deg < 5.0f)
                               : (cmd.steer_deg < -2.0f && cmd.steer_deg > -5.0f));

    /* 5. 宽黑：仍按质心转向，但不允许用高速冲过去。 */
    cmd = OneShot(0xff);
    assert(Near(cmd.steer_deg, 0.0f) && cmd.speed_cmd <= 25); /* TRACK_WIDE_SPEED_MAX */

    /* 6. 丢线搜索：只有「上一次有效检测已经严重偏离」（|error| ≥
     *    TRACK_SEARCH_MIN_ABS_ERROR = 800）才沿最后一次的修正方向继续转向找线；
     *    轻微偏离或一直压中线时丢线就直接停。超过 1 秒没找到线也停止搜索，
     *    只把速度置 0；数据过期（TRACK_INVALID）不搜索，停车由整车负责。 */
    mock_heading_yaw = 0.0f;
    line = Stable(0x80); Track_Update();          /* 只剩最外侧一路：|error| = 1000 */
    cmd = OneShot(0x00);                          /* 丢线 */
    assert(cmd.speed_cmd == 15);                  /* TRACK_SEARCH_SPEED */
    assert(line.error < 0 ? cmd.steer_deg > 0.0f : cmd.steer_deg < 0.0f);
    line = Stable(0x01); Track_Update();          /* 另一侧极限，方向应该相反 */
    cmd = OneShot(0x00);
    assert(cmd.speed_cmd == 15);
    assert(line.error > 0 ? cmd.steer_deg < 0.0f : cmd.steer_deg > 0.0f);
    now += 1000; Sense(0x00); Track_Update();     /* TRACK_SEARCH_TIMEOUT_MS 到点 */
    assert(TrackSpeed() == 0);
    Stable(0x18); Track_Update();                 /* 线路回来：立刻恢复正常循迹 */
    assert(TrackSpeed() == 25);                   /* 从下限重新起步 */
    /* 轻微偏离（|error| = 714 < 800）后丢线：不搜索，直接停。 */
    line = Stable(0x40); Track_Update();
    assert(line.error != 0);
    assert((line.error < 0 ? -(int32_t)line.error : line.error) < 800);
    cmd = OneShot(0x00);
    assert(cmd.speed_cmd == 0);
    /* 线居中（error = 0）后丢线：同样不搜索。 */
    Stable(0x18); Track_Update();
    cmd = OneShot(0x00);
    assert(cmd.speed_cmd == 0);
    now += 151; Track_Update();                   /* 数据过期 */
    assert(TrackSpeed() == 0);

    /* 7. 弯道学习：偏差长期存在时航向基准会朝同一侧慢慢转过去，
     *    所以修正角不会一直挂在限幅上（等效线偏差积分项）。 */
    mock_heading_yaw = 0.0f;
    Stable(0x02);
    Track_Reset(); /* 基准重新取当前 yaw */
    Sense(0x02); Track_Update();
    first = TrackSteer();
    for (i = 0; i < 20; ++i) { Sense(0x02); Track_Update(); }
    assert(TRACK_REVERSE_ORDER ? TrackSteer() < first - 5.0f : TrackSteer() > first + 5.0f);
}

int main(void)
{
    TrackStatus line;
    unsigned int before, i;
    uint32_t id;
    TestDynamicSteering();
    /* 上电预热：预热窗口内不发模式请求。当前固件 TRACK_WARMUP_MS = 5000 ms（5 秒）。 */
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
    /* 线居中：目标航向 = 航向基准；循迹只输出命令，不写电机。 */
    mock_heading_yaw = 0.0f;
    before = writes; Track_Update(); assert(writes == before);
    assert(Near(TrackSteer(), 0.0f) && TrackSpeed() > 0);
    Track_Reset(); assert(Track_GetStatus().state == TRACK_NORMAL);
    line = Stable(0x02); Track_Update();
    assert(TRACK_REVERSE_ORDER ? line.error > 0 : line.error < 0);
    assert(TRACK_REVERSE_ORDER ? TrackSteer() < 0.0f : TrackSteer() > 0.0f);
    line = Stable(0x40); Track_Update();
    assert(TRACK_REVERSE_ORDER ? line.error < 0 : line.error > 0);
    assert(TRACK_REVERSE_ORDER ? TrackSteer() > 0.0f : TrackSteer() < 0.0f);
    before = writes; now += 10; Track_Update(); assert(writes == before);
    assert(Sense(0x81).state == TRACK_AMBIGUOUS);
    assert(Sense(0xff).state == TRACK_WIDE);
    assert(Sense(0x00).state == TRACK_LOST);
    /* 轻微偏离后丢线（上一次 |error| = 714 < TRACK_SEARCH_MIN_ABS_ERROR）：不搜索。 */
    Track_Update();
    assert(TrackSpeed() == 0);
    assert(writes == before);
    /* 严重偏离后丢线（上一次 |error| = 1000）：沿最后一次的修正方向继续转。 */
    line = Stable(0x80); Track_Update();
    assert(Sense(0x00).state == TRACK_LOST);
    Track_Update();
    assert(TrackSpeed() == 15);               /* TRACK_SEARCH_SPEED */
    assert(line.error < 0 ? TrackSteer() > 0.0f : TrackSteer() < 0.0f);
    assert(writes == before);                 /* 搜索命令不写电机，电机归航向环 */
    /* 数据过期（TRACK_INVALID）不搜索：只把目标速度置 0，停车由整车负责。 */
    now += 151; Track_Update();
    assert(TrackSpeed() == 0);
    assert(writes == before);
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
    /* 左侧有障碍 → 向右让：目标航向 = 基准 - 45°，基准 = 进入避障时的航向。 */
    mock_heading_yaw = 0.0f;
    mock_heading_odo_left = mock_heading_odo_right = 0;
    before = writes; assert(Avoid_Start()); assert(writes == before); assert(!Avoid_Start());
    line.state = TRACK_NORMAL; line.error = 0;
    AvoidStep(10, &line);
    assert(Near(AvoidCmd().steer_deg, -45.0f)); /* AVOID_TURN_DEG */
    assert(AvoidCmd().speed_cmd == 10);         /* AVOID_TURN_SPEED */
    assert(writes == before);                   /* 避障也不写电机 */
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
    assert(AvoidCmd().speed_cmd == 0); /* DONE 时命令已经停住 */
    before = writes; Avoid_Update(&line);
    assert(writes == before && Avoid_GetStatus() == AVOID_DONE);
    Avoid_Reset(); assert(Avoid_GetStatus() == AVOID_IDLE);
    /* 右侧有障碍 → 向左让。 */
    mock_heading_yaw = 0.0f;
    sample.right_cm = 10; Avoid_SenseUpdate(); assert(Avoid_Start());
    AvoidStep(10, &line); assert(Near(AvoidCmd().steer_deg, 45.0f));
    sample.valid = 0; AvoidStep(10, &line); assert(Avoid_GetStatus() == AVOID_FAILED);
    Avoid_Init(); sample.left_cm = 10; Avoid_SenseUpdate(); assert(Avoid_Start());
    ToSearch(&line); line.state = TRACK_LOST; AvoidStep(4000, &line);
    assert(Avoid_GetStatus() == AVOID_FAILED);

    /* Actual CarControl + Track/Avoid, only peripherals, heading and motors replaced. */
    now = UINT32_MAX - 20030U; black_mask = 0x18; CarControl_Init(); Warmup();
    /* 循迹/避障都要经过航向环：替身必须就绪，并允许它写电机。 */
    mock_heading_ready = 1U;
    mock_heading_drive = 1U;
    mock_heading_yaw = 0.0f;
    mock_heading_odo_left = mock_heading_odo_right = 0;
    for (i = 0; i < 3; ++i) CarStep();
    assert(CarControl_Start()); CarStep(); assert(CarControl_GetState() == CAR_TRACKING);
    for (i = 0; i < 35; ++i) CarStep();
    assert(TrackSpeed() == 50);                   /* 速度斜坡到顶 */
    assert(mock_heading_target == TrackSteer());  /* 命令原样转交给航向环 */
    assert(mock_heading_speed == TrackSpeed());
    assert(left_demand == -100 && right_demand == 100); /* 航向环是唯一写电机者 */
    /* 先让线严重偏离（只剩最外侧一路，|error| = 1000），让循迹记住这次偏离，再
     * 丢线：车会继续转向找线，不再原地停车（1 秒内找到线就回到正常循迹）。 */
    black_mask = 0x80; CarStep(); CarStep(); CarStep();
    assert(CarControl_GetState() == CAR_TRACKING);
    black_mask = 0; CarStep();
    assert(CarControl_GetState() == CAR_TRACKING);        /* 丢线在容忍窗口内 */
    assert(TrackSpeed() == 15);                           /* TRACK_SEARCH_SPEED */
    assert(left_demand == -100 && right_demand == 100);   /* 航向环仍在驱动车轮 */
    black_mask = 0x18; CarStep(); CarStep(); CarStep();
    assert(TrackSpeed() == 25);                   /* 重新拿到稳定线后从下限起步 */
    for (i = 0; i < 35; ++i) CarStep();
    assert(TrackSpeed() == 50);
    /* 触发避障：车按「目标航向角 + 编码器里程」走完绕障，再交回循迹。 */
    sample.left_cm = 10; CarStep(); assert(CarControl_GetState() == CAR_AVOIDING);
    sample.left_cm = 100;
    for (i = 0; i < 200 && CarControl_GetState() == CAR_AVOIDING; ++i) {
        mock_heading_yaw = AvoidCmd().steer_deg; /* 模拟航向环已经把车转到目标角 */
        mock_heading_odo_left += 100;            /* 模拟一直在前进 */
        CarStep();
    }
    assert(CarControl_GetState() == CAR_TRACKING && left_demand == 0 && right_demand == 0);
    CarStep(); assert(TrackSpeed() == 25);
    for (i = 0; i < 35; ++i) CarStep();
    assert(TrackSpeed() == 50);
    CarControl_Stop(); assert(left_demand == 0 && right_demand == 0);
    assert(CarControl_Start()); CarStep(); assert(TrackSpeed() == 25);
    now += 151; CarControl_Update();
    assert(CarControl_GetState() == CAR_TRACKING && left_demand == 0 && right_demand == 0);
    now += CAR_TRACK_LOSS_TOLERANCE_MS; CarControl_Update();
    assert(CarControl_GetState() == CAR_FAULT && left_demand == 0 && right_demand == 0);
    puts("PASS: USART2 setup/protocol/warmup/retry, malformed/stale/error frames,");
    puts("      line error to target heading, fresh-frame DONE, CarControl handoff/fault.");
    return 0;
}
