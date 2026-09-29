/* 正交编码器驱动，标准外设库版本。
 *
 *   左：TIM2 CH1/CH2 = PA0 / PA1
 *   右：TIM4 CH1/CH2 = PB6 / PB7
 *
 * 配置逐项对齐 stm32test 的 encoder.c：编码器模式 TI12、双边沿等效 4 倍频、
 * IC 滤波 10、ARR 65535。区别只在于那边用 HAL 的 TIM_Encoder_Init，
 * 这里用标准外设库的 TIM_EncoderInterfaceConfig。
 */
#include "Encoder.h"
#include "stm32f10x.h"

#define ENC_FILTER  10U         /* 与 stm32test 一致，抑制电机换向噪声 */
#define ENC_PERIOD  65535U      /* 16 位满量程，靠读后清零取增量        */

static const int8_t enc_sign[2] = { ENCODER_SIGN_LEFT, ENCODER_SIGN_RIGHT };
static int32_t      enc_total[2];

static void Encoder_GpioInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* 四路都取上拉输入：编码器输出大多开漏或需要上拉，内部上拉可以少接
     * 外部电阻；即使编码器是推挽输出，上拉也不会影响电平（与 stm32test 一致）。*/
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;        /* 左：TIM2_CH1/CH2 */
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;        /* 右：TIM4_CH1/CH2 */
    GPIO_Init(GPIOB, &gpio);
}

static void Encoder_TimInit(TIM_TypeDef *tim, uint8_t ch)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_ICInitTypeDef       ic;

    if (tim == TIM2) { RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); }
    else             { RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); }

    TIM_DeInit(tim);

    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler     = 0U;
    base.TIM_Period        = ENC_PERIOD;
    base.TIM_ClockDivision = TIM_CKD_DIV1;
    base.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &base);

    /* 两路输入捕获通道都配成直接映射、上升沿、不分频、滤波 10 */
    TIM_ICStructInit(&ic);
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = ENC_FILTER;

    ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(tim, &ic);
    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(tim, &ic);

    /* 编码器模式 3 = TI12：A、B 相上升沿都计数，等效 4 倍频。
     * 极性取上升沿，A/B 相序决定的计数方向由 ENCODER_SIGN_* 统一修正。 */
    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetCounter(tim, 0U);
    TIM_Cmd(tim, ENABLE);

    enc_total[ch] = 0;
}

void Encoder_Init(void)
{
    Encoder_GpioInit();
    Encoder_TimInit(TIM2, ENC_CH_LEFT);
    Encoder_TimInit(TIM4, ENC_CH_RIGHT);
}

int16_t Encoder_GetCount(uint8_t ch)
{
    TIM_TypeDef *tim = (ch == ENC_CH_LEFT) ? TIM2 : TIM4;
    int32_t      delta;

    /* 标准外设库的 TIM_GetCounter 直接传 TIM_TypeDef*，
     * 不像 HAL 的 __HAL_TIM_GET_COUNTER 要传句柄。
     * (int16_t) 转换天然处理了计数器回绕：只要一周期的增量远小于 32768，
     * 结果就是正确的有符号增量。 */
    delta = (int32_t)(int16_t)TIM_GetCounter(tim) * (int32_t)enc_sign[ch];
    TIM_SetCounter(tim, 0U);

    enc_total[ch] += delta;
    return (int16_t)delta;
}

int32_t Encoder_GetTotal(uint8_t ch)
{
    return enc_total[ch];
}

void Encoder_ResetTotal(void)
{
    enc_total[ENC_CH_LEFT]  = 0;
    enc_total[ENC_CH_RIGHT] = 0;
}
