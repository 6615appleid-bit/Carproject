/* MPU6050 驱动本体：标准外设库 + 软件（位翻转）I2C。
 *
 * 由 stm32test/src/softiic.c + mpu6050.c 移植，算法保持一致，改动只有：
 *   1. HAL → 标准外设库（GPIO_Init / RCC_APB2PeriphClockCmd / ...）。
 *   2. 去掉「总线对象」抽象，只留 MPU6050 一条总线，省掉传参开销。
 *   3. 去掉 EXTI / INT 引脚那一整套：本工程在主循环里轮询，
 *      见 Imu.h 顶部的说明。
 *   4. delay_us 的 DWT 实现换成 NOP 循环。stm32test 自己的 control.c 里
 *      记录过本板 DWT->CYCCNT 会偶发「倒退」，所以不引入这个依赖。
 *   5. 毫秒延时用 Platform_GetMs() 轮询，不再新增 Platform 接口。
 *
 * ★ 保留的关键设计（都是 stm32test 实测踩出来的，不要改）：
 *   - SCL/SDA 用【推挽输出】而不是开漏。开漏靠 30~50k 内部上拉，配上
 *     杜邦线几十 pF 的电容，上升时间约 2.4 us，超过位周期，SDA 根本来不及
 *     变高，从机收不到有效起始条件和地址位，表现是「完全没有 ACK」。
 *   - PB3/PB4 必须先关 JTAG（见 Imu_Init 开头）。
 */
#include "Imu.h"
#include "Platform.h"
#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 寄存器地址
 *--------------------------------------------------------------------------*/
#define MPU_SMPLRT_DIV      0x19U
#define MPU_CONFIG          0x1AU
#define MPU_GYRO_CONFIG     0x1BU
#define MPU_ACCEL_CONFIG    0x1CU
#define MPU_INT_ENABLE      0x38U
#define MPU_ACCEL_XOUT_H    0x3BU
#define MPU_GYRO_XOUT_H     0x43U
#define MPU_GYRO_ZOUT_H     0x45U
#define MPU_USER_CTRL       0x6AU
#define MPU_PWR_MGMT_1      0x6BU
#define MPU_PWR_MGMT_2      0x6CU
#define MPU_FIFO_EN         0x23U
#define MPU_WHO_AM_I        0x75U

/*----------------------------------------------------------------------------
 * 软件 I2C 引脚与位操作
 *
 * 只支持一条总线，所以引脚直接写死，省掉总线对象。
 * 全部用寄存器操作而不是 GPIO_Init：读一个字节要切换两次方向、翻几百次
 * 电平，用库函数开销太大。原 11261 / stm32test 也是这么做的。
 *--------------------------------------------------------------------------*/
#define IMU_SCL_PORT    GPIOB
#define IMU_SCL_PIN     GPIO_Pin_4
#define IMU_SDA_PORT    GPIOB
#define IMU_SDA_PIN     GPIO_Pin_3

/* CRL / CRH 里的 4 位配置码 */
#define PIN_MODE_OUT_PP 0x3U        /* 推挽输出 50MHz              */
#define PIN_MODE_IN_PU  0x8U        /* 输入（配 ODR=1 即内部上拉） */

static uint8_t g_mpu_ok   = 0U;
/* volatile：只被写、不被读，不加会被优化掉，调试器就看不到 */
static volatile uint8_t g_who_am_i = 0U;
static volatile uint8_t g_i2c_found = 0xEEU;    /* 0xEE = 未扫描，0xFF = 无人应答 */
static float g_bias_gx = 0.0f;
static float g_bias_gy = 0.0f;
static float g_bias_gz = 0.0f;

/* 位周期延时。软件 I2C 只有最快速度限制、没有最慢限制，所以偏大是安全的。*/
static void Imu_IicDelay(void)
{
    volatile uint32_t n = IMU_IIC_DELAY_LOOPS;
    while (n-- != 0U) { __NOP(); }
}

/* 毫秒延时：靠 Platform 的 SysTick 时基轮询，不引入阻塞式 Delay 模块。
 * 只能在主循环上下文（中断已开）里用，且 Platform_Init() 必须已经跑过。 */
static void Imu_DelayMs(uint32_t ms)
{
    uint32_t start = Platform_GetMs();
    while ((uint32_t)(Platform_GetMs() - start) < ms) { /* spin */ }
}

/* 求某个引脚在 CRL/CRH 里的位移（每引脚 4 位） */
static uint32_t Imu_PinShift(uint16_t pin)
{
    uint32_t shift = 0U;
    uint16_t mask  = (pin <= 0x00FFU) ? pin : (uint16_t)(pin >> 8);
    while ((mask & 1U) == 0U) {
        mask >>= 1;
        shift += 4U;
    }
    return shift;
}

/* 直接改写 CRL/CRH 来切换引脚模式 */
static void Imu_PinMode(GPIO_TypeDef *port, uint16_t pin, uint32_t mode)
{
    volatile uint32_t *cr;
    uint32_t shift = Imu_PinShift(pin);
    uint32_t tmp;

    cr = (pin <= 0x00FFU) ? &port->CRL : &port->CRH;

    tmp  = *cr;
    tmp &= ~(0xFU << shift);
    tmp |=  (mode << shift);
    *cr   = tmp;
}

static void Imu_SclHigh(void) { IMU_SCL_PORT->BSRR = IMU_SCL_PIN; }
static void Imu_SclLow(void)  { IMU_SCL_PORT->BRR  = IMU_SCL_PIN; }

static void Imu_SdaHigh(void)
{
    Imu_PinMode(IMU_SDA_PORT, IMU_SDA_PIN, PIN_MODE_OUT_PP);
    IMU_SDA_PORT->BSRR = IMU_SDA_PIN;
}

static void Imu_SdaLow(void)
{
    Imu_PinMode(IMU_SDA_PORT, IMU_SDA_PIN, PIN_MODE_OUT_PP);
    IMU_SDA_PORT->BRR = IMU_SDA_PIN;
}

/* 释放 SDA：先写 ODR=1 再切成输入，这样内部上拉才是上拉而不是下拉 */
static void Imu_SdaRelease(void)
{
    IMU_SDA_PORT->BSRR = IMU_SDA_PIN;
    Imu_PinMode(IMU_SDA_PORT, IMU_SDA_PIN, PIN_MODE_IN_PU);
}

static uint8_t Imu_SdaRead(void)
{
    return (IMU_SDA_PORT->IDR & IMU_SDA_PIN) ? 1U : 0U;
}

/*----------------------------------------------------------------------------
 * 软件 I2C 基本时序
 *--------------------------------------------------------------------------*/
static void IicInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = IMU_SCL_PIN | IMU_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* 空闲状态：两条线都输出高 */
    Imu_SclHigh();
    Imu_SdaHigh();
}

static void IicStart(void)
{
    Imu_SdaHigh();
    Imu_SclHigh();
    Imu_IicDelay();
    Imu_SdaLow();
    Imu_IicDelay();
    Imu_SclLow();
    Imu_IicDelay();
}

static void IicStop(void)
{
    Imu_SclLow();
    Imu_SdaLow();
    Imu_IicDelay();
    Imu_SclHigh();
    Imu_SdaHigh();
    Imu_IicDelay();
}

/* 返回 0 = 收到应答，1 = 超时失败 */
static uint8_t IicWaitAck(void)
{
    uint8_t err = 0U;

    Imu_SdaRelease();       /* 主机释放 SDA，让从机拉低表示 ACK */
    Imu_IicDelay();
    Imu_SclHigh();
    Imu_IicDelay();

    while (Imu_SdaRead()) {
        if (++err > 250U) {
            IicStop();
            return 1U;
        }
    }

    Imu_SclLow();
    Imu_IicDelay();
    return 0U;
}

static void IicAck(void)
{
    Imu_SclLow();
    Imu_SdaLow();
    Imu_IicDelay();
    Imu_SclHigh();
    Imu_IicDelay();
    Imu_SclLow();
    Imu_IicDelay();
}

static void IicNAck(void)
{
    Imu_SclLow();
    Imu_SdaHigh();
    Imu_IicDelay();
    Imu_SclHigh();
    Imu_IicDelay();
    Imu_SclLow();
    Imu_IicDelay();
}

static void IicSendByte(uint8_t txd)
{
    uint8_t t;

    Imu_SclLow();
    for (t = 0U; t < 8U; t++) {
        if (txd & 0x80U) { Imu_SdaHigh(); } else { Imu_SdaLow(); }
        txd <<= 1;
        Imu_IicDelay();
        Imu_SclHigh();
        Imu_IicDelay();
        Imu_SclLow();
        Imu_IicDelay();
    }
}

static uint8_t IicReadByte(uint8_t ack)
{
    uint8_t i, receive = 0U;

    Imu_SdaRelease();       /* 主机释放 SDA，交给从机驱动 */
    for (i = 0U; i < 8U; i++) {
        Imu_SclLow();
        Imu_IicDelay();
        Imu_SclHigh();
        receive <<= 1;
        if (Imu_SdaRead()) { receive++; }
        Imu_IicDelay();
    }

    if (ack) { IicAck(); } else { IicNAck(); }

    return receive;
}

/* 写寄存器，返回 0 = 成功 */
static uint8_t IicWriteReg(uint8_t reg, uint8_t data)
{
    IicStart();
    IicSendByte((uint8_t)(IMU_ADDR << 1));      /* 设备地址(7位) + 写 */
    if (IicWaitAck()) { return 1U; }
    IicSendByte(reg);
    if (IicWaitAck()) { return 2U; }
    IicSendByte(data);
    if (IicWaitAck()) { return 3U; }
    IicStop();
    return 0U;
}

/* 连续读多个寄存器（MPU6050 支持地址自动递增） */
static uint8_t IicReadLen(uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;

    IicStart();
    IicSendByte((uint8_t)(IMU_ADDR << 1));
    if (IicWaitAck()) { return 1U; }
    IicSendByte(reg);
    if (IicWaitAck()) { return 2U; }

    IicStart();                                 /* 重复起始条件 */
    IicSendByte((uint8_t)((IMU_ADDR << 1) | 0x01U));
    if (IicWaitAck()) { return 3U; }

    for (i = 0U; i < len; i++) {
        /* 最后一个字节回 NACK，其余回 ACK */
        buf[i] = IicReadByte((uint8_t)((i + 1U) < len));
    }

    IicStop();
    return 0U;
}

/* 总线扫描，返回第一个应答的 7 位地址；0xFF = 一个器件都没有。
 * 调试用：即使初始化失败，也能从 Imu_GetScanAddress() 看出
 * 总线上到底有没有器件，用来区分接线问题和寄存器配置问题。 */
static uint8_t IicScan(void)
{
    uint8_t a;

    for (a = 0x08U; a < 0x78U; a++) {
        IicStart();
        IicSendByte((uint8_t)(a << 1));
        if (IicWaitAck() == 0U) {
            IicStop();
            return a;
        }
        IicStop();
    }
    return 0xFFU;
}

/* 设置采样率：陀螺内部固定 1 kHz 采样，这里做分频；LPF 取采样率的一半 */
static uint8_t MpuSetRate(uint16_t rate)
{
    uint8_t lpf_cfg;

    if (rate > 1000U) { rate = 1000U; }
    if (rate < 4U)    { rate = 4U; }

    if (IicWriteReg(MPU_SMPLRT_DIV, (uint8_t)(1000U / rate - 1U))) { return 1U; }

    if      (rate / 2U >= 188U) { lpf_cfg = 1U; }
    else if (rate / 2U >=  98U) { lpf_cfg = 2U; }
    else if (rate / 2U >=  42U) { lpf_cfg = 3U; }
    else if (rate / 2U >=  20U) { lpf_cfg = 4U; }
    else if (rate / 2U >=  10U) { lpf_cfg = 5U; }
    else                        { lpf_cfg = 6U; }

    return IicWriteReg(MPU_CONFIG, lpf_cfg);
}

/*----------------------------------------------------------------------------
 * 对外接口
 *--------------------------------------------------------------------------*/

uint8_t Imu_Init(void)
{
    uint8_t id = 0U;

    g_mpu_ok = 0U;

    /* ★★★ 关键：必须先关掉 JTAG，否则 PB3/PB4 根本当不了 GPIO ★★★
     *
     * STM32 复位后 SWJ_CFG = 000，JTAG 使能。此时 (md) 封装：
     *     PB3 = JTDO   （JTAG 数据输出，由 TAP 驱动，GPIO 配置无效）
     *     PB4 = NJTRST
     * 我们把 PB3 当 I2C 的 SDA 用，可引脚实际被 JTAG 拉着，
     * 于是数据位全读成 0 —— WHO_AM_I 读出 0x00，而 ACK 还会「假成功」。
     *
     * 用 JTAGDisable（SWJ_CFG = 010）而不是 DISABLE：只关 JTAG、保留 SWD，
     * 否则 ST-Link 连不上，只能靠 BOOT0 才能救回来。
     *
     * 原 11261 工程里这行藏在 oled.c 的 OLED_Init() 里（OLED 初始化排在
     * MPU 之前，顺手把 JTAG 关了）。本工程没搬 OLED，所以必须自己补上。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* MPU6050 上电后约 100 ms 才能正常响应 I2C（数据手册规定）。
     * 原 11261 工程里前面有 OLED 初始化天然等够了时间，本工程没有，必须显式等。*/
    Imu_DelayMs(200U);

    IicInit();
    Imu_IicDelay();

    /* 调试：先扫一遍总线，把第一个应答的地址存起来 */
    g_i2c_found = IicScan();

    /* 复位 */
    if (IicWriteReg(MPU_PWR_MGMT_1, 0x80U)) { return 1U; }
    Imu_DelayMs(100U);

    /* 唤醒 */
    if (IicWriteReg(MPU_PWR_MGMT_1, 0x00U)) { return 2U; }
    Imu_DelayMs(10U);

    /* 陀螺量程 ±2000 dps（与 stm32test 一致） */
    if (IicWriteReg(MPU_GYRO_CONFIG, 3U << 3)) { return 3U; }

    /* 加速度量程 ±2 g（与 stm32test 一致） */
    if (IicWriteReg(MPU_ACCEL_CONFIG, 0U << 3)) { return 4U; }

    /* 采样率 100 Hz，与控制环同频 */
    if (MpuSetRate(100U)) { return 5U; }

    /* 关掉数据就绪中断、FIFO 和 I2C 主模式。
     * 本工程轮询读取，不需要 INT 输出，所以保持 INT_ENABLE = 0，
     * PB5 全程空闲、不需要任何配置。 */
    if (IicWriteReg(MPU_INT_ENABLE, 0x00U)) { return 6U; }
    if (IicWriteReg(MPU_USER_CTRL,  0x00U)) { return 7U; }
    if (IicWriteReg(MPU_FIFO_EN,    0x00U)) { return 8U; }

    /* 验证器件 ID */
    if (IicReadLen(MPU_WHO_AM_I, 1U, &id)) { return 9U; }
    g_who_am_i = id;
    if (id != IMU_WHO_AM_I_VALUE) { return 10U; }

    /* 时钟源选 PLL 并参考 X 轴陀螺，比内部 RC 稳定得多，陀螺零漂主要靠这个 */
    if (IicWriteReg(MPU_PWR_MGMT_1, 0x01U)) { return 11U; }

    /* 所有轴都打开 */
    if (IicWriteReg(MPU_PWR_MGMT_2, 0x00U)) { return 12U; }

    Imu_DelayMs(50U);

    g_mpu_ok = 1U;
    return 0U;
}

uint8_t Imu_CalibrateGyro(uint16_t samples, float tol_dps)
{
    int32_t sx = 0, sy = 0, sz = 0;
    int16_t gx, gy, gz;
    uint8_t buf[6];
    uint16_t i;
    int16_t tol;

    if (samples == 0U) { return 1U; }
    if (!g_mpu_ok)     { return 2U; }

    /* 等滤波器稳定 */
    Imu_DelayMs(50U);

    tol = (int16_t)(tol_dps * IMU_GYRO_LSB_PER_DPS);

    for (i = 0U; i < samples; i++) {
        if (IicReadLen(MPU_GYRO_XOUT_H, 6U, buf)) { return 2U; }

        gx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        gy = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        gz = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

        /* 运动检测：任一轴超过容差就认为车在动，本次标定作废 */
        if (gx > tol || gx < -tol ||
            gy > tol || gy < -tol ||
            gz > tol || gz < -tol) {
            return 3U;
        }

        sx += gx;
        sy += gy;
        sz += gz;

        Imu_DelayMs(5U);
    }

    g_bias_gx = ((float)sx / (float)samples) / IMU_GYRO_LSB_PER_DPS;
    g_bias_gy = ((float)sy / (float)samples) / IMU_GYRO_LSB_PER_DPS;
    g_bias_gz = ((float)sz / (float)samples) / IMU_GYRO_LSB_PER_DPS;

    return 0U;
}

uint8_t Imu_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (IicReadLen(MPU_GYRO_XOUT_H, 6U, buf)) { return 1U; }

    *gx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *gy = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *gz = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    return 0U;
}

uint8_t Imu_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    if (IicReadLen(MPU_ACCEL_XOUT_H, 6U, buf)) { return 1U; }

    *ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    return 0U;
}

uint8_t Imu_ReadGyroDps(float *gx, float *gy, float *gz)
{
    int16_t rx, ry, rz;

    if (Imu_ReadGyroRaw(&rx, &ry, &rz)) { return 1U; }

    *gx = ((float)rx / IMU_GYRO_LSB_PER_DPS) - g_bias_gx;
    *gy = ((float)ry / IMU_GYRO_LSB_PER_DPS) - g_bias_gy;
    *gz = ((float)rz / IMU_GYRO_LSB_PER_DPS) - g_bias_gz;
    return 0U;
}

uint8_t Imu_ReadGyroZDps(float *gz)
{
    uint8_t buf[2];
    int16_t rz;

    /* 只读 GYRO_ZOUT_H/L 两个字节，不走 6 字节的整组读取 */
    if (IicReadLen(MPU_GYRO_ZOUT_H, 2U, buf)) { return 1U; }

    rz = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *gz = ((float)rz / IMU_GYRO_LSB_PER_DPS) - g_bias_gz;
    return 0U;
}

uint8_t Imu_ReadAccelG(float *ax, float *ay, float *az)
{
    int16_t rx, ry, rz;

    if (Imu_ReadAccelRaw(&rx, &ry, &rz)) { return 1U; }

    *ax = (float)rx / IMU_ACCEL_LSB_PER_G;
    *ay = (float)ry / IMU_ACCEL_LSB_PER_G;
    *az = (float)rz / IMU_ACCEL_LSB_PER_G;
    return 0U;
}

uint8_t Imu_IsReady(void)          { return g_mpu_ok; }
float   Imu_GetBiasZ(void)         { return g_bias_gz; }
float   Imu_GetBiasX(void)         { return g_bias_gx; }
float   Imu_GetBiasY(void)         { return g_bias_gy; }
uint8_t Imu_GetWhoAmI(void)        { return g_who_am_i; }
uint8_t Imu_GetScanAddress(void)   { return g_i2c_found; }
