/* MPU6050 六轴传感器驱动（标准外设库 + 软件 I2C）。
 *
 * 接线与 stm32test 完全一致，硬件不需要任何改动：
 *
 *     PB4 = SCL
 *     PB3 = SDA
 *     PB5 = INT（本驱动不使用，见下）
 *
 * 为什么不用 INT 引脚 / 外部中断？
 *   本工程的控制环跑在主循环的 10 ms 调度里（CarControl_Update），不是挂在
 *   MPU 的中断上。轮询读取反而更安全：软件 I2C 是有时序要求的总线，
 *   如果中断里也在读它，标定和中断会互相踩位时序（stm32test 就为此专门
 *   规定了「必须先标定再开中断」的顺序）。这里只有一个上下文访问总线。
 *
 * ⚠️ PB3 / PB4 复位后是 JTAG 的 JTDO / NJTRST，由 JTAG 单元驱动，
 *    不改 SWJ_CFG 的话这两个脚当不了 GPIO —— 现象是 SDA 恒低、
 *    数据位全读 0、WHO_AM_I 返回 0x00，而 ACK 还会「假成功」。
 *    Imu_Init() 里已经做了这一步（只关 JTAG、保留 SWD，不影响烧录）。
 *
 * 不加载 DMP：没有平衡功能，Pitch/Roll 用不上，而 MPU6050 没有磁力计，
 * DMP 的 Yaw 也是纯陀螺积分，精度并不比自己积分好，还省掉一大坨固件。
 * 这与 stm32test 的结论一致。
 */
#ifndef IMU_H
#define IMU_H
#include <stdint.h>

#define IMU_ADDR             0x68U
#define IMU_WHO_AM_I_VALUE   0x68U

/* 量程换算（本驱动固定使用这两档） */
#define IMU_GYRO_LSB_PER_DPS 16.4f      /* 陀螺 ±2000 dps → 16.4  LSB/(°/s) */
#define IMU_ACCEL_LSB_PER_G  16384.0f   /* 加速度 ±2 g   → 16384 LSB/g      */

/* 位周期延时循环次数。软件 I2C 只有「最快速度」限制，没有最慢限制，
 * 所以这个值偏大是安全的，偏小才会丢 ACK。需要时可调。
 * 对应约 2~3 us（72 MHz）。 */
#ifndef IMU_IIC_DELAY_LOOPS
#define IMU_IIC_DELAY_LOOPS  36U
#endif

/* 初始化 MPU6050：关 JTAG、配软件 I2C、复位、设量程与采样率。
 * 返回 0 = 成功；非 0 = 失败，数值即失败步骤（见 Imu.c 里的注释）。
 * 失败时可用 Imu_GetWhoAmI() 看器件 ID，或 Imu_GetScanAddress() 看总线上
 * 第一个应答的地址，用来区分「接线问题」和「寄存器配置问题」。 */
uint8_t Imu_Init(void);

/* 陀螺零偏标定。★ 调用时车必须完全静止！会阻塞约 samples*5 ms。
 * 返回 0 = 成功；1 = 采样数为 0；2 = I2C 读失败；3 = 检测到车在动。 */
uint8_t Imu_CalibrateGyro(uint16_t samples, float tol_dps);

uint8_t Imu_IsReady(void);              /* 1 = Imu_Init 成功过 */

uint8_t Imu_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);
uint8_t Imu_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);

/* 已减掉零偏的角速度，单位 °/s。返回 0 = 成功。 */
uint8_t Imu_ReadGyroDps(float *gx, float *gy, float *gz);

/* 只读 Z 轴角速度（°/s）。航向积分只需要 Z 轴，读 2 字节比读 6 字节
 * 省约 250 us I2C 时间，控制环里用这个。返回 0 = 成功。 */
uint8_t Imu_ReadGyroZDps(float *gz);

uint8_t Imu_ReadAccelG(float *ax, float *ay, float *az);

float   Imu_GetBiasZ(void);             /* 标定得到的 Z 轴零偏 (°/s) */
float   Imu_GetBiasX(void);
float   Imu_GetBiasY(void);

/* --- 调试用只读量（故意做成 volatile，避免被优化掉、调试器看不到） --- */
uint8_t Imu_GetWhoAmI(void);            /* WHO_AM_I 原始读数，正常 = 0x68 */
uint8_t Imu_GetScanAddress(void);       /* 总线扫描首个应答地址；0xFF = 无人应答 */

#endif /* IMU_H */
