# Hardware 接入记录（2026-09-20）

Keil 工程 `../MDK/CarProject.uvprojx` 的目标已改为
`CarControl_Hardware_STM32F103C8`，输出名为 `CarControl_Hardware`。
Hardware 分组包含 `Avoid.c`、`track.c`、`Ultrasound.c`、`Motor.c`。
头文件保留在 Interfaces 分组，新增了 `Modules/Ultrasound.h`。
硬件目标不包含 Test 下的 Mock 实现；主机调度测试仍单独使用 Mock。

## Motor 驱动

实现：`Hardware/Motor.c`；公共接口：`Drivers/Motor.h`。
沿用原平衡车 `HARDWARE/MOTOR/motor.c/.h` 和 `HARDWARE/PWM/pwm.c` 的接线及电平：

| 电机 | PWM | 正需求方向电平 | 负需求方向电平 |
|---|---|---|---|
| 左 / 原 moto1 | PA8 / TIM1_CH1 | PB14=0，PB15=1 | PB14=1，PB15=0 |
| 右 / 原 moto2 | PA11 / TIM1_CH4 | PB13=0，PB12=1 | PB13=1，PB12=0 |

- `Motor_Init()`：初始化方向 GPIO 和 PWM，输出保持停止。无需再调用旧 `PWM_Init_TIM1()`。
- `Motor_SetDemand(left, right)`：两路独立有符号 PWM 需求，限幅到 -1000～1000；500 对应 50% 占空比，1000 对应 100%。这不是闭环速度或 mm/s。
- `Motor_Stop()`：立即装载两路零 PWM，并将四个方向脚置低。不锁存停车，后续需求可恢复运动；停车距离需实车验证。
- TIM1 使用 PSC=0、ARR=7199，72 MHz 定时器时钟下 PWM 为 10 kHz，与原工程一致。该驱动独占 TIM1，不占用超声波的 TIM3、编码器的 TIM2/TIM4 或平台 SysTick。
- 换向时先将旧 PWM 清零再改变方向。没有自动加减速控制。
- `MOTOR_LEFT_POLARITY`、`MOTOR_RIGHT_POLARITY` 默认 +1，保留原 `Load()` 极性。若实车某侧正需求对应反转，可将该侧改为 -1。
- 按所给电路图 STBY 接高电平，不配置额外 STBY 引脚。
- 接口由主循环串行调用，不与中断并发写电机。未初始化时的控制调用被忽略。
- 不引入旧平衡角度控制、编码器 PID、`sys.h` 位带宏和旧 `Load()/Stop()` 全局变量依赖。

## 验证结果

1. 使用本机 Arm Compiler 5.06 对实际 Motor.c 编译，启用严格诊断并将警告作为错误，成功生成 ARM 对象文件。
2. `Test/run_motor_tests.ps1` 使用实际 Motor.c 与主机外设替身，验证初始化零输出、左右引脚、占空比、正反转、INT16 边界限幅、换向清零顺序、立即停车、再次启动和重复初始化；四种左右极性组合全部通过。
3. `Test/run_tests.ps1` 原 CarControl 调度测试通过。
4. uvprojx 和 uvoptx 分组已同步，工程引用文件路径全部存在。

这些验证不等同于实车验证；尚未烧录或驱动电机。

## Track / Avoid 封装更新

已在队友工程 `IO_Track/APP` 找到 app_irtracking.h/.c。
该头文件引入 AllHeader.h，继续依赖原 ZE 板的 BSP、四轮电机等代码。
因此只参考其中的数字探头定位和 PD 思路，在本工程内部实现所需功能，
不继续依赖 Car.h、Delay.h、app_irtracking.h 或原 main/定时器循迹入口。

### track.c：八路串口版本

- 根据新加入的模块文档、官方 STM32 串口源码，以及原电路图的 PA2/PA3 接口，选择 USART2 方案；不再使用空 GPIO 表或 ZE 板八路 IO 映射。
- USART2 初始化、收发 ISR、严格协议解析、左右探头排列、黑线电平、分类、稳定确认、时效检查、PD 参数和电机输出全部在此文件内。硬件为 PA2 TX→模块 RX、PA3 RX←模块 TX、115200 8N1。
- 每次 Track_Init 后非阻塞等待 5 秒（`TRACK_WARMUP_MS`）；发送 `$0,0,1#` 请求数字数据。缺少新鲜数据时每秒重试一次，发送也通过中断完成，不使用忙等。
- 仅接受完整的 43 字节 `$D,x1:0,...,x8:0#` 数字帧（每位 0 或 1）；逐一验证 x1～x8 标签、分隔符、长度和值。拒绝模拟帧、截断/超长/非法帧；新 `$` 可重新同步，整帧接收时间超过 20 ms 则丢弃。
- 八路无黑点为 LOST，至少六路黑为 WIDE，分离黑点簇为 AMBIGUOUS，单簇连续 3 个有效新帧后为 NORMAL；累计稳定次数的帧间隔至少 10 ms。
- 超过 150 ms 没有新鲜有效帧返回 INVALID，数据恢复后重新确认；串口接收错误也使数据失效。时间戳取接收完成时刻，重复读取或晚处理不刷新数据年龄。
- 偏差按探头位置映射到 -1000～1000；负值在左，正值在右。
- ★ 2026-09-23：**本文件不再写电机**。每步只算出「目标航向角 + 目标速度」（`DriveCmd`），由 CarControl 交给 `Heading_Drive()`，车轮差速由航向环产生（见下文 Heading.c）。
- 转向策略：`目标航向角 = 航向基准 + 修正角`。修正角 = `-TRACK_STEER_KP_DEG × error / 1000`（默认 22 度/满幅偏差，与旧开环 300/1000 差速力度相当，推导见 track.c 注释），限幅 ±45 度；偏差微分项 `TRACK_STEER_KD_DEG` 默认 0（航向环自带角速度阻尼）。
- 航向基准是「线路居中时车头朝向」：进循迹时取当前 yaw，之后按 `-TRACK_HEADING_TRIM_GAIN_DEG × error` 慢速修正，方向与修正角一致（等效线偏差积分项）。直道上偏差归零后基准不再动，锁定航向不画龙；弯道上偏差持续存在，基准跟着弯一直转过去。单步限幅 0.5 度，相对进入时朝向限幅 ±120 度防绕圈。
- 速度规划：`|error| ≤ 200` 时目标速度 50 计数/10ms，`≥ 800` 时降到 25，中间线性；宽黑上限 25；每个新样本最多加 1，减速立即生效。单位是「两轮编码器计数之和 / 10ms」，不再是通过直流增益反推的 PWM。
- ★ 2026-09-25 丢线搜索：只有「上一次有效检测已经严重偏离」时，丢线后才继续找线。门槛 `TRACK_SEARCH_MIN_ABS_ERROR` = 800（量级参考：最外侧一路 1000、最外侧两路 857 算严重；第二路 714、第二+第三路 571 算轻微，丢线即停）。满足门槛后按「车最后偏右 → 继续左转，最后偏左 → 继续右转」沿最后一次的修正方向持续旋转目标航向角（`TRACK_SEARCH_ENABLE`；速度 `TRACK_SEARCH_SPEED` = 15，速率 `TRACK_SEARCH_RATE_DEG` = 1.0 度/步 ≈ 100°/s，累计转角上限 `TRACK_SEARCH_MAX_DEG` = 150 度）。找到线（NORMAL/WIDE）立刻结束搜索、恢复正常循迹并从最低速度重新起步；`TRACK_SEARCH_TIMEOUT_MS` = 1000 ms 超时、没到门槛、线居中（`error == 0`）、数据无效（INVALID）时都输出 0 速度，是否停车由 CarControl 决定（`CAR_TRACK_LOSS_TOLERANCE_MS` 必须大于搜索超时，现为 1300 ms）。
- 偏离程度记忆 `last_turn_error`（最后一次有效偏差，含符号，居中帧会把它清成 0）故意跨 `ResetControl` 保留：丢线期间 `Track_SenseUpdate` 每一帧都会清控制历史，若连它一起清掉，搜索会每帧重新开始、永远到不了超时。只有 `Track_Reset`（整车停车/交接/故障）和 `Track_Init` 结束搜索。
- 参数全部是待实车标定的起点：先整定 `TRACK_STEER_KP_DEG`，再按需要加入 `TRACK_HEADING_TRIM_GAIN_DEG`（设为 0 即退回纯比例）。
- 每次控制更新上次误差，复位后第一次控制不产生微分突跳。Track_Reset 保留感知结果，只清除控制历史。
- 默认黑线为 0，X1 在车体左、X8 在右。本车实测转向相反，`track.c` 顶部 `TRACK_REVERSE_ORDER` 已设为 1（等价于镜像探头顺序）；对称位置权重确保中央 X4/X5 同时在线时偏差为 0。
- 新增只读 `Track_GetSampleId()` 供 Avoid 辨别新帧，保持 TrackStatus 结构及原控制接口不变；CarControl 无需读取帧编号。编号本身不代表有效性，必须结合线路状态使用。
- 协议没有校验和，语法检查不能检测所有位错误；软件也不能从数字帧判断模块是否已完成正确的黑白校准。接线和校准操作见 [八路循迹接线与联调](八路循迹接线与联调.md)。

### Avoid.c

- 测距判定、方向选择、绕障步骤、寻线修正和回线确认全部在此文件内。
- 15 cm 进入障碍判断，18 cm 解除，避免阈值附近反复切换；任一测距失效使检测结果 INVALID，执行期间距离小于 5 cm 则失败停车。
- ★ 2026-09-23：绕障动作从「固定时间 + 固定 PWM」改成「目标航向角 + 编码器里程」，本文件同样不写电机，只输出 `DriveCmd`：
  1. **转出**：目标航向 = 基准 ∓ `AVOID_TURN_DEG`（默认 45 度；`away_direction > 0` 表示“向右让”，而航向角正方向是左转，所以取负号），到位判据是航向角 ±5 度，超时 2 s；
  2. **直行**：保持基准航向，到位判据是两轮里程增量 ≥ `AVOID_PASS_COUNTS`（默认 900 计数），速度 `AVOID_PASS_SPEED` 25，超时 4 s；
  3. **回正**：转回基准航向，到位判据 ±5 度，速度 12，超时 3 s；
  4. **寻线**：偏差 → 相对基准的航向修正角（30 度/满幅，限幅 ±35 度），速度 18；没看到线就直着朝基准方向找。
- 基准航向在 `Avoid_Start()` 时取当前 yaw，绕障前后的前进方向因此由陀螺保证一致 —— 这也是规则里「不能逆行」的软件依据。
- SEARCHING 最多 4 秒。必须连续 5 个新帧 NORMAL、偏差绝对值不超过 300、障碍已解除，**并且航向已经回到基准方向 ±25 度以内**，才锁存 DONE。丢线、偏差过大、障碍或调度间隔过长清除确认计数；绕障阶段的线路观察不计入确认。
- 回线确认依赖 CarControl 每轮先 Track_SenseUpdate 再传入最新状态的调用顺序，内部检查 Track_GetSampleId；即使跨多个控制周期重复使用同一 UART 帧，也不会累计次数。
- SenseUpdate、Start、Reset 不写电机；DONE 由整车复位后才回到 IDLE。所有运动统一由 CarControl 交给 `Heading_Drive()`。
- 角度、里程、速度都是待实车标定的起点。超时兜底只保证「不会无限走下去」，不代表轨迹一定能绕过实物，也不能仅凭软件测试认定能过障碍。

### Ultrasound.c：仅保留底层采集职责

- PB10/PB11 为左右触发，PB0/PB1 为左右回波；沿用用户提供的 Ultrasound.h 接线。
- TIM3 作为 1 MHz 自由运行计数器，CC1 中断在约 20 us 后撤销触发；EXTI0/1 捕获上升、下降沿。不占用 SysTick、不控制电机。
- ⚠️ 2026-09-23：本模块的定时器已从 TIM4 改为 TIM3。原因是右轮编码器只能接 PB6/PB7，也就是 TIM4_CH1/CH2，而 stm32test 的编码器正是接在那里。本模块只用定时器当时基、不使用任何定时器引脚，所以换实例对外部没有任何影响：触发/回波引脚 PB10/PB11/PB0/PB1 仍原封不动。
- 主机测试的假外设头（`Test/ModuleHardware/stm32f10x.h`）同步从 `TIM4` 改为 `TIM3`，注意那里是 `#undef` + 重定义，改名时必须一起改。
- 两路交替测量，完成后静默 60 ms，35 ms 无完整回波判失效；不足 100 us、超过 30000 us 的脉宽无效。
- 完成时间按回波实际捕获时刻记录，超过 400 ms 失效；晚处理旧回波不会刷新其年龄。两路均有有效结果后才允许启动。
- Ultrasound_Update 由 Avoid_SenseUpdate 调用；原距离 getter 已改为只读缓存，不再同步触发测量。

## Encoder.c / Imu.c / Heading.c（2026-09-23 新增）

三个模块共同实现「顶层输入航向角，小车自动对准」。接线全部沿用原平衡车，**不需要任何硬件改动**。

### 资源分配

| 资源 | 用途 | 备注 |
|---|---|---|
| TIM2 = PA0/PA1 | 左编码器 | stm32test 原样 |
| TIM3 | 超声波时基 | 从 TIM4 迁来，不用引脚 |
| TIM4 = PB6/PB7 | 右编码器 | stm32test 原样 |
| PB4 = SCL / PB3 = SDA | MPU6050 软件 I2C | PB5（INT）不使用 |
| PB0/PB1/PB10/PB11 | 超声波回波/触发 | 不变 |

### Encoder.c

- 编码器接口模式 TI12（A、B 相都计上升沿，等效 4 倍频）、IC 滤波 10、ARR 65535，逐项对齐 stm32test。滤波 10 是那边实测出来压电机换向噪声的，不要改小。
- 标准外设库用 `TIM_EncoderInterfaceConfig()`（HAL 是 `HAL_TIM_Encoder_Init()`），`TIM_GetCounter()` 直接传 `TIM_TypeDef*`，不像 HAL 的 `__HAL_TIM_GET_COUNTER` 要传句柄。
- `Encoder_GetCount()` 读走本周期增量并清零，(int16_t) 转换天然处理计数器回绕；100 Hz 下每周期只有几十个计数，所以「读+清零」之间丢掉一个脉冲的影响可以忽略。
- `ENCODER_SIGN_LEFT/RIGHT` 默认 +1 / -1（与 stm32test 实测一致），必须实车确认。**只翻编码器，不要连电机极性一起翻**，两次反相会抵消，速度环变正反馈、PWM 顶到饱和。

### Imu.c

- 由 stm32test 的 `softiic.c` + `mpu6050.c` 移植，算法保持一致。改动：HAL → 标准外设库；去掉总线对象抽象；去掉 EXTI/INT 那一整套；`delay_us` 的 DWT 实现换成 NOP 循环；毫秒延时用 `Platform_GetMs()` 轮询（因此没有新增 Platform 接口）。
- ★ `Imu_Init()` 开头必须 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)`：PB3/PB4 复位后是 JTAG 的 JTDO/NJTRST，不关 JTAG 就当不了 GPIO，现象是 SDA 恒低、数据位全读 0、`WHO_AM_I` 返回 0x00，而 ACK 还会「假成功」。**必须用 JTAGDisable 而不是 DISABLE**，后者会把 SWD 一起关掉、ST-Link 就连不上了。原 11261 工程里这行藏在 `oled.c` 的 `OLED_Init()` 里，本工程没搬 OLED，所以必须自己补上。
- SCL/SDA 用**推挽输出**而不是开漏：开漏靠 30~50k 内部上拉，配上杜邦线几十 pF 电容，上升时间约 2.4 us 超过位周期，从机收不到有效起始条件，表现是「完全没有 ACK」。
- 不使用 INT 引脚：控制环跑在主循环的 10 ms 调度里，轮询反而避开了「中断和主循环抢同一条软件 I2C」的隐患（stm32test 为此专门规定了「必须先标定再开中断」的顺序）。
- 每个控制周期只读 Z 轴 2 字节（`Imu_ReadGyroZDps`）而不是 6 字节整组，省约 250 us I2C 时间。
- 只读诊断量：`Imu_GetWhoAmI()`、`Imu_GetScanAddress()`（总线扫描首个应答地址，0xFF = 无人应答），用来区分接线问题和寄存器配置问题。

### Heading.c

- ★ 2026-09-23：本模块是全车**唯一**的驱动出口。循迹和避障不再自己拼 PWM，而是各自给出「目标航向角 + 目标速度」（`DriveCmd`），由 `Heading_Drive()` 一次完成「设定目标 + 更新输出」。`Motor_SetDemand()` 现在只应该出现于 Heading.c 和停车路径。
- `Heading_Stop()`：停车并清速度环积分、低通、输出和编码器残留计数，但保留 yaw、目标角和目标速度，用于「先停一下再继续」（循迹丢线、避障交接、里程未到点的临时停车）；`Heading_Reset()` 仍然会把当前朝向重新定义为 0°。
- `Heading_ResetOdometer()`：只清累计里程（`Encoder_ResetTotal`），不动角度和速度设定。CarControl 在启动、进入航向模式和 Init 时调用它，「本次运行里程」因此从启动开始算。
- 结构同 stm32test：编码器 → 速度环(PI) 与 陀螺积分航向 → 转向环(P + 角速度 D) 做差速混控。
- 统一符号：正速度 = 前进，正航向角 = 左转（逆时针），正转向量 = 左转。混控为 `PWM左 = 速度环 - 转向环`、`PWM右 = 速度环 + 转向环`。
- 航向用「连续累加角度」，从根上避免了 ±180° 跳变，因此不需要 stm32test 里那段解绕分支。
- dt 用 `Platform_GetMs()` 实测并限幅到 [5ms, 30ms]。stm32test 因为本板 `DWT->CYCCNT` 会偶发「倒退」，反而只能用固定 dt；本模块没有 DWT 依赖。
- **增益默认值在 `Drivers/Heading.h`**（没有放进 `App/CarConfig.h`，而是沿用 `Drivers/Motor.h` 的「驱动自带 `#ifndef` 默认值」惯例）。取值是 stm32test 实测整定值按 PWM 量程等比缩放：满量程 7200 → 1000，系数 0.1389，因此环路增益、闭环带宽、相位裕度与那边实测一致。
- ⚠️ `VEL_I_LIMIT` 不能给小：去掉直立环后被控对象是有限直流增益，**稳态 PWM 必须全部由积分项承担**，所以要求 `KI × I_LIMIT ≥ OUT_LIMIT`。这个坑是主机测试抓出来的（给 300 时稳态误差约 47%）。

### 完整构建和测试

2026-09-20 使用 `MDK/build_hardware.ps1` 按 uvprojx 中的源文件、宏和头文件路径，
调用本机 Arm Compiler 5.06、汇编器和链接器，成功生成
`Build/HardwareCLI/CarControl_Hardware.axf` 与 `.hex`。
链接按 C8 的 64 KB Flash / 20 KB RAM 限制检查：2026-09-20 时 ROM 14108 字节，RW+ZI 1896 字节；加入编码器/IMU/航向环后 Flash 14748 字节（22.5%），RAM 1536 字节（7.5%）；2026-09-23 转向统一改造后（PlatformIO 构建）Flash 15640 字节（23.9%），RAM 1592 字节（7.8%）。
Keil 工程已启用 C99；脚本的输出目录独立于 IDE 的 Build 输出。

以下测试全部通过：

- `Test/run_module_tests.ps1`：实际 USART2 ISR、Track、Avoid 与 CarControl 联合测试，覆盖上电预热（5 秒）、启动命令与重试、格式校验/缓冲边界/超时重同步、错误帧不刷新时效、接收错误、两种极性×两种安装方向、线偏差→目标航向角、复位保持感知、只统计新帧的稳定回线、DONE 锁存、断线停车及毫秒回绕。
- `Test/run_ultrasound_tests.ps1`：实际采集状态机和 ISR，覆盖触发撤销、左右回波、静默间隔、错误通道、超时恢复、短脉冲、卡高、迟到回波、旧结果时效、16 位计时与 32 位毫秒回绕。
- `Test/run_heading_tests.ps1`：航向环闭环测试。用带测试钩子的传感器替身接一个差速小车模型，覆盖转向符号、±角度收敛、死区、速度环稳态误差、直行不跑偏、IMU 故障停车、复位语义。该测试经反向验证：把 `HEADING_GYRO_SIGN` 故意改成 +1 会导致航向发散并断言失败，证明它真能抓住符号错误。
- `Test/run_motor_tests.ps1` 和 `Test/run_tests.ps1`：电机与原调度回归测试。

### 2026-09-23 转向统一改造后的验证

- `pio run` 完整编译链接通过（Flash 15640 / 65536，RAM 1592 / 20480，无警告）。
- 主机测试（本机无 GCC，用工程内 zig/clang 代替；需要 `-Wno-deprecated-non-prototype` 绕过 CMSIS V1.30 的旧式声明；2026-09-23 起全部源文件已统一为 UTF-8，不再需要 `-Wno-invalid-utf8`）：
  - `Test/run_tests.ps1` 对应状态机测试：新增「循迹只产出命令、航向环是唯一写电机者」「里程到阈自动锁存停车」「惯导未就绪不允许启动」「运行中 IMU 失败→CAR_FAULT」「航向模式只调度 Heading_Update」五项断言，全部通过。
  - `Test/run_module_tests.ps1` 对应模块测试：真实 track.c / Avoid.c / CarControl.c，断言改为读 `Track_GetDrive` / `Avoid_GetDrive`，四种极性×安装方向组合全部通过。避障用例改为用可设置的 yaw 和里程读数推进阶段，因此绕障的角度/里程判据也在测试覆盖内。
  - `Test/run_heading_tests.ps1` 对应航向环测试：新增 `Heading_Drive`、`Heading_Stop`、`Heading_ResetOdometer` 三个用例，全部通过。
  - `Test/run_motor_tests.ps1`、`Test/run_ultrasound_tests.ps1` 仍全绿。
- 上电预热为 **5 秒**（`TRACK_WARMUP_MS = 5000U`）：模块测试改为按 5000 ms 断言（4999 ms 不发请求、5000 ms 发一次），代码与文档里的「20 秒」描述已同步改掉；模块 KEY 校准步骤里的 20 秒保留（那是模块手册的要求）。

没有烧录或实车验证。GPIO 电平、电机方向、传感器输出电压及动作参数仍需按实物核对。
main 仍为原调试命令入口，正式启动、起始横线和终点识别尚未接入。
原始三个模块和 Ultrasound.h 的备份位于 `Build/Before_module_integration/`。
本次串口改造前的 Track/Avoid/接口/模块测试备份位于 `Build/Before_uart_integration/`。
