# STM32F103C8T6 循迹避障小车

本工程按设计方案划分整车调度、循迹和避障，使用 STM32 标准外设库。
当前真实模块已完成软件接入和 ARM 编译链接，尚未实车验证。
**已按现有电路图和模块官方串口示例配置 USART2：PA2→模块 RX、PA3←模块 TX，115200 8N1。**
上电先非阻塞等待 5 秒（`TRACK_WARMUP_MS`），再请求八路数字数据；收到稳定有效帧后允许启动。

## 打开工程

打开 `MDK/CarProject.uvprojx`，目标选择 `CarControl_Hardware_STM32F103C8`。
使用 Arm Compiler 5.06、STM32F103C8、medium-density 启动文件、64 KB Flash / 20 KB RAM。
时钟沿用原工程的 8 MHz 外部晶振、72 MHz 系统时钟，需与实物一致。
若 Keil 已打开此工程，重新加载以读取更新的 Hardware 分组。

## 模块职责

| 位置 | 职责 |
|---|---|
| App/CarControl.c | 每 10 ms 调度，决定控制权，处理停车、故障、避障总超时和 DONE 交接 |
| Hardware/track.c | USART2 初始化/收发/解析、八路黑白状态、稳定确认、时效；线偏差 → 目标航向角 + 目标速度（不写电机） |
| Hardware/Avoid.c | 障碍判断、方向选择；绕障按「目标航向角 + 编码器里程」推进，寻线和稳定回线确认（不写电机） |
| Hardware/Ultrasound.c | TIM3 + EXTI0/1 异步回波采集，返回测距缓存（触发/回波引脚不变） |
| Hardware/Motor.c | 原平衡车双电机接线，TIM1 PWM、方向、限幅和停车 |
| Hardware/Encoder.c | TIM2(PA0/PA1)=左、TIM4(PB6/PB7)=右，硬件正交编码器 4 倍频 |
| Hardware/Imu.c | MPU6050（软件 I2C，PB4=SCL/PB3=SDA）、陀螺零偏标定、Z 轴角速度 |
| Hardware/Heading.c | **全车唯一写电机的地方**：接收「目标航向角 + 目标速度」，由航向环（P + 陀螺 D）和速度环(PI) 产生左右轮差速 |
| Modules / Drivers | 公共头文件，保留 Track/Avoid/CarControl 的原设计接口 |
| Platform | SysTick 毫秒时基 |
| Test | 主机测试，不加入 Keil 硬件目标 |

Track 与 Avoid 的 SenseUpdate 均不控制电机；它们的 Update 也只输出「目标航向角 + 目标速度」，真正写电机的是 CarControl 调用的 `Heading_Drive()`。
Avoid 在寻线阶段确认稳定回线后锁存 AVOID_DONE。
CarControl 收到 DONE 后停车、Avoid_Reset、Track_Reset，下一周期恢复循迹。
Track_Reset 只清除控制历史，保留线路感知。

## 转向与里程（2026-09-23 改造）

**转向只有一个入口：目标航向角 + 目标速度。** 三种运行模式都不再自己拼 PWM 差速：

```
线偏差 → 目标航向角 ─┐
                     ├→ 转向环(P + 陀螺角速度 D) ─┐
陀螺积分航向 ────────┘                            ├→ 差速 → 电机
目标速度 ─┐                                        │
编码器 ────→ 速度环(PI) ────────────────────────┘
```

- 接口在 `Drivers/Heading.h`：`DriveCmd { steer_deg, speed_cmd }` + `Heading_Drive()`（一次性完成设定目标和更新输出）。`steer_deg` 单位度、正 = 左转；`speed_cmd` 单位是「两轮编码器计数之和 / 10ms」，正 = 前进，0 = 原地对准。
- `CarControl` 每 10 ms 选一个模块说话，取它的命令交给 `Heading_Drive()`；`CarControl_GetDrive()` 可以读出本周期真正下发的命令（`main.c` 里就是 `debug_steer_deg` / `debug_speed_cmd`）。
- 各模式如何定目标角：
  - 循迹：`目标角 = 航向基准 + 修正角（-22°/满幅偏差，限幅 ±45°）`；航向基准是「线路居中时的车头朝向」，按偏差积分慢速修正（等效 I 项），所以直道锁航向、弯道跟着弯转。同时按偏差把目标速度从 50 降到 25（宽黑上限 25）。
  - 避障：转出 45°、直行到 900 计数、转回基准，然后线偏差 → 相对基准的修正角；回线还要航向回到基准 ±25° 内。
  - 航向模式：直接用顶层给的 `heading_target_deg` / `heading_speed_cmd`。
- 反馈依赖：三种模式都需要陀螺。IMU 未就绪时 `CarControl_Start()` 直接拒绝；运行中读取失败会立即故障停车。
- **里程自动停车**：本次运行累计里程（左右轮计数之和）≥ `CAR_ODOMETER_STOP_COUNTS`（`App/CarConfig.h`，默认 100000）时锁存 `CAR_FINISHED` 并停车，三种模式都适用；启动、进入航向模式和 Init 会把里程清零。注意这是编码器计数不是米，需先用标定的 counts/mm 换算。
- 可调参数都在各模块顶部（`TRACK_STEER_*` / `TRACK_HEADING_*` / `TRACK_SPEED_*` / `AVOID_*`），全部是待实车标定的起点。建议顺序：先把航向环的 `HEADING_KP/KD` 整定好（见上电自检第 7 项），再调 `TRACK_STEER_KP_DEG`（拐不过来就加大、画龙就减小），最后按需加入 `TRACK_HEADING_TRIM_GAIN_DEG`；避障的 45°/900 计数/速度同理。

## 当前配置与边界

- 八路数据经 PA2/PA3 两根串口信号线传输，不再需要八个 GPIO 输入。黑线默认 0、白色背景默认 1，X1 默认在车体左侧；本车实测转向相反，`track.c` 中 `TRACK_REVERSE_ORDER` 已设为 1（若以后安装方向变回相反再改回 0）。
- 模块第一次使用或改变安装高度/环境后，需通过板载 KEY 完成黑白校准。操作与接线见 [八路循迹接线与联调](Hardware/八路循迹接线与联调.md)。上电等待不能替代校准。
- 原 IO_Track 的 PC0～PC5 是 ZE 板配置，未复制进 C8。无需再引入 app_irtracking.h、AllHeader.h 或旧 BSP。
- 电机及超声波接线、控制参数、采集时序详见 [Hardware 接入说明](Hardware/README.md)。
- **转向只有一个入口**（2026-09-23 改造）：循迹、避障、航向三种模式都只给出「目标航向角 + 目标速度」，车轮差速统一由 `Heading.c` 的航向环产生，编码器速度闭环和陀螺航向闭环对三种模式都生效。具体角度、里程、增益仍需实车标定，详见[转向与里程](#转向与里程2026-09-23-改造)。
- 本次运行累计里程（左右轮计数之和）到 `CAR_ODOMETER_STOP_COUNTS`（`App/CarConfig.h`，默认 100000 计数）自动停车并锁存 `CAR_FINISHED`，三种运行模式都适用。
- main 上电后自动尝试启动：空闲时每 500 ms 调用一次受保护的 `CarControl_Start()`，在 5 秒预热和传感器数据就绪前会被拒绝。因此上电后先等待，数据可用后自动进入循迹，不需要调试器写入启动命令。
- `debug_command` 调试入口保留并覆盖自动启动：1 启动、2 停止、3 模拟已确认终点、4 重新初始化。命令 2 和 3 会关闭自动启动（便于锁定停车），1 和 4 重新打开。
- WIDE 只表示宽黑区候选，不作为终点；起始横线处理、正式启动按钮和终点判定尚未实现。宽黑单簇按质心继续循迹，不再判故障；**严重偏离后丢线会继续找线**（2026-09-25 改造）：只有上一次有效检测已经严重偏离（`|error| ≥ TRACK_SEARCH_MIN_ABS_ERROR`，默认 800，即线已滑到最外侧一两路）时，丢线后才沿最后一次的修正方向继续转向找线（车最后偏右就继续左转，最后偏左就继续右转），1 秒内找到线就立刻回到正常循迹；轻微偏离后丢线、线居中后丢线、数据无效（`TRACK_INVALID`）都只把目标速度置 0 并停车。整车在 `CAR_TRACK_LOSS_TOLERANCE_MS`（默认 1300 ms，见 `App/CarConfig.h`）内没拿回稳定线路才锁存 `CAR_ERROR_TRACK`；搜索参数见 `track.c` 顶部的 `TRACK_SEARCH_*`。
- 故障和完成状态锁存，Stop 不解除，非比赛复位调用 Init。

## 航向模式（顶层输入航向角）

顶层只给一个目标航向角，小车自己转过去并保持。接口在 `User/main.c` 顶部：

```c
volatile float heading_target_deg;  /* 目标航向角（度），正 = 左转 */
volatile int   heading_speed_cmd;   /* 目标速度（编码器计数和/10ms），0 = 原地 */
```

- 进入方式：`debug_command = 5`，或把 `main.c` 里的 `CAR_START_MODE_HEADING` 改成 1。
- 进入时把「车头当前朝向」定义为 0°，所以 `SetHeading(0)` = 沿当前方向直行，`SetHeading(90)` = 左转 90° 后锁住。可超出 ±180°，内部用连续角度累加，不存在跳变。
- 反馈：航向来自 MPU6050 陀螺 Z 轴积分，速度来自编码器。因此本模式不依赖循迹模块和超声波；测距仍会照常刷新，供上层做前方避让。
- 上电先做陀螺零偏标定（约 1.1 秒，**此时车必须静止**），失败重试 5 次；标定失败不影响原来的循迹模式，只是进不了航向模式。
- 增益默认值在 `Drivers/Heading.h`，由 stm32test 实测整定值按 PWM 量程（7200 → 1000，系数 0.1389）等比缩放而来，环路增益与那边实测一致，但仍需实车微调。

### 上电自检顺序

| 步骤 | 观察量 | 期望 | 不符合怎么办 |
|---|---|---|---|
| 1 | `debug_imu_who_am_i` | 0x68 | 0x00 → JTAG 未关成功；0xFF → I2C 接线/上拉问题 |
| 2 | `debug_imu_scan` | 0x68 | 0xFF → 总线上一个器件都没应答 |
| 3 | 静止 10 秒看 `debug_heading` | 漂移 < 1° | 偏大 → 检查 `PWR_MGMT_1 = 0x01`（PLL 时钟源）是否写入成功 |
| 4 | 用手把车**左转** 90° | `debug_heading` 增大 | 减小 → `HEADING_GYRO_SIGN` 从 -1 改成 +1 |
| 5 | `speed=0, target=90` | 左轮反转、右轮正转（原地左转） | 反了 → 查 `MOTOR_LEFT_POLARITY` / 左右轮接线 |
| 6 | 手动推轮子前进 | `debug_enc_left/right` 为正 | 为负 → 翻 `ENCODER_SIGN_*`。**只翻编码器，千万别连电机极性一起翻**，两次反相会抵消，速度环变正反馈、PWM 顶到饱和 |
| 7 | 整定 PID | —— | 先把 `HEADING_KD` 置 0，把 `HEADING_KP` 加到开始摆头，再加 KD 压住 |
| 8 | 手动把黑线放到车体**右**侧，看 `debug_steer_deg` | 变成负值（目标航向往右偏） | 符号反了 → 翻 `TRACK_REVERSE_ORDER`；绝对值太小 → 加大 `TRACK_STEER_KP_DEG` |
| 9 | 推车走一段已知距离，看 `debug_odometer` | 单调增大（正 = 前进） | 为负 → 翻 `ENCODER_SIGN_*`（仍然只翻编码器）。然后用 counts/mm 换算 `CAR_ODOMETER_STOP_COUNTS` |

## 验证与构建

在 CarProject 目录执行：

```powershell
./Test/run_tests.ps1
./Test/run_motor_tests.ps1
./Test/run_module_tests.ps1
./Test/run_ultrasound_tests.ps1
./Test/run_heading_tests.ps1
./MDK/build_hardware.ps1
```

主机测试需 GCC；构建脚本使用本机 Keil ARMCC 工具链，可通过 `-ToolchainBin` 指定安装路径。
脚本读取 Keil 工程中的源文件、宏和包含目录，生成独立的 `Build/HardwareCLI` 输出。
2026-09-20 全部测试及完整 ARM 编译链接通过；AXF/HEX 已生成，但不是完成实车标定的比赛固件。
主机测试通过模拟 UART 中断输入真实协议帧，覆盖两种极性和两种安装方向；未进行实物 UART 或跑车验证。

2026-09-23 加入编码器、MPU6050 和航向模式后重新验证：

- PlatformIO 完整编译链接通过（`pio run`），Flash 14748 字节 / 65536（22.5%），RAM 1536 / 20480（7.5%），无警告。
- `run_motor_tests`、`run_tests`、`run_ultrasound_tests`、`run_heading_tests` 全部通过。
- 新增 `run_heading_tests.ps1`：用带测试钩子的传感器替身接一个差速小车模型跑闭环，覆盖转向符号、正负角度收敛、死区、速度环稳态误差、直行、IMU 故障停车、复位语义。它经反向验证：故意把 `HEADING_GYRO_SIGN` 改成 +1 会导致航向发散并断言失败，证明测试真的能抓住符号错误。

2026-09-23 转向统一改造后重新验证（本机仍无 GCC，用工程内 zig/clang 代替 `gcc`；需要 `-Wno-deprecated-non-prototype` 绕过 CMSIS V1.30 的旧式声明。全部源文件已在同一天统一为 UTF-8，因此不再需要 `-Wno-invalid-utf8`）：

- `pio run` 完整编译链接通过，Flash 15640 字节 / 65536（23.9%），RAM 1592 / 20480（7.8%），无警告。
- 五套主机测试全部通过（四组极性/安装方向、四组电机极性在内）：状态机测试新增「模块只产出命令、航向环是唯一写电机者」「里程到阈锁存停车」「IMU 未就绪拒绝启动」「运行中 IMU 失败→CAR_FAULT」「航向模式只调度 Heading_Update」；模块测试与航向环测试同步扩充。
- 上电预热确认为 **5 秒**（`TRACK_WARMUP_MS = 5000U`）。模块测试原来按「20 秒」断言（19999 ms 时不应发请求，只有 20000 才成立），已改为按固件实际值断言（4999 ms 不发请求、5000 ms 发一次）；代码注释与各文档里的 20 秒预热描述已同步改掉。模块自身 KEY 校准步骤里的 20 秒保留（那是模块手册的要求，与固件无关）。
- ⚠️ `run_module_tests.ps1` 在本机失败（`Test/ModuleHardware/test_modules.c:149`）。用改动前的 `CarControl.c` 和原始 TIM4 命名编译，失败位置与信息完全相同，因此是**既有问题**，与本次改动无关：本机没有安装 GCC，只能用工程内自带的 zig/clang 编译，浮点/整型计算路径与当初通过验证的 GCC 不同。装上 GCC 后应重新确认。

详细记录见 [VALIDATION.md](VALIDATION.md)。
