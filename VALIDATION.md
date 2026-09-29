# 验证记录

## 2026-09-25 丢线搜索（脱线后自主找线）

需求：丢线后不再一律停车。只有**上一次有效检测已经严重偏离**（`|error| ≥
TRACK_SEARCH_MIN_ABS_ERROR`，默认 800）时，丢线才沿最后一次的修正方向继续转弯找线
（车最后偏右 → 继续左转，最后偏左 → 继续右转），找到线立刻恢复正常循迹；
超过 1 秒没找到就停止搜索。轻微偏离（如 00000110，|error| ≈ 571）、线居中、
以及数据无效时不找线，直接停车。

- `Hardware/track.c`：新增 `TRACK_SEARCH_*` 参数与 `SearchStep()`。`Track_Update()` 在
  NORMAL/WIDE 之外不再直接输出 0 速度，而是先判「严重偏离」门槛，通过后给出
  「持续旋转的目标航向角 + 搜索速度」（转角按 1.0 度/步 ≈ 100°/s 推进，
  累计 ±150 度与总时长 1000 ms 双重限幅）。
  偏离程度记忆 `last_turn_error`（最后一次有效偏差，含符号；居中帧会把它清成 0）
  只在 `Track_Init` 清零，方向取它的符号、门槛比它的绝对值。
- `Track_Reset()` 拆成 `ResetControl()`（清航向基准 / 偏差微分 / 速度斜坡）与 `ResetSearch()`：
  `Track_SenseUpdate()` 里三处原本的 `Track_Reset()` 全部改为 `ResetControl()`，否则每丢一帧
  都会把搜索状态清零，搜索会不断重新开始、永远到不了超时。
- `App/CarConfig.h`：`CAR_TRACK_LOSS_TOLERANCE_MS` 400 → 1300 ms（必须大于搜索超时 1000 ms）。
- `App/CarControl.c`：丢线分支不再直接 `Heading_Stop()`，改为转发 Track 的搜索命令；
  只在命令速度为 0（数据无效 / 没到门槛 / 搜索超时）时才停车，窗口耗尽仍锁存 `CAR_ERROR_TRACK`。
- `Test/Mock.h` + `Test/Track_Mock.c`：新增丢线搜索命令钩子（默认 0/0 = 不搜索），
  状态机测试据此验证「丢线期间整车照常把命令转交给航向环」。
- 测试更新：`test_modules.c` 新增「严重偏离（0x80/0x01 → |error| = 1000）两个方向都会搜索」、
  「轻微偏离（0x40 → 714）不搜索」、「居中不搜索」、「1 秒超时」以及整车联调用例
  （先偏到最外侧一路再丢线，车继续转向）；`test_car_control.c` 新增搜索命令转发断言。

验证结果：

- `pio run` 通过：Flash 16000 / 65536（24.4%），RAM 1616 / 20480（7.9%），无警告。
- `Test/run_tests.ps1`、`Test/run_module_tests.ps1`（四组配置）、`Test/run_heading_tests.ps1`、
  `Test/run_motor_tests.ps1`、`Test/run_ultrasound_tests.ps1` 全部通过。
- 未烧录、未实车验证。门槛 800、搜索速度、旋转速率、1 秒限时都需要实车标定。

## 2026-09-23 转向统一改造

需求：转向一律「直接定航向角、间接控制车轮差速」。改造后循迹、避障、航向三种模式都只输出
「目标航向角 + 目标速度」（`DriveCmd`），差速统一由 `Hardware/Heading.c` 的转向环产生；
并加入里程到阈自动停车（`CAR_ODOMETER_STOP_COUNTS`，默认 100000 计数）。

- 接口：新增 `Heading_Drive()` / `Heading_Stop()` / `Heading_ResetOdometer()` 与 `DriveCmd`（`Drivers/Heading.h`）；
  `Track_Update()` / `Avoid_Update()` 不再写电机，改为 `Track_GetDrive()` / `Avoid_GetDrive()` 输出命令；
  CarControl 是唯一的调度者与命令转发者，并新增 `CarControl_GetDrive()` 供调试观察。
- 循迹：偏差 → 航向修正角（`TRACK_STEER_KP_DEG` 默认 22°/满幅，限幅 ±45°）+ 航向基准慢速积分（`TRACK_HEADING_TRIM_*`）；
  速度规划从 PWM 改为「计数/10ms」（50 → 25，宽黑上限 25）。
- 避障：三段固定时间动作改成「转出 45°（航向到位）→ 直行 900 计数（里程到位）→ 转回基准」，
  寻线阶段仍然用偏差修正航向，回线判定额外要求航向回到基准 ±25° 内，并有每段超时兜底。
- 惯性单元成为硬依赖：`CarControl_Start()` 要求 `Heading_IsImuReady()`，运行中 `Heading_HasFault()` 立即转 `CAR_FAULT`。

验证结果（本机无 GCC，用工程内 zig/clang；需 `-Wno-deprecated-non-prototype` 绕过 CMSIS V1.30 旧式声明）：

- `pio run` 通过：Flash 15640 / 65536（23.9%），RAM 1592 / 20480（7.8%），无警告。改造前为 14748 / 1536。
- `Test/run_tests.ps1`（状态机，含新增用例）、`Test/run_module_tests.ps1`（真实 track/Avoid/CarControl，四组配置）、
  `Test/run_heading_tests.ps1`（新增 Drive/Stop/ResetOdometer 用例）、`Test/run_motor_tests.ps1`、
  `Test/run_ultrasound_tests.ps1` 全部通过。
- 编码统一：**全部源文件、文档、脚本、启动文件以及 `Libraries/` 下的标准外设库/CMSIS 副本均已从 GBK 转为 UTF-8（无 BOM）**，VS Code 下中文注释不再乱码；因此不再需要 `-Wno-invalid-utf8`。
  `Build/Before_*` 里的历史备份也一并转换。根目录的 nm/sections 导出本来就是 UTF-16（VS Code 可正常打开），未改动。
- 测试工程两处同步：`Test/ModuleHardware/test_modules.c` 现在需要 `-ITest`（包含 `Heading_Mock.h`），`run_module_tests.ps1` 已同步；`run_*.ps1` 里的 `gcc` 调用未变（本机无 GCC，本地验证时用工程内 zig/clang 代替）。
- 上电预热按实际固件统一为 **5 秒**：模块测试改为「4999 ms 不发模式请求、5000 ms 发一次」，代码注释与各文档里的「20 秒预热」描述一并改掉（模块自身 KEY 校准步骤里的 20 秒是模块手册要求，保留）。
- 未烧录、未实车验证。新增的角度、里程、速度增益全部是待标定起点；
  符号类项（`HEADING_GYRO_SIGN`、`ENCODER_SIGN_*`、`MOTOR_*_POLARITY`、`TRACK_REVERSE_ORDER`）必须按 README 自检表逐项实测。

## 2026-09-20 更新

- Hardware 分组已加入 Avoid.c、track.c、Ultrasound.c 和新 Motor.c；uvprojx 与 uvoptx 工作树已同步。
- Motor.c 使用已找到的本机 Arm Compiler 5.06 成功编译；主机电机测试（四种方向极性配置）及原整车调度测试通过。
- 后续完成 Track/Avoid 封装、八路 USART2 接入和超声波异步采集，已消除 Car.h、app_irtracking.h、Delay.h 依赖。`MDK/build_hardware.ps1` 完整 ARM 编译链接通过，生成 AXF/HEX；ROM 14108 字节，RW+ZI 1896 字节。
- 实际 Track/Avoid/CarControl 联合测试及 Ultrasound 状态机/ISR 主机测试通过。硬件中断脉宽和真实运动尚未实测。
- 已根据新提供的官方串口说明和现有电路图配置 PA2/PA3、115200 8N1，取消旧空 GPIO 表。上电预热后请求八路数字流，按新帧确认稳定性，超过 150 ms 无有效新帧判无效。
- UART 协议/中断测试覆盖预热、命令重试、错误帧、错误标签/值、截断、超长、整帧超时、重新同步、硬件接收错误、重复帧读取不累计、迟处理不刷新时效、断线停车和计时回绕；两种极性×两种安装方向共四组通过。
- Keil 工程的 USART2_IRQHandler 唯一实现来自 track.c；TIM3/EXTI0/EXTI1 来自 Ultrasound.c；SysTick 来自 Platform_stm32.c。TIM2/TIM4 由编码器以接口模式使用（不产生中断）。MPU6050 走软件 I2C 轮询，PB5/INT 和 EXTI9_5 均未使用。没有引入原厂 main 或其他中断文件。未烧录，模块校准、实际左右安装、电平和动作效果尚待实物验证。
- 详见 [Hardware 接入记录](Hardware/README.md)。以下为 2026-09-16 的历史记录。

日期：2026-09-16

## 已完成

- 使用本机MinGW GCC，以C99、`-Wall -Wextra -Werror -pedantic`编译实际CarControl及模拟模块，通过。
- 本次重新运行 `Test/run_tests.ps1`，所有断言通过。覆盖：10 ms调度、正常循迹、连续4次避障、绕行及寻线阶段不凭NORMAL提前回切、更新前已为DONE和更新中产生DONE两种交接、交接周期停车、电机控制互斥。
- 新交接测试检查：读取DONE不清除状态、交接复位模块、清除循迹控制历史而保留感知结果、下一周期恢复循迹且不重复复位。模拟避障只注入DONE，不验证真实的稳定回线算法。
- 同一测试还覆盖：数据无效、循迹丢线、WIDE不误判终点、避障启动失败、运行中失败、避障总超时、32位毫秒时间回绕、手动停止/重新启动、终点和故障锁存。
- Keil XML可解析，40个构建组文件路径均存在且指向本工程内部。
- 核对Keil配置：STM32F103C8、STM32F10X_MD、USE_STDPERIPH_DRIVER、64 KB Flash、20 KB RAM、medium-density启动文件。
- 使用GCC `-fsyntax-only`与本地STM32标准库头文件，检查main、Platform_stm32和system_stm32f10x的C语法，通过；此项不生成ARM机器码。

## 未完成及限制

- 未执行Keil/Arm Compiler目标编译和链接。在PATH及检查过的常见安装位置没有找到可用Keil可执行文件；不能据此断言电脑未安装其他位置的Keil。
- 未烧录、未连接实车验证，没有可交付的已验证ARM固件。
- 当前Track/Avoid/Motor均为模拟实现，未接入真实红外、超声波、PWM或编码器。
- 模块负责人需遵守更新后的接口：Avoid在寻线阶段确认稳定回线后锁存DONE；Track_Reset只清除控制历史，保留有效感知结果。
- 板载晶振是否8 MHz仍需核对；未验证实际时钟、引脚复用和电机制动行为。
- 起始横线处理、终点识别、真实回线稳定性和传感器故障策略需实车完成。

结论：软件协调框架通过本机模拟测试，Keil工程及本地依赖已建立；尚不能认定为可上车比赛的完整系统。
