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
- TIM1 使用 PSC=0、ARR=7199，72 MHz 定时器时钟下 PWM 为 10 kHz，与原工程一致。该驱动独占 TIM1，不占用超声波的 TIM4 或平台 SysTick。
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
- 每次 Track_Init 后非阻塞等待 20 秒；发送 `$0,0,1#` 请求数字数据。缺少新鲜数据时每秒重试一次，发送也通过中断完成，不使用忙等。
- 仅接受完整的 43 字节 `$D,x1:0,...,x8:0#` 数字帧（每位 0 或 1）；逐一验证 x1～x8 标签、分隔符、长度和值。拒绝模拟帧、截断/超长/非法帧；新 `$` 可重新同步，整帧接收时间超过 20 ms 则丢弃。
- 八路无黑点为 LOST，至少六路黑为 WIDE，分离黑点簇为 AMBIGUOUS，单簇连续 3 个有效新帧后为 NORMAL；累计稳定次数的帧间隔至少 10 ms。
- 超过 150 ms 没有新鲜有效帧返回 INVALID，数据恢复后重新确认；串口接收错误也使数据失效。时间戳取接收完成时刻，重复读取或晚处理不刷新数据年龄。
- 偏差按探头位置映射到 -1000～1000；负值在左，正值在右。控制基准需求 300，PD 系数 300/120，修正限幅 280。参数是待实车标定的起点，未照搬旧工程的速度单位。
- 每次控制更新上次误差，复位后第一次控制不产生微分突跳。Track_Reset 保留感知结果，只清除控制历史。
- 默认黑线为 0，X1 在车体左、X8 在右。本车实测转向相反，`track.c` 顶部 `TRACK_REVERSE_ORDER` 已设为 1（等价于镜像探头顺序）；对称位置权重确保中央 X4/X5 同时在线时偏差为 0。
- 新增只读 `Track_GetSampleId()` 供 Avoid 辨别新帧，保持 TrackStatus 结构及原控制接口不变；CarControl 无需读取帧编号。编号本身不代表有效性，必须结合线路状态使用。
- 协议没有校验和，语法检查不能检测所有位错误；软件也不能从数字帧判断模块是否已完成正确的黑白校准。接线和校准操作见 [八路循迹接线与联调](八路循迹接线与联调.md)。

### Avoid.c

- 测距判定、方向选择、绕障步骤、寻线修正和回线确认全部在此文件内。
- 15 cm 进入障碍判断，18 cm 解除，避免阈值附近反复切换；任一测距失效使检测结果 INVALID，执行期间距离小于 5 cm 则失败停车。
- BYPASSING 分为向空侧转弯 200 ms、前进 450 ms、向线路方向转弯 400 ms；只根据毫秒时钟推进，不使用 Delay。
- SEARCHING 最多 4 秒。必须连续 5 个新帧 NORMAL、偏差绝对值不超过 300 且障碍已解除，才停车并锁存 DONE。丢线、偏差过大、障碍或调度间隔过长清除确认计数；绕障阶段的线路观察不计入确认。
- 回线确认依赖 CarControl 每轮先 Track_SenseUpdate 再传入最新状态的调用顺序，内部检查 Track_GetSampleId；即使跨多个控制周期重复使用同一 UART 帧，也不会累计次数。
- SenseUpdate、Start、Reset 不写电机；DONE 由整车复位后才回到 IDLE。所有运动统一调用 Motor_SetDemand/Motor_Stop。
- 上述开环动作的时间和需求必须实车校准，不能仅凭软件测试认定轨迹可以绕过实际障碍。

### Ultrasound.c：仅保留底层采集职责

- PB10/PB11 为左右触发，PB0/PB1 为左右回波；沿用用户提供的 Ultrasound.h 接线。
- TIM4 作为 1 MHz 自由运行计数器，CC1 中断在约 20 us 后撤销触发；EXTI0/1 捕获上升、下降沿。不占用 SysTick、不控制电机。
- 两路交替测量，完成后静默 60 ms，35 ms 无完整回波判失效；不足 100 us、超过 30000 us 的脉宽无效。
- 完成时间按回波实际捕获时刻记录，超过 400 ms 失效；晚处理旧回波不会刷新其年龄。两路均有有效结果后才允许启动。
- Ultrasound_Update 由 Avoid_SenseUpdate 调用；原距离 getter 已改为只读缓存，不再同步触发测量。

## 完整构建和测试

2026-09-20 使用 `MDK/build_hardware.ps1` 按 uvprojx 中的源文件、宏和头文件路径，
调用本机 Arm Compiler 5.06、汇编器和链接器，成功生成
`Build/HardwareCLI/CarControl_Hardware.axf` 与 `.hex`。
链接按 C8 的 64 KB Flash / 20 KB RAM 限制检查：ROM 14108 字节，RW+ZI 1896 字节。
Keil 工程已启用 C99；脚本的输出目录独立于 IDE 的 Build 输出。

以下测试全部通过：

- `Test/run_module_tests.ps1`：实际 USART2 ISR、Track、Avoid 与 CarControl 联合测试，覆盖 20 秒预热、启动命令与重试、格式校验/缓冲边界/超时重同步、错误帧不刷新时效、接收错误、两种极性×两种安装方向、PD、复位保持感知、只统计新帧的稳定回线、DONE 锁存、断线停车及毫秒回绕。
- `Test/run_ultrasound_tests.ps1`：实际采集状态机和 ISR，覆盖触发撤销、左右回波、静默间隔、错误通道、超时恢复、短脉冲、卡高、迟到回波、旧结果时效、16 位计时与 32 位毫秒回绕。
- `Test/run_motor_tests.ps1` 和 `Test/run_tests.ps1`：电机与原调度回归测试。

没有烧录或实车验证。GPIO 电平、电机方向、传感器输出电压及动作参数仍需按实物核对。
main 仍为原调试命令入口，正式启动、起始横线和终点识别尚未接入。
原始三个模块和 Ultrasound.h 的备份位于 `Build/Before_module_integration/`。
本次串口改造前的 Track/Avoid/接口/模块测试备份位于 `Build/Before_uart_integration/`。
