# STM32F103C8T6 循迹避障小车

本工程按设计方案划分整车调度、循迹和避障，使用 STM32 标准外设库。
当前真实模块已完成软件接入和 ARM 编译链接，尚未实车验证。
**已按现有电路图和模块官方串口示例配置 USART2：PA2→模块 RX、PA3←模块 TX，115200 8N1。**
上电先非阻塞等待 20 秒，再请求八路数字数据；收到稳定有效帧后允许启动。

## 打开工程

打开 `MDK/CarProject.uvprojx`，目标选择 `CarControl_Hardware_STM32F103C8`。
使用 Arm Compiler 5.06、STM32F103C8、medium-density 启动文件、64 KB Flash / 20 KB RAM。
时钟沿用原工程的 8 MHz 外部晶振、72 MHz 系统时钟，需与实物一致。
若 Keil 已打开此工程，重新加载以读取更新的 Hardware 分组。

## 模块职责

| 位置 | 职责 |
|---|---|
| App/CarControl.c | 每 10 ms 调度，决定控制权，处理停车、故障、避障总超时和 DONE 交接 |
| Hardware/track.c | USART2 初始化/收发/解析、八路黑白状态、稳定确认、时效和 PD 循迹控制 |
| Hardware/Avoid.c | 障碍判断、方向选择、非阻塞绕障、寻线和稳定回线确认 |
| Hardware/Ultrasound.c | TIM4 + EXTI0/1 异步回波采集，返回测距缓存 |
| Hardware/Motor.c | 原平衡车双电机接线，TIM1 PWM、方向、限幅和停车 |
| Modules / Drivers | 公共头文件，保留 Track/Avoid/CarControl 的原设计接口 |
| Platform | SysTick 毫秒时基 |
| Test | 主机测试，不加入 Keil 硬件目标 |

Track 与 Avoid 的 SenseUpdate 均不控制电机；只有当前调度到的 Update 输出运动。
Avoid 在寻线阶段确认稳定回线后锁存 AVOID_DONE。
CarControl 收到 DONE 后停车、Avoid_Reset、Track_Reset，下一周期恢复循迹。
Track_Reset 只清除控制历史，保留线路感知。

## 当前配置与边界

- 八路数据经 PA2/PA3 两根串口信号线传输，不再需要八个 GPIO 输入。黑线默认 0、白色背景默认 1，X1 默认在车体左侧；本车实测转向相反，`track.c` 中 `TRACK_REVERSE_ORDER` 已设为 1（若以后安装方向变回相反再改回 0）。
- 模块第一次使用或改变安装高度/环境后，需通过板载 KEY 完成黑白校准。操作与接线见 [八路循迹接线与联调](Hardware/八路循迹接线与联调.md)。上电等待不能替代校准。
- 原 IO_Track 的 PC0～PC5 是 ZE 板配置，未复制进 C8。无需再引入 app_irtracking.h、AllHeader.h 或旧 BSP。
- 电机及超声波接线、控制参数、采集时序详见 [Hardware 接入说明](Hardware/README.md)。
- 当前控制是开环 PWM，无编码器速度闭环。避障动作时长、速度和回线阈值需实车标定。
- main 上电后自动尝试启动：空闲时每 500 ms 调用一次受保护的 `CarControl_Start()`，在 20 秒预热和传感器数据就绪前会被拒绝。因此上电后先等待，数据可用后自动进入循迹，不需要调试器写入启动命令。
- `debug_command` 调试入口保留并覆盖自动启动：1 启动、2 停止、3 模拟已确认终点、4 重新初始化。命令 2 和 3 会关闭自动启动（便于锁定停车），1 和 4 重新打开。
- WIDE 只表示宽黑区候选，不作为终点；起始横线处理、正式启动按钮和终点判定尚未实现。宽黑单簇按质心继续循迹。短暂全白丢线改为有限时低速找线，回线确认期间继续找线，持续异常超过 400 ms 才锁存故障；数据失效仍立即停电机。参数与验证见 [短暂丢线恢复](说明文档/短暂丢线恢复.md)。
- 故障和完成状态锁存，Stop 不解除，非比赛复位调用 Init。

## 验证与构建

在 CarProject 目录执行：

```powershell
./Test/run_tests.ps1
./Test/run_motor_tests.ps1
./Test/run_module_tests.ps1
./Test/run_ultrasound_tests.ps1
./MDK/build_hardware.ps1
```

主机测试需 GCC；构建脚本使用本机 Keil ARMCC 工具链，可通过 `-ToolchainBin` 指定安装路径。
脚本读取 Keil 工程中的源文件、宏和包含目录，生成独立的 `Build/HardwareCLI` 输出。
2026-09-20 全部测试及完整 ARM 编译链接通过；AXF/HEX 已生成，但不是完成实车标定的比赛固件。
主机测试通过模拟 UART 中断输入真实协议帧，覆盖两种极性和两种安装方向；未进行实物 UART 或跑车验证。

详细记录见 [VALIDATION.md](VALIDATION.md)。
