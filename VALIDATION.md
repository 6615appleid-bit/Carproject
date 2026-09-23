# 验证记录

## 2026-09-20 更新

- Hardware 分组已加入 Avoid.c、track.c、Ultrasound.c 和新 Motor.c；uvprojx 与 uvoptx 工作树已同步。
- Motor.c 使用已找到的本机 Arm Compiler 5.06 成功编译；主机电机测试（四种方向极性配置）及原整车调度测试通过。
- 后续完成 Track/Avoid 封装、八路 USART2 接入和超声波异步采集，已消除 Car.h、app_irtracking.h、Delay.h 依赖。`MDK/build_hardware.ps1` 完整 ARM 编译链接通过，生成 AXF/HEX；ROM 14108 字节，RW+ZI 1896 字节。
- 实际 Track/Avoid/CarControl 联合测试及 Ultrasound 状态机/ISR 主机测试通过。硬件中断脉宽和真实运动尚未实测。
- 已根据新提供的官方串口说明和现有电路图配置 PA2/PA3、115200 8N1，取消旧空 GPIO 表。上电等待 20 秒后请求八路数字流，按新帧确认稳定性，超过 150 ms 无有效新帧判无效。
- UART 协议/中断测试覆盖预热、命令重试、错误帧、错误标签/值、截断、超长、整帧超时、重新同步、硬件接收错误、重复帧读取不累计、迟处理不刷新时效、断线停车和计时回绕；两种极性×两种安装方向共四组通过。
- Keil 工程的 USART2_IRQHandler 唯一实现来自 track.c；TIM4/EXTI0/EXTI1 来自 Ultrasound.c；SysTick 来自 Platform_stm32.c。没有引入原厂 main 或其他中断文件。未烧录，模块校准、实际左右安装、电平和动作效果尚待实物验证。
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
