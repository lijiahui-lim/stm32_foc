# stm32_foc

基于 STM32F103 的有感 FOC（磁场定向控制）无刷电机固件：MT6701 磁编码器 + DRV8313/MS8313 三相驱动 + INA199 电流采样，Cortex-M3 无 FPU，控制核心为 Q15 定点实现。

- 原理与代码逐步实现讲解：https://blog.csdn.net/qq570437459/category_12672491
- 适配硬件：https://item.taobao.com/item.htm?ft=t&id=839302894720
- 后继版本（无感 FOC / VF、IF 启动 + 滑模 + PLL，架构更优）：https://gitee.com/best_pureer/stm32_foc_2

## 特性

- 8 kHz 电流环 / FOC：Clarke/Park、电流 PI、反 Park、SVPWM 全部 Q15 定点
- 1 kHz 速度环、位置环（浮点），支持 位置 → 速度 → 电流 串级控制
- 5 kHz MT6701 角度采样（SPI + DMA + CRC 校验，丢失帧自动判故障）
- 连续时间 PID 调参接口：Ki 单位为每秒、Kd 单位为秒，按各环实际频率自动离散化，改控制频率不影响积分/微分强度
- 完善保护链：电流零点自检、瞬时过流、持续过流（200 ms RMS）、编码器失联、堵转、DRV8313 nFAULT 硬件刹车锁存
- 常用占空比上下端各保留 10% 采样窗口，SVPWM 超限时等比例压缩保持矢量方向

## 硬件

| 项目 | 参数 |
| --- | --- |
| MCU | STM32F103C8T6 @ 72 MHz（无硬件浮点） |
| 预驱 | MS8313 / DRV8313PWPR，PB12 接低有效 nFAULT（TIM1 Break） |
| 电流采样 | 两相 20 mΩ 分流电阻 + INA199A1（×50），1.65 V 偏置，量程约 ±1.65 A |
| 编码器 | MT6701，SPI，14 bit 机械角度 |
| PWM | TIM1 中心对齐 40 kHz，TRGO 触发 ADC 注入同步采样 |
| 供电 | 强电 DC12V（VM）；逻辑由右侧 5V（Type-C）经 ME6206 转 3.3V。板子不对外提供 5V，防止损坏 USB |

> 注意：电流环增益按 12V 母线整定，且无母线电压补偿。改用 24V 供电时等效环路增益翻倍，校零/启动阶段易触发过流或驱动器保护，需将 `set_motor_pid` 中两组电流 PI 增益按 12/24 比例缩放（如 Kp 1.2→0.6、Ki 160→80）。

## 编译与烧录

```bash
make          # 需要 arm-none-eabi-gcc；优化等级至少 -O2
```

也可使用 MDK-ARM 工程（需 ARM Compiler V6）。上板前先实测确认电流采样、PWM 频率与 nFAULT 脚位波形。

## 启动流程

1. 等待 DRV8313 nFAULT 连续 10 ms 为高（上电瞬态会短暂拉低）
2. 等待 MT6701 首帧有效角度，确认转子静止 100 ms
3. ADC 自校准后开 PWM（三相占空比 0），256 点采样校正两路电流零点，偏置接近电源轨则报故障
4. alpha 轴限流校零 400 ms（`MOTOR_ALIGN_CURRENT_A` = 0.30 A），记录转子零位
5. 进入限流速度环，目标速度 1.57 s 内由 0 升到 1200 RPM（约 80 rad/s²）

## 电流与保护参数（`Drivers/motor/conf.h`）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `MAX_CURRENT` | 1.5 A | 标幺基准，对应采样量程上限 |
| `MOTOR_ALIGN_CURRENT_A` | 0.30 A | 校零目标电流 |
| `MOTOR_RUN_CURRENT_A` | 0.50 A | 正常运行 q 轴电流指令上限 |
| `MOTOR_SUSTAINED_CURRENT_A` | 0.60 A | 200 ms 平滑过流保护阈值 |

请按电机额定连续电流与实际温升调整。

## 故障码

USART2 每约 200 ms 输出：

```
speed_mrad_s=<速度> iq_mA=<q轴电流> id_mA=<d轴电流> fault=<故障码> frames=<编码器帧数>
```

| fault | 含义 |
| --- | --- |
| 1 | 编码器失联（MT6701 超过 1.5 ms 无有效帧） |
| 2 | 瞬时过流（任一相超过 ±1.5 A 标幺上限，连续 3 个采样） |
| 3 | 堵转 / 未达到目标速度（低于目标一半持续 2 s） |
| 4 | 控制输出非法（占空比非有限值或越界） |
| 5 | 电流零点异常（偏置接近电源轨） |
| 6 | 持续电流过高（约 200 ms RMS 超 0.6 A） |
| 7 | 驱动器故障（nFAULT 拉低，触发 TIM1 硬件刹车并锁存） |

停机后需排除原因并重新上电。

## 运行监控

只接 SWD 下载器（不占用串口、不重置电机）：

```bash
python3 tools/monitor_speed.py
```

每 200 ms 从 ELF 符号表定位 `motor_speed`/`motor_fault` 地址并实时读取转速（rad/s 与 RPM）及故障码，Ctrl+C 结束。

## 实测性能（72 MHz Cortex-M3）

| 项目 | 数值 |
| --- | --- |
| FOC / 电流环 | 8.0 kHz |
| ADC 中断耗时 | 平均约 1,396 cycles（19.4 µs），最长约 43.9 µs |
| MT6701 角度/速度更新 | 5.0 kHz |
| 主要中断合计 CPU 占用 | 约 43%，余量约 57% |
| 1200 RPM 目标实测 | 1,191–1,209 RPM，平均约 1,200 RPM |

角度更新率 5 kHz 是高速下的主要数字限制：7 极对电机若保留每电周期 20 次角度更新，建议机械转速 ≤ 2,140 RPM；10 次更新的上限约 4,280 RPM。

## 许可证

GPL-3.0，见 [LICENSE](LICENSE)。
