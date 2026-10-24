# stm32f411_ball —— 芯片 B：STM32F411 摆杆稳球固件

H 题要求 4（行驶中把钢球稳在摆杆中心 O，误差 ≤ 1cm）中"摆杆"这一半的实现：
小球位置闭环 + 摆杆步进驱动。视觉由 MaixCam 负责（二进制帧直连本芯片），
循迹行驶由芯片 A（`mspm0_line_follow/`）负责。

## 控制方案

**滑模 PD + 回零偏学习 + 静摩擦脱困脉冲 + 车体加速度前馈**（100Hz），
移植自成熟的 MSPM0G3507 ball_balance 参考实现，符号已适配本机机械约定
（本机 +角度 = 摆杆 +x 端/画面左端抬高，与参考工程机械方向相反）：

| 环节 | 说明 |
| --- | --- |
| 测速 | 位置 2 帧差分 / 实测帧周期 + EMA（相机速度字段偏小，不用） |
| PD | `u = +KP·e_pred + KD·v`，e = 球相对目标偏差，位置前推 120ms |
| 限幅 | 偏差 >40mm 放宽 ±14° 大幅回正，近端 ±10° 防过冲 |
| 偏置学习 | 球准静止 150ms 后慢速积累常驻倾角（±3°），替代积分消偏 |
| 脱困脉冲 | 球顶住 200ms 后发斜坡-保持-卸力脉冲（近端 ≤5°、远端 ≤9°） |
| 方向约束 | 只朝目标推球；球速 >8mm/s 滚向目标时允许同侧刹车 |
| 前馈 | 芯片 A 的加速度帧线性映射 ±5°，绕过方向约束直接叠加 |
| 丢球 | 保持最后指令 600ms → 12°/s 回平，**任务不退出**，球回视野自动恢复 |

所有可调参数集中在 `Core/Inc/ball_ctrl.h`；现场整定与上车测试流程见
[TESTING.md](TESTING.md)（六阶段：上电自检 → 方向标定 → 视觉链路 → 静止
稳球 → 前馈验证 → 上车联调）。

## 编译（Keil MDK）

1. Keil MDK（编译器 AC6 / armclang）打开 `MDK-ARM/MOTER.uvprojx`，直接 Build。
   期望：0 错误 0 警告，约 **25.8KB**（STM32F411RE Flash 512KB，MDK-Lite 32KB 限制内）。
2. `MDK-ARM/MOTER/MOTER.sct` 为构建时自动生成的 scatter，未入库；
   CubeMX 重新生成工程时也会自动重建。
3. 若用 CubeMX 重新生成代码（`MOTER.ioc`），用户代码都在 `USER CODE` 区，
   `stepper/ball_ctrl/key/encoder` 为独立文件不会被覆盖。

| 组件 | 版本 |
| --- | --- |
| IDE / 编译器 | Keil MDK 5.x，AC6（armclang 6.24） |
| 生成工具 | STM32CubeMX（`MOTER.ioc`） |
| 时钟 | HSI 16MHz → PLL → 100MHz |

## 引脚

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| ST1 | PA8 (TIM1_CH1) | 步进脉冲 STEP，固定 10µs 脉宽，伺服微动跟踪 |
| DIR1 | PB1 | 方向 |
| EN1 | PB0 | 驱动器使能（低有效，上电默认失能） |
| KEY | PA0 | 唯一按键：短按启动稳球闭环，运行中长按 1s 停止回平 |
| USART1 TX/RX | PA9 / PA10 | 调试打印 115200 / MaixCam 帧 + 芯片 A 帧 |

步进 1600 步/圈（1.8° 电机 × 8 细分），驱动器拨码必须与 `Core/Inc/stepper.h`
的 `STEPPER_STEPS_PER_REV` 一致。驱动器光耦为 5V 输入时改共阳接法。

## 串口接口（当前实现）

- **MaixCam → 本芯片（PA10）**：12 字节二进制帧
  `AA 55 | type=1 | seq | flags | pos(int16 LE, 0.1mm, 画面左=正) | vel | conf | CRC16-CCITT`，
  20ms 恒发。帧格式细节见 `Core/Src/ball_ctrl.c` 头部注释。
- **芯片 A → 本芯片**：ASCII 过渡帧 `A,<加速度 mm/s²>`（行驶前馈用）；
  另有 `M,<mdeg>` 手动摆角、`R,<0.1mm>` 球目标（要求 6）、`Z` 水平清零。
- **本芯片 → 芯片 A（PA9，可选监听）**：`W1` 丢球减速 / `W0` 重捕获 / `W2` 回平告警。

> ⚠️ `docs/protocol.md` 的双芯二进制协议目前是**草案**，本固件尚未对接；
> 协议定稿后需改 `ball_ctrl.c` 的帧解析（UART 环形缓冲与帧状态机已就位，
> 只需替换分发逻辑）。视觉与芯片 A 目前共线复用 PA10（串 1k 电阻线与），
> 条件允许时建议分线。

## 目录

| 路径 | 内容 |
| --- | --- |
| `Core/Src/ball_ctrl.c` | 稳球控制器：滑模 PD / 偏置学习 / 脱困脉冲 / 前馈 / 帧解析 |
| `Core/Src/stepper.c` | 步进驱动：斜坡加减速、伺服微动、零多余脉冲停机 |
| `Core/Src/key.c`、`encoder.c` | 按键消抖 / 编码器（已退役保留） |
| `TESTING.md` | 上车测试手册（含整定表与丢球行为说明） |
| `MOTER.ioc` | CubeMX 工程：TIM1/2/4 + USART1 + 11 个 GPIO |
