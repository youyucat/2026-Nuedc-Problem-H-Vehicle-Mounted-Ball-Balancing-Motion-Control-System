# line_follow_mspm0g3507 — 循迹专用工程（双芯架构的「循迹芯片」）

从 `empty_LP_MSPM0G3507_nortos_ticlang`（循迹 + 步进球平衡一体机）里把
**循迹这一半**单独抽出来的独立 CCS 工程：只保留灰度循迹传感器、TB6612 双直流
电机、左右车轮编码器，以及完整的循迹控制链。

双芯分工：

| 芯片 | 职责 | 工程 |
| --- | --- | --- |
| **本工程（循迹芯片）** | 8 路灰度 → 循迹 PD → 差速 → 轮速 PI → 直流电机 | `line_follow_mspm0g3507` |
| 另一颗芯片 | 视觉/小球位置 → 步进闭环 → 维持小球平衡 | 另行开发，不在本工程内 |

两颗芯片**不共享任何外设**（本工程不再使用 TIMG0/TIMG8/TIMG12、步进 DIR/EN/Z、
MPU6050 的 I2C1、SSD1306、UART2 视觉口），只在机械上共存。

## 1. 保留 / 删除清单

保留（逐字来自源工程，只删注释里的无关说明）：

```
main.c                         入口：bringup → line_task_init → 5 ms 循环
app/line_task.{c,h}            循迹任务（由 app/template_app.c 的 running_step 抽出）
config/template_config.h       行为参数（已裁到只剩循迹相关宏）
control/line_controller.{c,h}  PD 线控制器
control/line_estimator.{c,h}   加权重心 → [-1000,1000] 线误差
drivers/line_sensor.{h}, line_sensor_mux_adc.c   灰度异步帧驱动
drivers/motor.{h}, motor_tb6612.c                TB6612 双电机驱动
drivers/encoder_speed.{c,h}                     编码器测速（x4 计数 → mm/s）
services/line_processing.{c,h}                  灰度归一化 / 判黑 / 标定数据
hal/                            hal.h + hal_time/hal_gray/hal_motor/hal_encoder/hal_system
bsp/board.c                     灰度 MUX+ADC、TB6612 PWM/方向、编码器组中断
bsp/board_profile.h             引脚/IOMUX/通道顺序映射
bsp/hal_system_mspm0.c          SysConfig 电源/时钟/ADC/SysTick bring-up
startup_mspm0g350x_ticlang.c    启动文件（与源工程同一份）
```

删除（属于另一颗芯片或上位机）：

- `control/ball_balance.*`、`control/stepper_closed_loop.*`、`drivers/stepper_*`、
  `hal/hal_stepper.h`、`bsp/hal_stepper_mspm0.c` —— **步进 + 球平衡**
- `control/imu_service.*`、`drivers/imu_mpu6050.*`、`hal/hal_i2c.h`、
  `bsp/hal_i2c_mspm0.c` —— IMU（I2C1）
- `control/oled_service.*`、`control/oled_scene.h`、`drivers/oled_ssd1306.*` —— OLED
- `drivers/uart_async.*`、`drivers/vision_uart.*`、`hal/hal_uart.h`、
  `bsp/hal_uart_mspm0.c`、`services/pc_protocol.*`、`services/boot_control.*` —— 串口 / 上位机调参 / BSL
- `services/parameter_store.*`、`hal/hal_flash.h`、`bsp/hal_flash_mspm0.c` —— Flash 参数区
- `app/template_app.*`、`control/oled_*`、`bootloader/`、`tests/`、`tools/`、
  `maixcam_*.py`、`app_linker.cmd`（不再需要 0x3100 的 Bootloader 重定位）

## 2. 控制链（5 ms = 200 Hz 一拍）

```
灰度 8 路（MUX 扫描 + ADC 中断异步帧，零阻塞）
  → 归一化/判黑（未标定时用缺省阈值 2047）
  → 加权重心 → line_error ∈ [-1000,1000]（右偏为正）
  → EMA(55%) + 死区(12)                       ← 抑制直线蛇形
  → PD（base 260 / Kp 0.12 / Kd 0.13）
  → 差速限幅 ±120 mm/s + 转弯减速(10%，下限 80)
  → 软起步斜坡(1 mm/s 每拍，停车立即)
  → 轮速增量 PI（Kp 0.55 / Ki 0.38，最小占空比 225‰）+ 左右配平(1070/930‰)
  → TB6612（TIMA1 双通道，8 kHz）
```

丢线策略：保持最后目标 80 ms → 左右交替摇头扫描（每 150 ms 换向，幅度随失线
时间从 60 线性增到 340 mm/s）→ 1500 ms 仍无线则停车。
堵转保护：单侧目标 ≥250 mm/s 且输出 ≥900‰ 而实测 <20 mm/s，持续 180 ms 进入
FAULT（电机断电，需 `line_task_reset_fault()`）。

## 3. 接口速查

```c
/* app/line_task.h */
void line_task_init(void);                    // 板级初始化 + 灰度预热 + 电机待机
void line_task_step(uint32_t now_ms);         // 主循环每圈调用，内部按 5 ms 限拍
void line_task_get_snapshot(line_task_snapshot_t *s);  // 调试器观察用
bool line_task_start(void);                   // 手动起步（上电 1 s 后自动调用一次）
void line_task_stop(void);
void line_task_reset_fault(void);
bool line_task_set_gains(int16_t base, int16_t kp_milli, int16_t kd_milli);
void line_task_get_gains(int16_t *base, int16_t *kp, int16_t *kd);
bool line_task_set_motor_trim(uint16_t left_permille, uint16_t right_permille);
```

`line_task_snapshot_t` 里有 `state / sequence / line_valid / line_error /
active_mask / sensor_error / left_target / right_target / left_output /
right_output / left_speed_mm_s / right_speed_mm_s` —— 断点或 Live Watch 里
看这一个结构就能判断"传感器对不对、控制有没有输出"。

底层接口（换板只动 `bsp/` 和 `.syscfg`）：

```c
hal_time_millis(); hal_time_delay_us(us);
hal_gray_enable(bool); hal_gray_sampling_begin(); hal_gray_sampling_take(v, &err);
hal_gray_positions(); hal_gray_output_index(addr);
hal_motor_set_pwm(permille_l, permille_r); hal_motor_set_left/right_direction(bool);
hal_encoder_init/reset/snapshot(...);
```

## 4. 接线表（与源工程一致，未改动）

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| 灰度 AD0/AD1/AD2 | PB10 / PB11 / PB12 | MUX 地址，低有效（`GRAY_ADDRESS_ACTIVE_LOW`） |
| 灰度 EN | PB13 | 使能，低有效 |
| 灰度 ERR | PB14 | 传感板报错（上拉输入） |
| 灰度模拟输出 | PA27 = ADC0_CH0 | 12 bit，单 MEM0 + 中断逐次读值 |
| TB6612 PWMA / PWMB | PB2 / PB3 | TIMA1_CCP0 / CCP1，8 kHz |
| TB6612 AIN1/AIN2 | PA14 / PA15 | 左电机方向 |
| TB6612 BIN1/BIN2 | PA16 / PA17 | 右电机方向 |
| TB6612 STBY | 硬件接 +5 V | 固件不驱动 |
| 左轮编码器 A/B | PB8 / PB9 | GPIOB 组中断，x4 |
| 右轮编码器 A/B | PB6 / PB7 | 同上 |

通道顺序修正（板级线序交叉）在 `bsp/board_profile.h` 的
`BOARD_SENSOR_OUTPUT_MAP` = `{7,5,6,4,3,1,2,0}`，探头物理位置
`BOARD_SENSOR_POSITION_MAP` = `{-3500…+3500}`（单位 0.1 mm 的名义值，只用于归一化）。

## 5. 构建

```bash
# 1) 生成 SysConfig 代码
C:/ti/sysconfig_1.26.2/sysconfig_cli.bat \
  -s C:/ti/mspm0_sdk_2_11_00_07/.metadata/product.json \
  --script line_follow_mspm0g3507.syscfg -o Debug/syscfg_build --compiler ticlang

# 2) 编译 + 生成 TI-TXT（也可直接 make all，makefile 会自动跑第 1 步）
C:/ti/ccs2051/ccs/utils/bin/gmake.exe -f makefile all
```

产物：`Debug/line_follow_mspm0g3507.out`（调试/烧录）、
`Debug/line_follow_mspm0g3507.txt`（TI-TXT，供离线烧录工具）。

**实测结论（2026-10-24，本机）**：`0 error / 0 warning`（编译 + 链接全干净；
SysConfig 只报 3 条 CLKOUT gate 提示，与源工程一致）。
FLASH 占用 `0x2920`（约 10.5 KB / 128 KB），SRAM `0x477`（约 1.1 KB / 32 KB）。

链接脚本由 SysConfig 生成（`.syscfg` 里 `genLinker = true`），程序从 `0x00000000`
开始 —— 不再沿用源工程 `app_linker.cmd` 的 `0x3100`（那是给 Bootloader 预留的），
所以本工程是**独立烧录**的固件，没有升级引导。

## 6. 与源工程的行为差异（3 处，都有原因）

1. **上电自动起步**：源工程要等上位机 `run_start`（还要求"已标定 + 看到线"）。
   本工程在 `TEMPLATE_STARTUP_HOLD_MS`（默认 1000 ms）后自动 `line_task_start()`。
   若上电时车不在线上，`s_had_valid_line` 为 false，目标速度保持 0 → 电机不转，
   直到灰度看到线才起步。要改回手动，把 `line_task_step()` 里那段自动 start 删掉即可。
2. **灰度预热帧放到开中断之后**：源工程在 `hal_system_critical_enter()` 区间里做
   `line_sensor_read()`，靠 ADC 中断完成的帧在关中断时永远等不到（白等 1e6 次）。
   本工程在 `main()` 里先 `hal_system_critical_exit()` 再 `line_task_init()`，
   预热帧真正有效。
3. **没有灰度标定界面**：源工程的标定靠上位机/按键，本工程没有串口也没有按键，
   `line_processing` 走"未标定"分支（阈值固定 2047、`normalized = 4095 - raw`）。
   要写死实测值，在 `line_task_init()` 里 `line_processing_init();` 之后加：

```c
static const uint16_t black[TEMPLATE_SENSOR_MAX_CHANNELS] = { /* 8 个通道实测黑场 */ };
static const uint16_t white[TEMPLATE_SENSOR_MAX_CHANNELS] = { /* 8 个通道实测白场 */ };
(void)line_processing_apply_points(black, white);
```

   （`apply_points` 会做"黑白差 ≥ TEMPLATE_CAL_MIN_SPAN"校验，不合法则不生效。）

## 7. 想加串口调参怎么办

本工程刻意不带串口。要临时看数据，两个不成负担的做法：

- **调试器**：Live Watch `line_task_get_snapshot()` 的结果，或直接在
  `line_task_step()` 里打断点看 `s_snapshot`；
- **要真串口**：从源工程拷 `drivers/uart_async.* + bsp/hal_uart_mspm0.c +
  hal/hal_uart.h`，`.syscfg` 加回 UART0（PA10/PA11，115200），
  `hal_system_bringup()` 补 `SYSCFG_DL_UART_0_init()` 和 RX 上拉那两行，
  然后自己打印 `s_snapshot` 即可。**不要**直接拷 `services/pc_protocol.c`
  —— 它同时依赖步进/球平衡/IMU/OLED，会拖回一整套已删除的模块。

## 8. 调参顺序建议

1. 先确认 `line_error` 符号：手推车让线偏右，`line_error` 应变正、左轮应加速；
   反了就把 `motor_tb6612.c` 的 `TEMPLATE_LEFT/RIGHT_MOTOR_INVERTED` 翻一下。
2. 直线跑偏 → 调 `TEMPLATE_DEFAULT_LEFT/RIGHT_MOTOR_TRIM_PERMILLE`（1070/930）。
3. 直线蛇形 → 提高 `TEMPLATE_LINE_ERROR_EMA_NUM`（55→70）或加大
   `TEMPLATE_LINE_ERROR_DEADZONE`（12→18）。
4. 弯道冲出 → 降 `TEMPLATE_DEFAULT_BASE_OUTPUT`（260→220）或提高
   `TEMPLATE_LINE_MAX_CORRECTION_MM_S`（120→150）。
5. 小球被甩 → 降 `TEMPLATE_LINE_RAMP_STEP_MM_S_PER_TICK`（1 已是保守值）、
   提高 `TEMPLATE_LINE_TURN_DECEL_FRACTION`（10→15）、降
   `TEMPLATE_LINE_TURN_MIN_RUN_MM_S`（80→60）。
