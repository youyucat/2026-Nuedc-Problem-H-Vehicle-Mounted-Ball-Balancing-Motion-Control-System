# 滚球平衡系统重构方案

> 状态：**方案待确认**，未开始实施。
> 范围：`stm32f411_ball/` 内部的球平衡控制部分，**不含** CubeMX 生成的 HAL 初始化代码。

## 1. 目标与非目标

### 目标

| # | 目标 | 验收方式 |
| --- | --- | --- |
| 1 | 把 `ball_ctrl.c`（1027 行 / 32 函数 / 约 50 个全局状态）拆成五个职责单一模块 | 单文件 ≤ 250 行 |
| 2 | 控制律做成可替换策略，PD 与 LQR **同接口**，不再是 `#if` 交织 | 新增策略不改动其它模块 |
| 3 | 监督逻辑（脱困/方向约束/限幅/丢球恢复）与控制律**分离** | 控制律文件内不出现限幅与脉冲逻辑 |
| 4 | `link` / `est` / `ctrl` 三层**不依赖 HAL**，可在 PC 编译 | `sim/` 下 `make` 通过 |
| 5 | 能在 PC 上用录制的视觉帧回放，对比 PD 与 LQR 输出 | 一条命令出对比曲线 |
| 6 | 状态可完整重置 | `*_reset()` 只做一次 `memset` + 必要的默认值 |

### 非目标

- 不改 `main.c`、不改 CubeMX 生成代码、不改 `.ioc`
- 不改对外 19 个函数签名（见第 2 节）
- 不改变通信协议（`docs/protocol.md` 第 3 节的 12 字节帧）
- 不在本次重构中调参或改控制策略行为

## 2. 兼容契约（硬约束）

`main.c` 调用了以下 19 个函数，**签名与语义必须保持不变**：

```
ball_ctrl_init / ball_ctrl_start / ball_ctrl_stop_level / ball_ctrl_tick_10ms
ball_ctrl_uart_poll
ball_ctrl_accel / ball_ctrl_angle_mdeg / ball_ctrl_ball_01mm / ball_ctrl_ball_ok
ball_ctrl_conf / ball_ctrl_crc_err / ball_ctrl_fault / ball_ctrl_fb_mdeg
ball_ctrl_ff_mdeg / ball_ctrl_frame_count / ball_ctrl_sat / ball_ctrl_vel_mms
ball_ctrl_vision_cal / ball_ctrl_xref_01mm
```

重构后 `ball_ctrl.c` 变成一个**薄壳**：只做转发，不含任何算法。

## 3. 目标文件结构

```
stm32f411_ball/Core/
├── Inc/
│   └── ball_ctrl.h            对外接口（不变）+ 参数集中区
├── Src/
│   └── ball_ctrl.c            薄壳：转调各模块，不含算法
├── link/                      不依赖 HAL ── 可在 PC 编译
│   ├── frame_bin.c/h          12 字节二进制帧解析 + CRC16
│   └── frame_ascii.c/h        ASCII 行协议（R/M/Z/B，A 帧已废弃）
├── est/                       不依赖 HAL
│   └── ball_est.c/h           EMA 滤波 / 帧差测速 / 跳变拒绝 / 重捕获
├── ctrl/                      不依赖 HAL
│   ├── ctrl.h                 策略接口
│   ├── ctrl_pd.c              KP·e + KD·v
│   └── ctrl_lqr.c             K1·e + K2·v + KI·∫e
├── app/
│   ├── ball_task.c/h          100Hz 调度 + 状态机（运行/丢球/重捕获）
│   └── ball_super.c/h         监督层：脱困脉冲、方向约束、角度限幅、斜率限幅
├── act/
│   └── stepper.c/h            步进驱动（复用现有，不改）
└── sim/                       PC 回放（gcc，不参与 Keil 编译）
    ├── replay.c
    └── Makefile
```

## 4. 核心设计

### 4.1 用 ctx 结构体替掉 50 个全局变量

每个有状态的模块自带上下文，函数吃指针：

```c
typedef struct {
    int32_t  x_ema_01mm;
    int32_t  x_hist[BALL_DIFF_WINDOW_MAX];
    uint8_t  hist_wr, hist_cnt;
    uint32_t last_frame_no;
    uint32_t frame_period_ms;
    int32_t  v_est_mms;
    uint8_t  ready;
} ball_est_t;

void ball_est_reset(ball_est_t *e);              /* memset + 默认值，一条命 */
void ball_est_push(ball_est_t *e, int32_t x01mm,
                   uint32_t frame_no, uint32_t now_ms);
int32_t ball_est_pos(const ball_est_t *e);
int32_t ball_est_vel(const ball_est_t *e);
```

好处：可单测（栈上放一个 ctx 就行）、可重置（不怕漏字段）、
同一套代码可以同时跑两个实例（对比实验时 A/B 并存）。

### 4.2 控制律策略接口

```c
typedef struct {
    const char *name;
    void    (*reset)(void *ctx);
    /* 只吃必要输入，不碰全局，不碰执行器 */
    int32_t (*step)(void *ctx, int32_t err01mm, int32_t v_mms,
                    uint32_t dt_ms, int32_t limit_mdeg);
    void   *ctx;
} ctrl_iface_t;

const ctrl_iface_t *ctrl_pd_create(void);
const ctrl_iface_t *ctrl_lqr_create(void);
```

选哪个由 `ball_ctrl.h` 的 `BALL_USE_LQR` 决定，**编译期**绑定：

```c
#if BALL_USE_LQR
static const ctrl_iface_t *s_ctrl = ...
#endif
```

保留编译期而非运行期切换，是为了不把两套代码都链接进固件（MDK-Lite 32KB 余量只有 6KB）。

### 4.3 监督层单独成层

现在这些逻辑都塞在 `control_update()` 里，重构后全部归 `ball_super.c`：

- 角度限幅（近端 ±10° / 远端 ±14° 的松弛）
- 方向约束（只朝目标推球，快速回中时允许同侧刹车）
- 静摩擦脱困脉冲
- 斜率限幅（60°/s，刹车 120°/s，回平 12°/s）
- 启动稳定窗、微调死区

**这是让 PD 与 LQR 能公平对比的前提**——两者共用同一套监督逻辑，差别只在反馈项。

### 4.4 数据流与节拍

```
视觉 20ms ─→ frame_bin ─→ ball_est ─┐
                                    ├─→ ctrl_step ─→ ball_super ─→ stepper
10ms tick ──────────────────────────┘
```

- `ball_est_push()` 只在收到合法视觉帧时调用（20 ms 一次）
- `ctrl_step()` 在 10 ms tick 里跑（100 Hz），速度用 `ball_est` 的最新估计
- 未检出帧不刷新丢球计时，位置/速度保持上次值（沿用现有语义）

## 5. 迁移映射（现有函数 → 新位置）

| 现有（`ball_ctrl.c`） | 新位置 |
| --- | --- |
| `crc16_ccitt` / `ball_ctrl_bin_parse` | `link/frame_bin.c` |
| `ball_ctrl_split_fields` / `line_dispatch` / `ascii_parse_byte` | `link/frame_ascii.c` |
| `filter_step` / `estimate_ball_velocity` / 跳变拒绝 | `est/ball_est.c` |
| `control_update` 的 PD/LQR 部分 | `ctrl/ctrl_pd.c` / `ctrl/ctrl_lqr.c` |
| `settle_bias_update` | `app/ball_super.c`（PD 模式的等效积分） |
| `stiction_recovery_output` | `app/ball_super.c` |
| `active_angle_limit` / 方向约束 / 斜率限幅 | `app/ball_super.c` |
| `ball_ctrl_tick_10ms` 的调度与丢球状态机 | `app/ball_task.c` |
| 所有 `ball_ctrl_xxx()` 公共函数 | `ball_ctrl.c` 薄壳转发 |

## 6. 行为基线（动手前必须先做）

重写会让现有台架验证过的行为失效，**必须能证明「重构前后等价」**，否则无法判断是改好了还是改坏了。

步骤：

1. 用**当前 main 的固件**（PD 模式）跑固定场景：球从 ±40 mm 处释放、静止 150 ms 后起步
2. 通过现有遥测记录每 10 ms 的 `err01mm, v_mms, out_mdeg`，存成 CSV 提交到仓库
3. 重构后用同一份输入回放，逐拍比对输出
4. **验收标准：PD 模式输出逐拍差异 ≤ 1 mdeg**（只允许取整误差）

基线数据放在 `stm32f411_ball/Core/sim/baseline/*.csv`，是重构期间唯一的裁判。

## 7. 分阶段实施（四个 PR，每步可回滚）

| PR | 内容 | 风险 | 验收 |
| --- | --- | --- | --- |
| A | 抽 `link/` + 搭 `sim/` 回放框架 | 低 | 基线对比通过，帧解析结果逐字节一致 |
| B | 抽 `est/` | 低 | 基线对比通过 |
| C | 抽 `ctrl/`，PD 原样搬家 + LQR 套接口 | 中 | 基线对比通过，`BALL_USE_LQR=0/1` 都能编译 |
| D | 抽 `app/` 监督层，`ball_ctrl.c` 变薄壳 | 中 | 基线对比通过，19 个接口行为不变 |

关键：**A 阶段先把回放框架搭起来**，后面三个 PR 每个都能自动验证，
而不是靠「感觉好像没坏」。

## 8. 风险

| 风险 | 应对 |
| --- | --- |
| 台架验证过的行为回归 | 第 6 节基线，逐拍比对 |
| MDK-Lite 32KB 上限（现 25.8KB） | 每次 PR 后记录 `.map` 里的 Code+RO 大小，超过 30KB 就停 |
| Keil 工程要逐个加文件，容易冲突 | A 阶段一次性把目录建好、`.uvprojx` 一次加到齐，后续 PR 不再动工程文件 |
| 他的代码同时在改 | 从最新 `main` 拉分支；重写期间约定他不动 `Core/Src/ball_ctrl.c` |
| 比赛时间紧 | 四个 PR 每个独立可回滚，随时可以停在任一阶段 |
