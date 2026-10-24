# 2026 NUEDC H 题 —— 车载滚球平衡运动控制系统

双芯片协作方案：两颗 MCU 各管一半，机械解耦、外设不共享、通过串口交换状态。

## 目录结构

```
.
├── README.md                    本文件：整体架构与协作约定
├── .gitignore                   三套工具链（CCS / Keil / CubeIDE）的忽略规则
├── .gitattributes               换行符与二进制文件规则
├── docs/
│   └── protocol.md              双芯串口通信协议（唯一真相源）
├── mspm0_line_follow/           芯片 A：MSPM0G3507 —— 循迹
└── stm32f411_ball/              芯片 B：STM32F411 —— 滚球平衡
```

## 双芯分工

| 芯片 | 型号 | 职责 | 工程目录 | 负责人 |
| --- | --- | --- | --- | --- |
| A | MSPM0G3507 | 8 路灰度 → 循迹 PD → 差速 → 轮速 PI → TB6612 双直流电机 | `mspm0_line_follow/` | youyucat |
| B | STM32F411 | 小球位置采集 → 导轨闭环 → 步进电机维持平衡 | `stm32f411_ball/` | jove061 |

两颗芯片**不共享任何外设**，只在机械结构上共存；数据交互只走串口，帧格式见 `docs/protocol.md`。

## 编译

### 芯片 A（MSPM0G3507）

用 TI CCS Theia 打开 `mspm0_line_follow/` 目录，或在该目录下命令行编译：

```bash
cd mspm0_line_follow
make
```

产出 `Debug/line_follow_mspm0g3507.out`。

工具链版本必须一致，否则 SysConfig 生成的代码行为会不同：

| 组件 | 版本 |
| --- | --- |
| MSPM0 SDK | 2.11.00.07 |
| CCS | 20.5.1 |
| SysConfig | 1.26.2 |
| 编译器 | ti-cgt-armllvm 4.0.4.LTS |

`mspm0_line_follow/makefile` 顶部的 `SDK_DIR` / `COMPILER_DIR` / `SYSCONFIG_DIR` 是本机绝对路径，**安装位置不同请只在本地改，不要提交**。

### 芯片 B（STM32F411）

用 Keil MDK（编译器 AC6 / armclang）打开 `stm32f411_ball/MDK-ARM/MOTER.uvprojx`，
直接 Build：0 错误 0 警告，约 25.8KB（MDK-Lite 32KB 内）。工程由 STM32CubeMX
生成（`stm32f411_ball/MOTER.ioc`）。控制方案、引脚与串口接口详见
`stm32f411_ball/README.md`，上车测试流程见 `stm32f411_ball/TESTING.md`。

> 注：本芯片与 MaixCam 视觉、芯片 A 的通信目前用 ASCII 过渡帧，
> `docs/protocol.md` 二进制协议定稿后再对接（见 `stm32f411_ball/README.md`）。

## 协作约定

### 分支

- `main` 为受保护分支，**不允许直接 push**
- 每人从 `main` 拉自己的功能分支：`feature/<模块名>`，如 `feature/ball-closed-loop`
- 推分支 → 网页开 PR → 另一人 review → Merge
- 每天至少同步一次 `main`，别攒着

### 目录所有权

| 范围 | 谁能改 |
| --- | --- |
| `mspm0_line_follow/` | youyucat |
| `stm32f411_ball/` | jove061 |
| `docs/protocol.md` | 改动前必须双方确认 |
| `.gitignore` / `.gitattributes` / 根 README | 改动前先说一声 |

### 单点维护文件（冲突高发区，只由一人负责）

| 文件 | 负责人 | 说明 |
| --- | --- | --- |
| `mspm0_line_follow/line_follow_mspm0g3507.syscfg` | youyucat | GUI 生成的引脚/外设配置，两人同改必冲突 |
| `mspm0_line_follow/makefile` | youyucat | 源文件清单与工具链路径 |
| STM32 工程的 `.uvprojx` / `.ioc` | jove061 | 同上 |

### 提交信息

```
<type>(<scope>): <简述>

feat(mspm0): 灰度阈值自适应标定
fix(stm32): 修复串口帧校验越界
docs(protocol): 补充帧头定义
```

## 注意事项

- **不要提交编译产物**：`Debug/`、`Objects/`、`Listings/` 均已在 `.gitignore` 中
- **不要把另一个工程的根目录文件带进来**：两个工程各有自己的 `main.c`、`makefile`
- **不要用 `git add -f` 强行加入被忽略的文件**
- Keil 的 `*.uvguix.*`（窗口布局）是每人本机私有的，已忽略，**不要解除**
