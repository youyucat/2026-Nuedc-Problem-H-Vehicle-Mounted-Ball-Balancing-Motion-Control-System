# 2026 NUEDC H 题 —— 车载滚球平衡运动控制系统

两块 MCU **完全独立运行，彼此不接线**，只在机械结构上共存；唯一的数据流是
摄像头 → 滚球板的单向串口，详见 `docs/protocol.md`。

## 目录结构

```
.
├── README.md                    本文件：整体架构与协作约定
├── .gitignore                   CCS / Keil / CubeIDE / Python 的忽略规则
├── .gitattributes               换行符与二进制文件规则
├── docs/
│   └── protocol.md              串口协议（唯一真相源）
├── mspm0_line_follow/           循迹板：MSPM0G3507
├── stm32f411_ball/              滚球板：STM32F411
└── vision/
    └── maixcam/                 摄像头：小球检测 + 串口发帧
```

## 三块硬件的分工

| 名字 | 型号 | 职责 | 工程目录 | 负责人 |
| --- | --- | --- | --- | --- |
| 循迹板 | MSPM0G3507 | 8 路灰度 → 循迹 PD → 差速 → 轮速 PI → TB6612 双直流电机 | `mspm0_line_follow/` | youyucat |
| 滚球板 | STM32F411 | 小球位置采集 → 导轨闭环 → 步进电机维持平衡 | `stm32f411_ball/` | jove061 |
| 摄像头 | MaixCam | 管道标定 + 钢球检测 → 12 字节帧 | `vision/maixcam/` | 共用 |

**循迹板与滚球板之间不接任何信号线。** 曾经考虑过把车体加速度发给滚球板做前馈，
已评估取消（滚球板串口接收脚被摄像头占用，且把车开稳收益更大），
决策记录见 `docs/protocol.md` 第 1 节。

由此产生一个硬性约束：**循迹板开得越平稳，小球越稳**，
因为没有任何跨板补偿能救急。量化分析见 `docs/protocol.md` 第 2 节。

视觉侧部署与标定见 `vision/maixcam/README.md`。**注意摄像头 WiFi 密码放在
`secret_config.py`（已被 gitignore），不要直接写进 `CONFIG` 后提交。**

## 编译

### 循迹板（MSPM0G3507）

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

### 滚球板（STM32F411）

用 Keil MDK（编译器 AC6 / armclang）打开 `stm32f411_ball/MDK-ARM/MOTER.uvprojx`，
直接 Build：0 错误 0 警告，约 25.8KB（MDK-Lite **32KB 上限，余量约 6KB，加功能要盯住**）。
工程由 STM32CubeMX 生成（`stm32f411_ball/MOTER.ioc`）。控制方案、引脚与串口接口详见
`stm32f411_ball/README.md`，上车测试流程见 `stm32f411_ball/TESTING.md`。

> 注：滚球板只接一路串口——**摄像头的 12 字节二进制帧**，帧格式见 `docs/protocol.md` 第 3 节。
> 固件里另有一套 ASCII 行帧解析（`A`/`R`/`M`/`Z`/`B`），目前无人接线，
> 仅作台架调试与未来扩展保留，见该文档第 4 节。

## 协作约定

### 分支

- `main` 是主干。**目前尚未开启 GitHub 分支保护**，技术上可以直接 push —— 但请不要这样做。
  等协作节奏稳定后，在 Settings → Branches 勾上「Require a pull request before merging」，
  开了之后连管理员也不能直推，规则才真正生效
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
