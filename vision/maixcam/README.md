# MaixCam 视觉 —— 小球检测与串口发送

MaixCam 上运行的小球识别程序：Pi 上跑 OpenCV 圆检测 + 管道标定，通过 UART1
每 20 ms 向滚球板（STM32F411）发送 12 字节二进制帧，同时提供网页/WebRTC 预览。

## 文件

| 文件 | 说明 |
| --- | --- |
| `maixcam_ball_detect.py` | 主程序（原 `maixcam_wifi_webrtc_cv2.py`） |
| `secret_config.example.py` | WiFi 配置模板，**提交的就是这个** |
| `secret_config.py` | 本地真实 WiFi 账号密码，**已被 .gitignore 忽略，不要提交** |

## 部署

1. 把 `maixcam_ball_detect.py` 和 `secret_config.example.py` 拷到 MaixCam
2. 在 MaixCam 上生成私密配置并填写 SSID/密码：

   ```bash
   cp secret_config.example.py secret_config.py
   vi secret_config.py        # 填 WIFI_SSID / WIFI_PASSWORD
   ```

   > ⚠️ **`secret_config.py` 必须单独存在于设备上。**
   > 主脚本里只有占位符，缺了这个文件会连不上热点。
   > 如果嫌每次在设备上重新填麻烦，就在本地填好 `secret_config.py`
   > 后把**两个文件一起**传上去 —— 它是被 .gitignore 忽略的，不会进仓库。

3. 运行：

   ```bash
   python maixcam_ball_detect.py
   ```

   未填 WiFi 会直接报错退出，不会带着占位符去连网。

4. 浏览器打开 `http://<MaixCam IP>:8000` 看实时画面与检测状态
   （WebRTC 信令端口 8001）。

## 硬件连接

| 项目 | 取值 | 代码位置 |
| --- | --- | --- |
| UART 设备 | `/dev/ttyS1` | `CONFIG["uart_device"]` |
| 引脚映射 | `A19 = UART1_TX`，`A18 = UART1_RX` | `VisionUartLink.__init__` |
| 波特率 | 115200 8N1 | `CONFIG["uart_baud"]` |
| 发送周期 | 20 ms（50 Hz） | `CONFIG["uart_period_ms"]` |

⚠️ **本程序只写串口，不读串口**（全脚本无 `readline`/`read`）。
由于滚球板的唯一串口接收脚被本程序占用，且摄像头不做转发，
**循迹板与滚球板之间已决定不接线**，详见 `docs/protocol.md` 第 1 节。

## 输出帧格式（12 字节，小端）

| 字节 | 内容 |
| --- | --- |
| 0-1 | 帧头 `0xAA 0x55` |
| 2 | 类型 = `1` |
| 3 | 序号，0→255 循环 |
| 4 | 标志位：bit0 检测到球，bit1 管道已标定，bit2 滤波器有历史 |
| 5-6 | 球位置 int16，单位 **0.1 mm**（画面左为正） |
| 7-8 | 球速度 int16，单位 mm/s（**仅供遥测，固件不直接采用**） |
| 9 | 置信度 0~100 |
| 10-11 | CRC16-CCITT（`0x1021`，初值 `0xFFFF`）**低字节在前**，覆盖字节 0~9 |

滚球板侧解析见 `stm32f411_ball/Core/Inc/ball_ctrl.h` 头部注释；
固件用「位置 2 帧差分 ÷ 实测帧周期」自己算速度，因为相机速度字段偏小。

## 标定流程（启动时自动）

1. **前 5 秒（250 帧）忽略**：给操作者时间把相机对准管道
2. **接着 5 秒（250 帧）统计**：估计管道的左右端点与宽度
3. 之后几何固定，只对「随摆杆移动的那一端」每 5 帧重新检测并 EMA 平滑
   （`mobile_end_side` 默认 `"right"`）

启动后**不要移动相机**，否则标定失效，需要重启。

## 关键参数（`CONFIG`）

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `roi_top_ratio` / `roi_bottom_ratio` | 0.40 / 0.50 | 只在这个水平条带内接受球心 |
| `min_radius` / `max_radius` | 7 / 22 | 钢球像素半径范围，换相机高度必须改 |
| `min_candidate_score` | 0.36 | 候选圆综合评分门槛 |
| `acquire_confirm_frames` | 2 | 连续 N 帧确认才锁定，防误检 |
| `tracking_gate_px` | 170 | 跟踪时允许的最大位置跳变 |
| `pipe_length_cm` | 25.0 | 管道实际长度，像素→cm 换算依据 |
| `center_trim_cm` | 0.0 | **相机零点与机械中点的偏差修正** |
| `cam_w` / `cam_h` | 640 / 480 | 采集分辨率 |

### `center_trim_cm` 怎么填

把球停在管道**机械中点**，读网页上显示的位置值 `x`，把本参数设为 `-x`。
固件里的偏置学习只能修正导轨不水平（倾角），修不了相机零位偏差，
所以必须在视觉侧对齐。

## 显示内容

屏幕叠加框从左到右：ROI 条带、管道左右盲区线、球心圆与半径、位置(cm)/速度、
置信度、管道中心与宽度、当前状态（TRACK / ACQUIRE / LOSE / SUSPECT）。
