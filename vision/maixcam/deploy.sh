#!/usr/bin/env bash
# MaixCam 视觉脚本一键部署：主脚本 + secret_config.py 一起传。
#
# 用法（Git Bash / Windows Terminal 均可）：
#   ./deploy.sh 192.168.x.x            # 传到设备 /root
#   ./deploy.sh 192.168.x.x /root/mydir
#
# MaixCam 出厂 SSH：用户 root，密码 root（输一次即可）。
# 设备 IP 在 MaixCam 开机屏幕上显示，或与电脑同网段用路由器后台查。
#
# 为什么要连 secret_config.py 一起传：
#   凭据只允许存在于 secret_config.py（本地 + 设备）。只传主脚本的话，
#   设备上没有 secret_config.py，脚本会落到占位符连不上热点——
#   而把密码填回主脚本的习惯已经导致公开仓库三次泄露。

set -e

IP="${1:?用法: ./deploy.sh <MaixCam IP> [设备目录]}"
DIR="${2:-/root}"
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ ! -f "$HERE/secret_config.py" ]; then
    echo "错误：$HERE/secret_config.py 不存在。"
    echo "先复制 secret_config.example.py 为 secret_config.py 并填入 WiFi 账号密码。"
    exit 1
fi

if grep -q "YOUR_WIFI_SSID" "$HERE/secret_config.py"; then
    echo "错误：secret_config.py 里还是占位符，先填入真实的 WIFI_SSID / WIFI_PASSWORD。"
    exit 1
fi

echo "==> 部署到 root@$IP:$DIR"
scp "$HERE/maixcam_ball_detect.py" "$HERE/secret_config.py" "root@$IP:$DIR/"

echo ""
echo "==> 完成。在 MaixCam 上运行："
echo "    python $DIR/maixcam_ball_detect.py"
echo "    （或 MaixVision 里打开运行）"
