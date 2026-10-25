#!/usr/bin/env python3

import math
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
from maix import (
    app,
    camera,
    display,
    err,
    image,
    network,
    pinmap,
    time,
    uart,
    webrtc,
)


# 本地私密配置：把 secret_config.example.py 复制为 secret_config.py 并填入
# 自己的 WiFi 账号密码。secret_config.py 已在 .gitignore 中，不会入库。
try:
    from secret_config import WIFI_SSID, WIFI_PASSWORD
except ImportError:
    WIFI_SSID = "YOUR_WIFI_SSID"
    WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"


CONFIG = {
    # WiFi 账号密码来自 secret_config.py；仍是占位符时启动会直接报错退出。
    "wifi_ssid": WIFI_SSID,
    "wifi_password": WIFI_PASSWORD,
    "wifi_timeout_s": 60,
    "web_port": 8000,
    "signaling_port": 8001,
    "uart_device": "/dev/ttyS1",
    "uart_baud": 115200,
    "uart_period_ms": 20,
    "cam_w": 640,
    "cam_h": 480,

    # 性能相关：LCD 刷新和 overlay 绘制不是每帧都必须，降频可把 CPU 让给检测。
    # display_every_n = 1 表示每帧都刷；3 表示每 3 帧刷一次。
    # perf_log_ms > 0 时每 N 毫秒在终端打印一次实测帧率，0 = 关闭。
    "display_every_n": 3,
    "perf_log_ms": 5000,

    # Only the area between these horizontal lines accepts a ball center.
    "roi_top_ratio": 0.40,
    "roi_bottom_ratio": 0.50,

    # Conservative circle detector tuning for the steel ball.
    "min_radius": 7,
    "max_radius": 22,
    "hough_min_distance": 35,
    "hough_param1": 60,
    "hough_param2": 19,

    # Calibrate the pipe once at startup, then keep its geometry fixed.
    # The first 5 seconds are ignored so the operator can aim the camera;
    # real statistics are collected in the following 5 seconds (20 ms/frame).
    "calibration_warmup_frames": 250,
    "pipe_calibration_frames": 250,
    "pipe_calibration_min_frames": 50,
    "pipe_calibration_timeout_frames": 500,

    # Independent left/right endpoint blind lines (pixels from each pipe end).
    # The mobile end (the side that tilts up/down) must be tracked live so the
    # blind line follows it; see mobile_end_* below.
    "endpoint_guard_left_px": 19,
    "endpoint_guard_right_px": 17,
    "endpoint_penalty_zone_left_px": 28,
    "endpoint_penalty_zone_right_px": 28,
    "endpoint_score_penalty_left": 0.28,
    "endpoint_score_penalty_right": 0.28,

    # Which pipe end moves when the mechanism tilts ("left" or "right").
    # The live pipe contour is re-detected every mobile_end_refresh_interval
    # frames and EMA-smoothed so the blind line tracks the moving end; the
    # fixed end keeps the calibrated position.
    "mobile_end_side": "right",
    "mobile_end_smoothing": 0.15,
    "mobile_end_refresh_interval": 5,

    # Candidate verification: dark fill, circular edge and local contrast.
    "min_fill_ratio": 0.12,
    "max_fill_ratio": 1.00,
    "min_edge_ratio": 0.045,
    "min_contrast": 2.0,
    "min_candidate_score": 0.36,

    # Require consecutive frames before acquisition and reject large jumps.
    "acquire_confirm_frames": 2,
    "acquire_tolerance_px": 55,
    "pending_hold_ms": 120,
    "tracking_gate_px": 170,
    "radius_gate_px": 10,
    "tracking_window_half_width": 140,
    "reacquire_scan_interval": 8,
    "reacquire_stationary_speed_px_s": 18,
    "reacquire_distance_px": 55,
    "reacquire_min_score": 0.46,
    "reacquire_min_motion_ratio": 0.025,
    "reacquire_confirm_frames": 3,
    "reacquire_min_motion_px": 2,
    "reacquire_max_step_px": 70,
    "reacquire_hold_ms": 450,

    # Stationary-lock suspect detection: a locked target that sits still away
    # from the pipe center is probably a static pipe feature (reflection,
    # seam, screw) rather than the ball. After suspect_stationary_frames of
    # stillness outside suspect_center_deadzone_px, force full-pipe scans
    # every frame and relax the challenger motion gate so a real (possibly
    # still) ball can steal the lock.
    "suspect_center_deadzone_px": 40,
    "suspect_stationary_frames": 25,
    "suspect_score_penalty": 0.25,
    # Suspect recovery: if the suspect state stays active this many frames
    # without any challenger stealing the lock, the locked target is almost
    # certainly the REAL ball parked by static friction (waiting for the
    # MCU breakaway pulses). Clear the penalty so detection keeps flowing;
    # re-arm only after real ball motion or a full loss/reacquire.
    "suspect_recover_frames": 60,

    # Mid-point trim (cm). Park the ball exactly on the rail's mechanical
    # mid-point, read the position reported by the web UI, and set this to
    # the NEGATIVE of that value. The firmware settle bias can only trim the
    # rail level (tilt); it cannot remove a camera-zero bias, so align the
    # visual zero with the mechanical mid-point here.
    "center_trim_cm": 0.0,

    # Pipe geometry and position conversion.
    "pipe_length_cm": 25.0,
    "pipe_min_width_ratio": 0.40,
}


def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= int(value) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class VisionUartLink:
    def __init__(self, cfg):
        err.check_raise(
            pinmap.set_pin_function("A19", "UART1_TX"),
            "Failed to map A19 as UART1_TX",
        )
        err.check_raise(
            pinmap.set_pin_function("A18", "UART1_RX"),
            "Failed to map A18 as UART1_RX",
        )
        self.port = uart.UART(cfg["uart_device"], cfg["uart_baud"])
        self.period_ms = int(cfg["uart_period_ms"])
        self.last_send_ms = 0
        self.sequence = 0
        print("Vision UART: {} @ {}".format(
            cfg["uart_device"], cfg["uart_baud"]
        ))

    def send(self, result, state, now_ms):
        if now_ms - self.last_send_ms < self.period_ms:
            return
        self.last_send_ms = now_ms

        flags = 0
        if result is not None:
            flags |= 1 << 0
        if state.pipe_calibrated:
            flags |= 1 << 1
        if result is not None and state.filter.x is not None:
            flags |= 1 << 2

        position_x10_mm = 0
        velocity_mm_s = 0
        confidence = 0
        if result is not None:
            position_x10_mm = int(round(result["position_cm"] * 100.0))
            velocity_mm_s = int(round(result["velocity_mm_s"]))
            confidence = int(round(result["confidence"] * 100.0))

        position_x10_mm = max(-32768, min(32767, position_x10_mm))
        velocity_mm_s = max(-32768, min(32767, velocity_mm_s))
        confidence = max(0, min(100, confidence))
        position_u16 = position_x10_mm & 0xFFFF
        velocity_u16 = velocity_mm_s & 0xFFFF
        frame = bytearray((
            0xAA,
            0x55,
            1,
            self.sequence,
            flags,
            position_u16 & 0xFF,
            (position_u16 >> 8) & 0xFF,
            velocity_u16 & 0xFF,
            (velocity_u16 >> 8) & 0xFF,
            confidence,
            0,
            0,
        ))
        crc = crc16_ccitt(frame[:10])
        frame[10] = crc & 0xFF
        frame[11] = (crc >> 8) & 0xFF
        self.port.write(bytes(frame))
        self.sequence = (self.sequence + 1) & 0xFF


STATUS_LOCK = threading.Lock()
STATUS = {
    "detected": False,
    "position_cm": None,
    "x": None,
    "y": None,
    "radius": None,
    "confidence": None,
    "pipe_center_x": None,
    "pipe_width": None,
    "updated_ms": 0,
}


def update_web_status(result, state):
    payload = {
        "detected": result is not None,
        "position_cm": None if result is None else round(result["position_cm"], 2),
        "x": None if result is None else result["x"],
        "y": None if result is None else result["y"],
        "radius": None if result is None else result["radius"],
        "confidence": None if result is None else round(result["confidence"], 3),
        "pipe_center_x": round(state.pipe_center_x, 1),
        "pipe_width": None if state.pipe_w < 0 else round(state.pipe_w, 1),
        "center_trim_cm": round(float(CONFIG["center_trim_cm"]), 2),
        "updated_ms": time.ticks_ms(),
    }
    with STATUS_LOCK:
        STATUS.clear()
        STATUS.update(payload)


WEB_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>MaixCAM WebRTC</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #15171a; color: #fff; }
    main { width: min(920px, 100%); margin: auto; padding: 16px; }
    h1 { margin: 4px 0 14px; font-size: 22px; }
    .video-wrap { overflow: hidden; border-radius: 8px; background: #000; }
    video { display: block; width: 100%; max-height: 72vh; background: #000; }
    .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 14px; }
    button { min-height: 48px; border: 0; border-radius: 8px; color: #fff;
             font-size: 17px; font-weight: 700; }
    button:disabled { opacity: .4; }
    #start { background: #16894c; }
    #stop { background: #c93636; }
    .status { display: flex; justify-content: space-between; margin-top: 12px;
              padding: 10px 12px; background: #ffffff0d; border-radius: 8px; }
    .telemetry { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px;
                 margin-top: 12px; }
    .metric { padding: 10px; border-radius: 8px; background: #ffffff0d; }
    .metric small { display: block; color: #aeb5bf; font-size: 12px; }
    .metric strong { display: block; margin-top: 4px; font-size: 18px; }
    @media (max-width: 600px) { .telemetry { grid-template-columns: repeat(2, 1fr); } }
    #download { display: none; margin-top: 12px; padding: 12px; border-radius: 8px;
                text-align: center; background: #1769aa; color: #fff; }
  </style>
</head>
<body>
<main>
  <h1>MaixCAM WebRTC + OpenCV</h1>
  <div class="video-wrap"><video id="video" autoplay playsinline muted></video></div>
  <div class="controls">
    <button id="start" disabled>Start recording</button>
    <button id="stop" disabled>Stop and save</button>
  </div>
  <div class="status"><span id="state">Connecting...</span><span id="timer">00:00</span></div>
  <div class="telemetry">
    <div class="metric"><small>Ball</small><strong id="ballState">Waiting</strong></div>
    <div class="metric"><small>Position</small><strong id="ballPosition">-- cm</strong></div>
    <div class="metric"><small>Pixel</small><strong id="ballPixel">--</strong></div>
    <div class="metric"><small>Pipe center</small><strong id="pipeCenter">--</strong></div>
  </div>
  <a id="download">Download recording</a>
</main>
<script>
  const video = document.getElementById('video');
  const startButton = document.getElementById('start');
  const stopButton = document.getElementById('stop');
  const stateText = document.getElementById('state');
  const timerText = document.getElementById('timer');
  const downloadLink = document.getElementById('download');
  const ballState = document.getElementById('ballState');
  const ballPosition = document.getElementById('ballPosition');
  const ballPixel = document.getElementById('ballPixel');
  const pipeCenter = document.getElementById('pipeCenter');
  let peerConnection = null, mediaRecorder = null, chunks = [];
  let timerHandle = null, recordStartedAt = 0, lastDownloadUrl = null;

  async function refreshTelemetry() {
    try {
      const response = await fetch('/status', {cache: 'no-store'});
      const data = await response.json();
      ballState.textContent = data.detected ? 'Detected' : 'Not found';
      ballPosition.textContent = data.position_cm === null ? '-- cm' : `${data.position_cm.toFixed(2)} cm`;
      ballPixel.textContent = data.x === null ? '--' : `${data.x}, ${data.y}`;
      pipeCenter.textContent = data.pipe_center_x === null ? '--' : `${data.pipe_center_x.toFixed(1)} px`;
    } catch (_) {
      ballState.textContent = 'Offline';
    }
  }
  refreshTelemetry();
  setInterval(refreshTelemetry, 200);

  const clientId = Array.from(crypto.getRandomValues(new Uint8Array(10)))
    .map(value => (value % 36).toString(36)).join('');
  const websocket = new WebSocket(`ws://${location.hostname}:8001/${clientId}`);
  websocket.onopen = () => {
    stateText.textContent = 'Connected, waiting for video';
    websocket.send(JSON.stringify({id: 'server', type: 'request'}));
  };
  websocket.onmessage = async event => {
    if (typeof event.data !== 'string') return;
    const message = JSON.parse(event.data);
    if (message.type === 'offer') await handleOffer(message);
  };
  websocket.onerror = () => { stateText.textContent = 'Connection failed'; };
  websocket.onclose = () => {
    stateText.textContent = 'Connection closed';
    startButton.disabled = true;
    if (mediaRecorder && mediaRecorder.state === 'recording') mediaRecorder.stop();
  };

  function createPeerConnection() {
    const pc = new RTCPeerConnection({bundlePolicy: 'max-bundle', iceServers: []});
    pc.ontrack = event => {
      video.srcObject = event.streams[0];
      video.play().catch(() => {});
      startButton.disabled = false;
      stateText.textContent = 'Video connected';
    };
    pc.onconnectionstatechange = () => {
      if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)) {
        stateText.textContent = `WebRTC ${pc.connectionState}`;
        startButton.disabled = true;
      }
    };
    return pc;
  }

  async function waitForIce() {
    if (peerConnection.iceGatheringState === 'complete') return;
    await new Promise(resolve => {
      const listener = () => {
        if (peerConnection.iceGatheringState === 'complete') {
          peerConnection.removeEventListener('icegatheringstatechange', listener);
          resolve();
        }
      };
      peerConnection.addEventListener('icegatheringstatechange', listener);
    });
  }

  async function handleOffer(offer) {
    peerConnection = createPeerConnection();
    await peerConnection.setRemoteDescription(offer);
    await peerConnection.setLocalDescription(await peerConnection.createAnswer());
    await waitForIce();
    websocket.send(JSON.stringify({id: 'server', type: peerConnection.localDescription.type,
      sdp: peerConnection.localDescription.sdp}));
  }

  function chooseMimeType() {
    return ['video/webm;codecs=vp8,opus', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4']
      .find(type => MediaRecorder.isTypeSupported(type)) || '';
  }
  function updateTimer() {
    const seconds = Math.floor((Date.now() - recordStartedAt) / 1000);
    timerText.textContent = `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  }
  startButton.onclick = () => {
    if (!video.srcObject || typeof MediaRecorder === 'undefined') return;
    chunks = [];
    downloadLink.style.display = 'none';
    if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);
    const mimeType = chooseMimeType();
    mediaRecorder = new MediaRecorder(video.srcObject, mimeType ? {mimeType} : undefined);
    mediaRecorder.ondataavailable = event => { if (event.data?.size) chunks.push(event.data); };
    mediaRecorder.onstop = () => {
      clearInterval(timerHandle);
      const type = mediaRecorder.mimeType || mimeType || 'video/webm';
      const blob = new Blob(chunks, {type});
      lastDownloadUrl = URL.createObjectURL(blob);
      downloadLink.href = lastDownloadUrl;
      downloadLink.download = `maixcam-${new Date().toISOString().replace(/[:.]/g, '-')}.webm`;
      downloadLink.textContent = `Download recording (${(blob.size / 1048576).toFixed(1)} MB)`;
      downloadLink.style.display = 'block';
      stateText.textContent = 'Recording complete';
      startButton.disabled = false;
      stopButton.disabled = true;
    };
    mediaRecorder.start(1000);
    recordStartedAt = Date.now();
    updateTimer();
    timerHandle = setInterval(updateTimer, 500);
    stateText.textContent = 'Recording';
    startButton.disabled = true;
    stopButton.disabled = false;
  };
  stopButton.onclick = () => { if (mediaRecorder?.state === 'recording') mediaRecorder.stop(); };
</script>
</body>
</html>
"""


class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True


class WebPageHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/status":
            with STATUS_LOCK:
                body = json.dumps(STATUS, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = WEB_PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        pass


class AdaptiveAlphaBetaFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))
        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)
        motion = min(1.0, error / 10.0) ** 2
        alpha = 0.20 + (0.95 - 0.20) * motion
        beta = 0.01 + (0.12 - 0.01) * motion
        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.last_ms = now_ms
        return self.x, self.y

    def predict(self, now_ms):
        if self.x is None or self.last_ms is None:
            return None
        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.0, min(0.150, dt))
        return self.x + self.vx * dt, self.y + self.vy * dt

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= 250:
            self.reset()
            return True
        return False


class Cv2State:
    def __init__(self, image_width, cfg):
        self.filter = AdaptiveAlphaBetaFilter()
        self.pipe_x = -1.0
        self.pipe_w = -1.0
        self.pipe_live_x = -1.0
        self.pipe_live_w = -1.0
        self.pipe_center_x = image_width * 0.5
        self.cm_per_pixel = cfg["pipe_length_cm"] / image_width
        self.pipe_calibrated = False
        self.pipe_calibration_attempts = 0
        self.pipe_calibration_x = []
        self.pipe_calibration_w = []
        self.last_detect_ms = 0
        self.detect_fps = 0.0
        self.last_radius = None
        self.pending_x = None
        self.pending_radius = None
        self.pending_count = 0
        self.pending_last_ms = None
        self.pending_hold_ms = cfg["pending_hold_ms"]
        self.search_left = 0
        self.search_right = image_width - 1
        self.detect_left = 0
        self.detect_right = image_width
        self.scan_count = 0
        self.challenger_x = None
        self.challenger_y = None
        self.challenger_radius = None
        self.challenger_count = 0
        self.challenger_last_ms = None
        self.previous_gray = None
        self.suspect_stationary_frames = 0
        self.stationary_suspect = False
        self.suspect_recovery_frames = 0
        self.suspect_recovered = False
        self.center_trim_px = 0.0

    def clear_pending(self):
        self.pending_x = None
        self.pending_radius = None
        self.pending_count = 0
        self.pending_last_ms = None

    def clear_challenger(self):
        self.challenger_x = None
        self.challenger_y = None
        self.challenger_radius = None
        self.challenger_count = 0
        self.challenger_last_ms = None

    def mark_missing(self, now_ms):
        if (
            self.pending_last_ms is not None
            and now_ms - self.pending_last_ms > self.pending_hold_ms
        ):
            self.clear_pending()
        if self.filter.mark_missing(now_ms):
            self.last_radius = None


def calculate_ball_center_position(ball_x, center_x, cm_per_pixel):
    """Return signed position: center is zero, left positive, right negative."""
    position_px = float(center_x) - float(ball_x)
    scale = max(0.0, float(cm_per_pixel))
    if abs(position_px) < 0.5:
        position_px = 0.0
    return {
        "position_px": position_px,
        "position_cm": position_px * scale,
    }


def candidate_motion_ratio(motion_gray, x, y, radius):
    if motion_gray is None:
        return 0.0
    margin = max(2, int(round(radius * 1.35)))
    x1 = max(0, int(x) - margin)
    x2 = min(motion_gray.shape[1], int(x) + margin + 1)
    y1 = max(0, int(y) - margin)
    y2 = min(motion_gray.shape[0], int(y) + margin + 1)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    patch = motion_gray[y1:y2, x1:x2]
    _, changed = cv2.threshold(patch, 12, 255, cv2.THRESH_BINARY)
    return float(cv2.countNonZero(changed)) / max(1, patch.size)


def evaluate_circle_candidate(gray, dark_mask, edges, x, y, radius, cfg):
    margin = int(math.ceil(radius * 1.45))
    x1 = max(0, x - margin)
    y1 = max(0, y - margin)
    x2 = min(gray.shape[1], x + margin + 1)
    y2 = min(gray.shape[0], y + margin + 1)
    if x2 - x1 < radius * 2 or y2 - y1 < radius * 2:
        return None

    patch_gray = gray[y1:y2, x1:x2]
    patch_dark = dark_mask[y1:y2, x1:x2]
    patch_edges = edges[y1:y2, x1:x2]
    center = (x - x1, y - y1)
    mask_shape = patch_gray.shape[:2]
    inner_mask = np.zeros(mask_shape, np.uint8)
    edge_mask = np.zeros(mask_shape, np.uint8)
    outer_mask = np.zeros(mask_shape, np.uint8)
    cv2.circle(inner_mask, center, max(1, int(radius * 0.78)), 255, -1)
    cv2.circle(edge_mask, center, max(1, int(radius * 1.22)), 255, -1)
    cv2.circle(edge_mask, center, max(1, int(radius * 0.78)), 0, -1)
    cv2.circle(outer_mask, center, max(1, int(radius * 1.42)), 255, -1)
    cv2.circle(outer_mask, center, max(1, int(radius * 1.12)), 0, -1)

    inner_count = cv2.countNonZero(inner_mask)
    edge_count = cv2.countNonZero(edge_mask)
    outer_count = cv2.countNonZero(outer_mask)
    if inner_count == 0 or edge_count == 0 or outer_count == 0:
        return None

    fill_ratio = float(cv2.countNonZero(
        cv2.bitwise_and(patch_dark, inner_mask)
    )) / inner_count
    edge_ratio = float(cv2.countNonZero(
        cv2.bitwise_and(patch_edges, edge_mask)
    )) / edge_count
    inner_mean = float(cv2.mean(patch_gray, mask=inner_mask)[0])
    outer_mean = float(cv2.mean(patch_gray, mask=outer_mask)[0])
    contrast = outer_mean - inner_mean

    if not (cfg["min_fill_ratio"] <= fill_ratio <= cfg["max_fill_ratio"]):
        return None
    if edge_ratio < cfg["min_edge_ratio"] or contrast < cfg["min_contrast"]:
        return None

    fill_score = 1.0 - min(1.0, abs(fill_ratio - 0.58) / 0.58)
    edge_score = min(1.0, edge_ratio / 0.20)
    contrast_score = min(1.0, contrast / 45.0)
    return {
        "fill_ratio": fill_ratio,
        "edge_ratio": edge_ratio,
        "contrast": contrast,
        "score": fill_score * 0.30 + edge_score * 0.45 + contrast_score * 0.25,
    }


def robust_middle_mean(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    trim = len(ordered) // 5
    middle = ordered[trim:len(ordered) - trim] if trim > 0 else ordered
    return sum(middle) / len(middle)


def update_pipe_calibration(gray, cfg, state):
    state.pipe_calibration_attempts += 1
    # First 5 seconds are camera-aiming time: collect nothing.
    if state.pipe_calibration_attempts <= cfg["calibration_warmup_frames"]:
        return False
    _, binary = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    binary = cv2.morphologyEx(
        binary, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
    )
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours:
        widest = max(contours, key=lambda item: cv2.boundingRect(item)[2])
        pipe_x, _, pipe_w, _ = cv2.boundingRect(widest)
        if pipe_w > gray.shape[1] * cfg["pipe_min_width_ratio"]:
            state.pipe_calibration_x.append(float(pipe_x))
            state.pipe_calibration_w.append(float(pipe_w))

    valid_count = len(state.pipe_calibration_w)
    enough_samples = valid_count >= cfg["pipe_calibration_frames"]
    timed_out = (
        state.pipe_calibration_attempts >= cfg["pipe_calibration_timeout_frames"]
        and valid_count >= cfg["pipe_calibration_min_frames"]
    )
    if not enough_samples and not timed_out:
        return False

    state.pipe_x = robust_middle_mean(state.pipe_calibration_x)
    state.pipe_w = robust_middle_mean(state.pipe_calibration_w)
    state.pipe_live_x = state.pipe_x
    state.pipe_live_w = state.pipe_w
    state.pipe_center_x = state.pipe_x + state.pipe_w * 0.5
    state.cm_per_pixel = cfg["pipe_length_cm"] / max(1.0, state.pipe_w)
    # Mechanical mid-point trim: shifts the visual zero line so the ball
    # resting on the true rail mid-point reports position 0.
    state.center_trim_px = (
        float(cfg["center_trim_cm"]) / state.cm_per_pixel
        if state.cm_per_pixel > 0.0
        else 0.0
    )
    state.pipe_calibrated = True
    state.pipe_calibration_x = []
    state.pipe_calibration_w = []
    print(
        "Pipe calibrated: x={:.1f}, width={:.1f}, center={:.1f}".format(
            state.pipe_x, state.pipe_w, state.pipe_center_x
        )
    )
    return True


def refresh_mobile_end(gray, cfg, state):
    """Re-detect the pipe contour and EMA-smooth its live endpoints so the
    mobile (tilting) end's blind line follows the pipe as it moves up/down.
    The result is clamped around the calibrated geometry so a ball on the
    pipe cannot drag the contour off the pipe."""
    if state.pipe_live_x < 0.0:
        state.pipe_live_x = state.pipe_x
        state.pipe_live_w = state.pipe_w
        return
    _, binary = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    binary = cv2.morphologyEx(
        binary, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
    )
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return
    widest = max(contours, key=lambda item: cv2.boundingRect(item)[2])
    live_x, _, live_w, _ = cv2.boundingRect(widest)
    if live_w <= gray.shape[1] * cfg["pipe_min_width_ratio"]:
        return
    smoothing = cfg["mobile_end_smoothing"]
    state.pipe_live_x += smoothing * (float(live_x) - state.pipe_live_x)
    state.pipe_live_w += smoothing * (float(live_w) - state.pipe_live_w)
    base_w = state.pipe_w
    state.pipe_live_x = max(
        state.pipe_x - 0.25 * base_w,
        min(state.pipe_x + 0.25 * base_w, state.pipe_live_x),
    )
    state.pipe_live_w = max(
        base_w * 0.60, min(base_w * 1.15, state.pipe_live_w)
    )


def detect_pipe_and_ball(img, roi_top, roi_bottom, cfg, state):
    # 只把 ROI 这一条转成 numpy。
    # 原来是「整幅 640x480 转完再切片」，90% 的像素白拷一遍，是主循环最大
    # 的单笔开销。crop 在 C++ 层做，只拷真正要用的那几十行。
    roi_h = roi_bottom - roi_top
    if hasattr(img, "crop"):
        roi_img = img.crop(0, roi_top, cfg["cam_w"], roi_h)
        frame = image.image2cv(roi_img, False, False)
    else:                                   # 老固件没有 crop 时降级
        frame = image.image2cv(img, False, False)[roi_top:roi_bottom, :]
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    now_ms = time.ticks_ms()
    motion_gray = None
    if (
        state.previous_gray is not None
        and state.previous_gray.shape == gray.shape
    ):
        motion_gray = cv2.absdiff(gray, state.previous_gray)
    state.previous_gray = gray.copy()

    if not state.pipe_calibrated:
        update_pipe_calibration(gray, cfg, state)
        state.mark_missing(now_ms)
        if not state.pipe_calibrated:
            return None

    # Live pipe-end tracking: the mobile (tilting) end's blind line must
    # follow the pipe when it moves up/down; the fixed end keeps the
    # calibrated position.
    if state.pipe_calibrated and (
        state.scan_count % cfg["mobile_end_refresh_interval"] == 0
    ):
        refresh_mobile_end(gray, cfg, state)

    fixed_left = max(0, int(round(state.pipe_x)))
    fixed_right = min(gray.shape[1], int(round(state.pipe_x + state.pipe_w)))
    live_left = max(0, int(round(state.pipe_live_x)))
    live_right = min(
        gray.shape[1], int(round(state.pipe_live_x + state.pipe_live_w))
    )
    if cfg["mobile_end_side"] == "left":
        pipe_left = live_left
        pipe_right = fixed_right
    else:
        pipe_left = fixed_left
        pipe_right = live_right

    search_left = min(
        pipe_right, pipe_left + cfg["endpoint_guard_left_px"]
    )
    search_right = max(
        pipe_left, pipe_right - cfg["endpoint_guard_right_px"]
    )
    state.search_left = search_left
    state.search_right = search_right

    predicted = state.filter.predict(now_ms)
    track_x = None if predicted is None else predicted[0]
    state.scan_count += 1
    track_stationary = (
        track_x is not None
        and abs(state.filter.vx) <= cfg["reacquire_stationary_speed_px_s"]
    )
    # Stationary-lock suspect detection.
    state.stationary_suspect = False
    if track_x is not None and track_stationary:
        if abs(track_x - state.pipe_center_x) > cfg["suspect_center_deadzone_px"]:
            if state.suspect_recovered:
                # A previous suspect round ended with no challenger: the
                # locked target is the real (stuck) ball. Re-arm the
                # suspect check only after real motion or a reacquire.
                state.suspect_stationary_frames = 0
            else:
                state.suspect_stationary_frames += 1
            if (state.suspect_stationary_frames >=
                    cfg["suspect_stationary_frames"]):
                state.stationary_suspect = True
        else:
            state.suspect_stationary_frames = 0
    else:
        state.suspect_stationary_frames = 0
        if track_x is None or not track_stationary:
            # Ball actually moved (or the lock was lost): a fresh suspect
            # round is meaningful again.
            state.suspect_recovered = False
    # Suspect recovery: nothing challenged the lock for a long window, so
    # the stationary target is the real ball waiting for the MCU breakaway
    # pulses. Lift the penalty instead of dropping detection for seconds.
    if state.stationary_suspect:
        state.suspect_recovery_frames += 1
        if state.suspect_recovery_frames >= cfg["suspect_recover_frames"]:
            state.stationary_suspect = False
            state.suspect_stationary_frames = 0
            state.suspect_recovery_frames = 0
            state.suspect_recovered = True
    else:
        state.suspect_recovery_frames = 0
    full_reacquire_scan = track_x is None or (
        track_stationary
        and state.scan_count % cfg["reacquire_scan_interval"] == 0
    ) or state.stationary_suspect
    detect_left = pipe_left
    detect_right = pipe_right
    if track_x is not None and not full_reacquire_scan:
        # Motion-aware window: a fast ball needs a wider search window.
        half_width = cfg["tracking_window_half_width"] + int(
            round(abs(state.filter.vx) * 0.06)
        )
        detect_left = max(pipe_left, int(round(track_x - half_width)))
        detect_right = min(pipe_right, int(round(track_x + half_width)))
    state.detect_left = detect_left
    state.detect_right = detect_right

    search_gray = gray[:, detect_left:detect_right]
    if search_gray.shape[1] <= cfg["max_radius"] * 2:
        state.mark_missing(now_ms)
        return None

    blur = cv2.medianBlur(search_gray, 3)
    _, dark_mask = cv2.threshold(
        search_gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
    )
    dark_mask = cv2.morphologyEx(
        dark_mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
    )
    edges = cv2.Canny(
        blur, max(10, cfg["hough_param1"] // 2), cfg["hough_param1"]
    )
    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=cfg["hough_min_distance"],
        param1=cfg["hough_param1"],
        param2=cfg["hough_param2"],
        minRadius=cfg["min_radius"],
        maxRadius=cfg["max_radius"],
    )
    candidates = []
    if circles is not None:
        for x, y, radius in np.round(circles[0]).astype("int"):
            candidates.append((int(x), int(y), int(radius), None))

    # Once a real circle has been locked, a fast-moving ball may lose its
    # circular Hough edge for one or two frames. Follow a nearby dark blob as
    # a temporary fallback, but never use blobs for initial acquisition.
    if track_x is not None:
        blob_contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        min_blob_area = math.pi * max(1, cfg["min_radius"] - 2) ** 2 * 0.45
        # Motion blur stretches the blob, so keep the upper bound generous.
        max_blob_area = math.pi * (cfg["max_radius"] * 2.2) ** 2
        for contour in blob_contours:
            area = float(cv2.contourArea(contour))
            if area < min_blob_area or area > max_blob_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue
            aspect = float(width) / height
            solidity = area / float(width * height)
            radius = int(round(min(width, height) * 0.5))
            if not (0.35 <= aspect <= 2.8 and solidity >= 0.25):
                continue
            if radius < cfg["min_radius"] - 2 or radius > cfg["max_radius"] + 4:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            center_x = int(round(moments["m10"] / moments["m00"]))
            center_y = int(round(moments["m01"] / moments["m00"]))
            quality = {
                "fill_ratio": solidity,
                "edge_ratio": 0.0,
                "contrast": 0.0,
                "score": 0.40 + min(0.15, solidity * 0.15),
            }
            candidates.append((center_x, center_y, radius, quality))

    if not candidates:
        state.mark_missing(now_ms)
        return None

    accepted = []
    challengers = []
    # Motion-adaptive tracking gate: a fast ball is allowed to jump further
    # from the predicted position without being treated as a new target.
    adaptive_gate = cfg["tracking_gate_px"]
    if track_x is not None:
        adaptive_gate += int(round(abs(state.filter.vx) * 0.05))
    for local_x, local_y, radius, quality_hint in candidates:
        global_x = int(local_x + detect_left)
        if global_x <= search_left or global_x >= search_right:
            continue
        distance = None if track_x is None else abs(global_x - track_x)
        outside_tracking_gate = (
            distance is not None and distance > adaptive_gate
        )
        if outside_tracking_gate and not full_reacquire_scan:
            continue
        if track_x is not None and not outside_tracking_gate:
            if state.last_radius is not None and abs(
                radius - state.last_radius
            ) > cfg["radius_gate_px"]:
                continue
        quality = quality_hint
        if quality is None:
            quality = evaluate_circle_candidate(
                search_gray, dark_mask, edges,
                int(local_x), int(local_y), int(radius), cfg
            )
        if quality is None:
            continue
        score = quality["score"]
        motion_ratio = candidate_motion_ratio(
            motion_gray, global_x, int(local_y), int(radius)
        )
        # Independent left/right endpoint penalty zones.
        dist_left = global_x - search_left
        dist_right = search_right - global_x
        if dist_left < cfg["endpoint_penalty_zone_left_px"]:
            ratio = max(
                0.0, float(dist_left) / cfg["endpoint_penalty_zone_left_px"]
            )
            score -= cfg["endpoint_score_penalty_left"] * (1.0 - ratio)
        if dist_right < cfg["endpoint_penalty_zone_right_px"]:
            ratio = max(
                0.0, float(dist_right) / cfg["endpoint_penalty_zone_right_px"]
            )
            score -= cfg["endpoint_score_penalty_right"] * (1.0 - ratio)
        if track_x is not None and not outside_tracking_gate:
            score += 0.30 * max(0.0, 1.0 - distance / adaptive_gate)
            if state.last_radius is not None:
                radius_error = abs(radius - state.last_radius)
                score += 0.10 * max(
                    0.0, 1.0 - radius_error / cfg["radius_gate_px"]
                )
        item = (score, global_x, int(local_y), int(radius), quality)
        # During a stationary-suspect deadlock, penalize candidates that sit
        # on the stuck target so a real ball elsewhere can win.
        if (state.stationary_suspect and track_x is not None and
                distance is not None and distance < 30):
            score -= cfg["suspect_score_penalty"]
        min_motion = (0.0 if state.stationary_suspect
                      else cfg["reacquire_min_motion_ratio"])
        moving_challenger = (
            track_x is not None
            and full_reacquire_scan
            and distance >= cfg["reacquire_distance_px"]
            and quality["edge_ratio"] >= cfg["min_edge_ratio"]
            and score >= cfg["reacquire_min_score"]
            and motion_ratio >= min_motion
        )
        if moving_challenger:
            challengers.append(item + (motion_ratio,))
        elif not outside_tracking_gate and score >= cfg["min_candidate_score"]:
            accepted.append(item)

    # A local tracker can stay attached to a static pipe feature forever.
    # During periodic full-pipe scans, let a verified moving circle outside
    # the tracking gate challenge the lock. Three consistent observations
    # are required, so a single Hough false positive cannot steal tracking.
    takeover = None
    if track_x is not None and full_reacquire_scan:
        if challengers:
            challenger = max(challengers, key=lambda item: item[0] + item[5])
            _, cx, cy, cr, _, _ = challenger
            if state.challenger_x is None:
                state.challenger_count = 1
            else:
                challenger_step = abs(cx - state.challenger_x)
                if (
                    cfg["reacquire_min_motion_px"] <= challenger_step
                    <= cfg["reacquire_max_step_px"]
                ):
                    state.challenger_count += 1
                elif challenger_step > cfg["reacquire_max_step_px"]:
                    state.challenger_count = 1
            state.challenger_x = float(cx)
            state.challenger_y = float(cy)
            state.challenger_radius = float(cr)
            state.challenger_last_ms = now_ms
            if state.challenger_count >= cfg["reacquire_confirm_frames"]:
                takeover = (
                    challenger[0],
                    int(round(state.challenger_x)),
                    int(round(state.challenger_y)),
                    int(round(state.challenger_radius)),
                    challenger[4],
                )
                state.filter.reset()
                state.last_radius = None
                state.clear_pending()
                state.clear_challenger()
        else:
            if (
                state.challenger_last_ms is not None
                and now_ms - state.challenger_last_ms > cfg["reacquire_hold_ms"]
            ):
                state.clear_challenger()

    if takeover is not None:
        score, raw_x, raw_y, radius, quality = takeover
    elif accepted:
        score, raw_x, raw_y, radius, quality = max(
            accepted, key=lambda item: item[0]
        )
    else:
        state.mark_missing(now_ms)
        return None
    if state.filter.x is None:
        same_candidate = (
            state.pending_x is not None
            and abs(raw_x - state.pending_x) <= cfg["acquire_tolerance_px"]
            and abs(radius - state.pending_radius) <= cfg["radius_gate_px"]
        )
        if same_candidate:
            state.pending_count += 1
            state.pending_x = (state.pending_x + raw_x) * 0.5
            state.pending_radius = (state.pending_radius + radius) * 0.5
            state.pending_last_ms = now_ms
        else:
            state.pending_x = float(raw_x)
            state.pending_radius = float(radius)
            state.pending_count = 1
            state.pending_last_ms = now_ms
        if state.pending_count < cfg["acquire_confirm_frames"]:
            return None
        raw_x = int(round(state.pending_x))
        radius = int(round(state.pending_radius))
        state.clear_pending()

    filtered_x, _ = state.filter.update(raw_x, raw_y, now_ms)
    if state.last_radius is None:
        state.last_radius = float(radius)
    else:
        state.last_radius = state.last_radius * 0.75 + radius * 0.25
    previous_detect_ms = state.last_detect_ms
    state.last_detect_ms = now_ms
    elapsed = now_ms - previous_detect_ms
    if previous_detect_ms > 0 and elapsed > 0:
        state.detect_fps = state.detect_fps * 0.8 + (1000.0 / elapsed) * 0.2
    actual_y = int(raw_y + roi_top)
    measurement = calculate_ball_center_position(
        filtered_x,
        state.pipe_center_x + state.center_trim_px,
        state.cm_per_pixel,
    )
    return {
        "x": int(filtered_x),
        "y": actual_y,
        "raw_x": int(raw_x),
        "radius": int(radius),
        # Report visual evidence only. Tracking proximity is useful for
        # ranking, but must not turn a static pipe feature into 100% confidence.
        "confidence": min(1.0, float(quality["score"])),
        "fill_ratio": quality["fill_ratio"],
        "edge_ratio": quality["edge_ratio"],
        "position_px": measurement["position_px"],
        "position_cm": measurement["position_cm"],
        "velocity_mm_s": -state.filter.vx * state.cm_per_pixel * 10.0,
    }


def connect_wifi(cfg):
    wifi = network.wifi.Wifi()
    print("Connecting to WiFi:", cfg["wifi_ssid"])
    result = wifi.connect(
        cfg["wifi_ssid"],
        cfg["wifi_password"],
        wait=True,
        timeout=cfg["wifi_timeout_s"],
    )
    err.check_raise(result, "Failed to connect to WiFi")
    ip = wifi.get_ip()
    if not ip:
        raise RuntimeError("WiFi connected, but no IP address was assigned")
    print("WiFi connected, IP:", ip)
    return wifi, ip


def start_web_server(cfg):
    httpd = ReusableHTTPServer(("0.0.0.0", cfg["web_port"]), WebPageHandler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def draw_overlay(img, roi_top, roi_bottom, cv2_result, state, image_width, image_height):
    img.draw_line(0, roi_top, image_width - 1, roi_top, image.COLOR_RED, 2)
    img.draw_line(0, roi_bottom, image_width - 1, roi_bottom, image.COLOR_RED, 2)
    img.draw_string(5, roi_top + 6, "TRACKING AREA", image.COLOR_RED, scale=1.0)

    if not state.pipe_calibrated:
        warmup = CONFIG["calibration_warmup_frames"]
        if state.pipe_calibration_attempts <= warmup:
            img.draw_string(
                5, 5,
                "ADJUST CAMERA %d/%d" % (
                    state.pipe_calibration_attempts, warmup
                ),
                image.COLOR_BLUE,
                scale=1.0,
            )
        else:
            progress = min(
                len(state.pipe_calibration_w),
                CONFIG["pipe_calibration_frames"],
            )
            img.draw_string(
                5, 5,
                "CALIBRATING PIPE %d/%d" % (
                    progress, CONFIG["pipe_calibration_frames"]
                ),
                image.COLOR_BLUE,
                scale=1.0,
            )
        return

    # The zero line is the calibrated pipe center plus the mechanical
    # mid-point trim, not the image center.
    center_x = int(round(state.pipe_center_x + state.center_trim_px))
    img.draw_line(
        center_x,
        0,
        center_x,
        image_height - 1,
        image.COLOR_GREEN,
        2,
    )
    img.draw_line(
        int(state.pipe_x),
        roi_bottom + 5,
        int(state.pipe_x + state.pipe_w),
        roi_bottom + 5,
        image.COLOR_BLUE,
        2,
    )
    img.draw_line(
        int(state.search_left), roi_top,
        int(state.search_left), roi_bottom,
        image.COLOR_BLUE, 1,
    )
    img.draw_line(
        int(state.search_right), roi_top,
        int(state.search_right), roi_bottom,
        image.COLOR_BLUE, 1,
    )
    center_label = "CENTER 0"
    if abs(state.center_trim_px) >= 0.5:
        center_label = "CENTER 0 (trim %+.2fcm)" % CONFIG["center_trim_cm"]
    img.draw_string(
        min(image_width - 80, center_x + 5),
        5,
        center_label,
        image.COLOR_GREEN,
        scale=1.0,
    )

    if cv2_result is None:
        img.draw_string(5, 5, "No ball detected", image.COLOR_RED, scale=1.0)
        return

    img.draw_string(
        5, 25,
        "confidence %.2f" % cv2_result["confidence"],
        image.COLOR_GREEN,
        scale=1.0,
    )

    x = cv2_result["x"]
    y = cv2_result["y"]
    radius = cv2_result["radius"]
    # The horizontal segment directly represents the signed distance
    # between the ball center and the calibrated pipe center line.
    img.draw_line(center_x, y, x, y, image.COLOR_BLUE, 3)
    img.draw_cross(center_x, y, image.COLOR_BLUE, size=8, thickness=2)
    img.draw_circle(x, y, radius, image.COLOR_GREEN, thickness=2)
    img.draw_cross(x, y, image.COLOR_RED, size=8, thickness=2)
    img.draw_string(
        5,
        5,
        "distance to center: %+.2f cm" % cv2_result["position_cm"],
        image.COLOR_GREEN,
        scale=1.0,
    )


def main():
    print("MaixCAM ball detection: fast tracking + motion reacquisition v2")
    if CONFIG["wifi_ssid"] == "YOUR_WIFI_SSID":
        raise SystemExit(
            "未配置 WiFi：请把 secret_config.example.py 复制为 secret_config.py，"
            "填入 WIFI_SSID / WIFI_PASSWORD 后再运行"
        )
    _, ip = connect_wifi(CONFIG)
    web_server = start_web_server(CONFIG)

    yuv_cam = camera.Camera(
        CONFIG["cam_w"],
        CONFIG["cam_h"],
        image.Format.FMT_YVU420SP,
        buff_num=3,
    )
    rgb_cam = yuv_cam.add_channel(
        CONFIG["cam_w"],
        CONFIG["cam_h"],
        image.Format.FMT_RGB888,
        buff_num=1,
    )
    detector_server = webrtc.WebRTC(
        signaling_port=CONFIG["signaling_port"],
        http_server=False,
    )
    err.check_raise(
        detector_server.bind_camera(yuv_cam),
        "Failed to bind camera to WebRTC",
    )
    err.check_raise(detector_server.start(), "Failed to start WebRTC server")

    disp = display.Display()
    image_width = CONFIG["cam_w"]
    image_height = CONFIG["cam_h"]
    roi_top = int(image_height * CONFIG["roi_top_ratio"])
    roi_bottom = int(image_height * CONFIG["roi_bottom_ratio"])
    state = Cv2State(image_width, CONFIG)
    uart_link = VisionUartLink(CONFIG)
    print("Open in browser: http://{}:{}".format(ip, CONFIG["web_port"]))

    try:
        frame_i = 0
        perf_t0 = time.ticks_ms()
        perf_n = 0
        while not app.need_exit():
            rgb_img = rgb_cam.read()
            if rgb_img is None:
                continue
            result = detect_pipe_and_ball(
                rgb_img,
                roi_top,
                roi_bottom,
                CONFIG,
                state,
            )
            uart_link.send(result, state, time.ticks_ms())
            update_web_status(result, state)
            # overlay 与 LCD 刷新降频：这两步对控制没有贡献，但每帧都要
            # 画几十个图元 + 一次全屏拷贝，占的 CPU 相当可观。
            if frame_i % CONFIG["display_every_n"] == 0:
                draw_overlay(
                    rgb_img,
                    roi_top,
                    roi_bottom,
                    result,
                    state,
                    image_width,
                    image_height,
                )
                disp.show(rgb_img)
            frame_i += 1

            # 实测帧率（终端可见，用于判断优化是否生效）
            perf_n += 1
            elapsed_ms = time.ticks_ms() - perf_t0
            if (CONFIG["perf_log_ms"] > 0 and
                    elapsed_ms >= CONFIG["perf_log_ms"]):
                print("[PERF] loop {:.1f} fps".format(
                    perf_n * 1000.0 / max(1, elapsed_ms)))
                perf_t0 = time.ticks_ms()
                perf_n = 0
    finally:
        print("Stopping...")
        detector_server.stop()
        web_server.shutdown()
        web_server.server_close()
        rgb_cam.close()
        yuv_cam.close()


if __name__ == "__main__":
    main()
