# BBRobot — 并联平台球追踪系统

三舵机并联平台 + 树莓派 + 摄像头，通过霍夫圆检测红色小球位置，PID 控制平台倾斜使球稳定在图像中心。

## 硬件

| 组件 | 型号 |
|---|---|
| 主控 | 树莓派 5 (Raspberry Pi OS Bookworm) |
| 摄像头 | OV5647 CSI 摄像头 (或兼容 picamera2) |
| 舵机驱动 | PCA9685 16 通道 PWM 板 (I2C 0x40) |
| 舵机 | 标准 PWM 舵机 ×3 (180°, 500-2500µs) |
| 机械结构 | 三舵机 120° 并联平台 |

### 接线

```
PCA9685        树莓派
──────────────────────
VCC    →  5V  (Pin 2/4)
GND    →  GND (Pin 6)
SDA    →  GPIO2 (Pin 3)
SCL    →  GPIO3 (Pin 5)

舵机信号线 → PCA9685 CH0, CH1, CH2
舵机电源   → 独立 5-6V 供电
```

### 机械参数

| 参数 | 值 |
|---|---|
| 舵机支点距圆心 (L₀) | ~40.8 mm |
| 下连杆 (L₁) | 64 mm |
| 上连杆 (L₂) | 80 mm |
| 平台球头半径 (L₃) | 64 mm |
| 舵机间距 | ~70.7 mm |

## 软件安装

```bash
# 树莓派系统配置
sudo raspi-config
# Interface Options → I2C → Enable
# Interface Options → Camera → Enable（旧版 Pi OS）
sudo reboot

# 验证
sudo i2cdetect -y 1          # 应显示 0x40
libcamera-hello --list-cameras

# Python 环境
cd ~/bbrobot
python -m venv .venv
source .venv/bin/activate
pip install adafruit-circuitpython-pca9685 opencv-python numpy smbus2
```

## 文件结构

```
~/bbrobot/
├── README.md          本文件
├── class_servo.py     PCA9685 舵机驱动
├── class_BBRobot.py   并联平台运动学
├── class_PID.py       2D PID 控制器
├── class_Camera.py    霍夫圆检测
├── main.py            主程序
├── test_camera.py     相机调参工具（Pi 桌面/VNC 用）
└── capture_debug.py   帧采集诊断工具
```

## 运行

```bash
cd ~/bbrobot
source .venv/bin/activate

# 视觉调参（需要桌面或 VNC）
python test_camera.py

# 帧诊断（SSH 用，保存 5 帧到当前目录）
python capture_debug.py

# 主程序
python main.py
```

主程序终端输出：

```
img_fps: 62, rob_fps: 170  HIT x=+15 y=-08 area=7238
```

- `img_fps`：图像采集帧率
- `rob_fps`：球检测帧率
- `HIT`：检测到球，显示机器人坐标和面积
- `MISS`：未检测到球

## 检测原理

```
摄像头 (480×480, 120fps, 固定曝光 8000µs)
    │
    ▼
灰度 + 高斯模糊
    │
    ▼
霍夫梯度圆检测 (dp=1.2, param1=50, param2=35)
    │
    ▼
取最佳圆 → 坐标变换 → 机器人坐标系 (x, y, area)
    │
    ▼
PID 计算 (θ, φ)
    │
    ▼
逆运动学 → 三舵机角度 → 平台倾斜 → 球滚向中心
```

> 树莓派 OV5647 摄像头色彩管线特殊，HSV 颜色检测不可靠，采用纯霍夫圆方案。

## 可调参数

### `class_Camera.py` — 霍夫圆

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hough_dp` | 1.2 | 累加器分辨率 |
| `hough_param1` | 50 | Canny 高阈值（越低边缘越多） |
| `hough_param2` | 35 | 累加器阈值（越低圆越多） |
| `hough_minR` | 20 | 最小半径 (px) |
| `hough_maxR` | 120 | 最大半径 (px) |

### `main.py` — PID

| 参数 | 默认值 | 说明 |
|---|---|---|
| `K_PID[0]` (Kp) | 0.03 | 比例增益：误差响应速度 |
| `K_PID[1]` (Ki) | 0.0002 | 积分增益：消除稳态残差 |
| `K_PID[2]` (Kd) | 0.008 | 微分增益：制动防震荡 |
| `k` | 1.5 | φ 缩放系数：像素误差→倾斜角度 |
| `goal` | [0, 0] | 目标坐标（机器人坐标系像素） |
| `CTRL_DT` | 0.01 | 控制周期 (s) |

### `class_BBRobot.py` — 运动学

| 参数 | 默认值 | 说明 |
|---|---|---|
| `L` | [0.0408, 0.064, 0.080, 0.064] | 连杆长度 (m) |
| `ini_pos` | [0, 0, 0.085] | 初始姿态 (θ, φ, Pz) |
| `pz_max/min` | 0.105 / 0.060 | 高度限位 (m) |
| `phi_max` | 20 | 最大倾斜角 (°) |

## 故障排查

| 现象 | 排查 |
|---|---|
| 舵机不动 | `sudo i2cdetect -y 1` 检查 0x40 |
| 相机打不开 | `libcamera-hello --list-cameras`；`ps aux \| grep python` 杀掉残留进程 |
| 一直 MISS | 运行 `python capture_debug.py` 查看帧图像是否拍到球 |
| FPS 为 0 | 等待 2-3 秒累积 100 帧 |
| 舵机方向反 | `class_servo.py` 中 `_angle_to_duty` 的 `-angle` 取反符号 |
| 球频繁掉落 | 增大 Kp / 减小轨迹速度 / 检查 phi_max |

## 许可

基于原作者的程序架构，舵机驱动从 Futaba RS304MD 串口协议改为 PCA9685 I2C PWM，视觉检测从 HSV 改为霍夫梯度圆。
