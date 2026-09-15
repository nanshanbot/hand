"""使用 RGB-D 手部关键点连续控制 11 自由度机械手。

处理流程：采集彩色图和对齐深度图，检测右手关键点，将关键点反投影为
真实 3D 坐标，计算关节角并经过滤波、限速后通过串口发送给控制板。
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from rgbd_hand_3d import (
    BoneLengthConstraint,
    OrbbecCamera,
    PointSmoother,
    calculate_angle,
    landmark_pixels,
    reconstruct_points,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BASE_DIR = Path(__file__).resolve().parent
HAND_MODEL = BASE_DIR / "hand_landmarker.task"
SERIAL_PORT = "COM20"
BAUD_RATE = 115200

# 固件侧的机械限位；通道顺序必须与控制板和机械接线保持一致。
SERVO_MIN = 5
SERVO_MAXES = (140, 170, 140, 170, 170, 170, 170, 170, 170, 170, 170)
SERVO_NAMES = (
    "thumb rotation",
    "thumb proximal",
    "thumb distal",
    "index proximal",
    "index distal",
    "middle proximal",
    "middle distal",
    "ring proximal",
    "ring distal",
    "pinky proximal",
    "pinky distal",
)

# 每个通道的实测范围：(弯曲角, 伸展角)。
# 请用 --calibrate 采集后替换。方向由标定自动判定，这里保持"弯曲 < 伸展"。
MP_RANGES = (
    (  30.0,   47.2),  # CH00 thumb rotation  SWAPPED
    ( 136.1,  162.8),  # CH01 thumb proximal  OK
    ( 100.8,  162.5),  # CH02 thumb distal  OK
    ( 105.5,  167.7),  # CH03 index proximal  OK
    (  98.3,  168.9),  # CH04 index distal  OK
    (  81.3,  165.5),  # CH05 middle proximal  OK
    (  90.0,  160.5),  # CH06 middle distal  OK
    (  77.9,  164.4),  # CH07 ring proximal  OK
    ( 103.4,  161.1),  # CH08 ring distal  OK
    (  86.5,  170.3),  # CH09 pinky proximal  OK
    ( 107.8,  157.6),  # CH10 pinky distal  OK
)

# 如果某个通道标定后发现方向反了，用这个集合翻转。
INVERT_CHANNELS: set[int] = set()

FINGER_JOINTS = (
    (5, 6, 7, 8),
    (9, 10, 11, 12),
    (13, 14, 15, 16),
    (17, 18, 19, 20),
)
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)

SMOOTHING_ALPHA = 0.35
MAX_STEP_PER_FRAME = 8.0
SEND_INTERVAL = 0.05
RECONNECT_INTERVAL = 2.0
STABLE_FRAMES_REQUIRED = 3
MEDIAN_WINDOW = 5          # 中值滤波窗口
MIN_VALID_DEPTH_LANDMARKS = 16


def clamp(value: float, lower: float, upper: float) -> float:
    """把数值限制在闭区间 [lower, upper]。"""
    return max(lower, min(upper, value))


def extract_joint_angles(points: np.ndarray) -> list[float]:
    """从深度相机反投影得到的米制 3D 关键点提取 11 个关节角。"""
    angles = [calculate_angle(points[5], points[0], points[2])]
    angles.append(calculate_angle(points[1], points[2], points[3]))
    angles.append(calculate_angle(points[2], points[3], points[4]))
    for mcp, pip, dip, tip in FINGER_JOINTS:
        angles.append(calculate_angle(points[mcp], points[pip], points[dip]))
        angles.append(calculate_angle(points[pip], points[dip], points[tip]))
    return angles


def joint_angles_to_servo(joint_angles: list[float]) -> list[float]:
    """按标定范围把人体关节角线性映射为各通道舵机角度。"""
    servo_angles = []
    for channel, (joint_angle, (mp_min, mp_max)) in enumerate(zip(joint_angles, MP_RANGES)):
        span = mp_max - mp_min
        if abs(span) < 1e-6:
            ratio = 0.0
        else:
            ratio = clamp((joint_angle - mp_min) / span, 0.0, 1.0)
        if channel in INVERT_CHANNELS:
            ratio = 1.0 - ratio
        value = SERVO_MIN + ratio * (SERVO_MAXES[channel] - SERVO_MIN)
        servo_angles.append(clamp(value, SERVO_MIN, SERVO_MAXES[channel]))
    return servo_angles


class AngleFilter:
    """中值滤波去尖峰 + 一阶低通平滑。"""

    def __init__(self, channels: int):
        self.median_buffers = [deque(maxlen=MEDIAN_WINDOW) for _ in range(channels)]
        self.filtered: list[float] | None = None

    def update(self, raw: list[float]) -> list[float]:
        medians = []
        for channel, value in enumerate(raw):
            buf = self.median_buffers[channel]
            buf.append(value)
            medians.append(float(np.median(buf)))
        if self.filtered is None:
            self.filtered = medians[:]
        else:
            self.filtered = [
                old + SMOOTHING_ALPHA * (med - old)
                for old, med in zip(self.filtered, medians)
            ]
        return self.filtered[:]

    def reset(self) -> None:
        for buffer in self.median_buffers:
            buffer.clear()
        self.filtered = None


class ServoTracker:
    """让舵机角度以每帧最大步长逼近目标。"""

    def __init__(self, channels: int):
        self.current = [float(SERVO_MIN)] * channels

    def step(self, target: list[float]) -> list[float]:
        result = []
        for channel, (old, new) in enumerate(zip(self.current, target)):
            delta = clamp(new - old, -MAX_STEP_PER_FRAME, MAX_STEP_PER_FRAME)
            result.append(clamp(old + delta, SERVO_MIN, SERVO_MAXES[channel]))
        self.current = result
        return result


class HandSerial:
    """维护控制板串口连接，并仅发送发生变化的舵机角度。"""

    def __init__(self, requested_port: str | None) -> None:
        self.requested_port = requested_port
        self.connection = None
        self.port_name = None
        self.last_attempt = 0.0
        self.last_send = 0.0
        self.last_angles: tuple[int, ...] | None = None
        self.message = "waiting for serial"
        self.last_error = None

    @property
    def connected(self) -> bool:
        return self.connection is not None and self.connection.is_open

    def _candidate_ports(self) -> list[str]:
        if self.requested_port:
            return [self.requested_port]
        if list_ports is None:
            return []
        return [item.device for item in list_ports.comports()]

    def poll_connection(self, now: float) -> None:
        """定期尝试连接，并用 PING/PONG 避免误连其他串口设备。"""
        if self.connected or now - self.last_attempt < RECONNECT_INTERVAL:
            return
        self.last_attempt = now
        if serial is None:
            self.message = f"pyserial missing in {Path(sys.executable).name}"
            return

        candidates = self._candidate_ports()
        if not candidates:
            self.message = "waiting for serial device"
            return

        for port_name in candidates:
            candidate = None
            try:
                candidate = serial.Serial(
                    port_name, BAUD_RATE, timeout=0.08, write_timeout=0.2
                )
                candidate.reset_input_buffer()
                candidate.write(b"PING\r\n")
                deadline = time.monotonic() + 0.7
                while time.monotonic() < deadline:
                    if candidate.readline().strip() == b"PONG":
                        self.connection = candidate
                        self.port_name = port_name
                        self.last_angles = None
                        self.last_error = None
                        self.message = f"connected: {port_name}"
                        print(f"已连接灵巧手控制板：{port_name}")
                        return
                candidate.close()
            except (OSError, serial.SerialException) as exc:
                if candidate is not None and candidate.is_open:
                    candidate.close()
                error_text = str(exc)
                self.message = f"{port_name} busy/error: {error_text}"
                if error_text != self.last_error:
                    print(f"串口 {port_name} 打开失败：{error_text}")
                    self.last_error = error_text
        if self.requested_port:
            self.message = f"no HAND11 response on {self.requested_port}"
        else:
            self.message = "controller not found; retrying"

    def send_angles(self, angles: list[float], now: float) -> None:
        """按发送频率和机械限位下发发生变化的通道。"""
        if not self.connected or now - self.last_send < SEND_INTERVAL:
            return

        limited = tuple(
            int(round(clamp(value, SERVO_MIN, SERVO_MAXES[channel])))
            for channel, value in enumerate(angles)
        )
        if limited == self.last_angles:
            return

        commands = [
            f"SET {channel} {value}\r\n"
            for channel, value in enumerate(limited)
            if self.last_angles is None or value != self.last_angles[channel]
        ]
        try:
            pending = self.connection.read_all().decode("ascii", errors="replace")
            for line in pending.splitlines():
                if line.startswith("ERR"):
                    print("控制板返回：", line)
            for command in commands:
                self.connection.write(command.encode("ascii"))
                response = self.connection.readline().decode("ascii", errors="replace").strip()
                if response != "OK":
                    self.message = f"{self.port_name} reply: {response or 'timeout'}"
                    print(f"控制板未确认命令 {command.strip()}：{response or 'timeout'}")
                    return
            self.last_angles = limited
            self.last_send = now
            self.message = f"connected: {self.port_name} | TX {','.join(map(str, limited))}"
        except (OSError, serial.SerialException) as exc:
            self.message = f"serial disconnected: {exc}"
            print(self.message)
            self.close()

    def close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except (OSError, serial.SerialException):
                pass
        self.connection = None
        self.port_name = None


def find_target_hand(result, target_label: str):
    """返回指定手的图像关键点；深度相机负责提供真实 3D 坐标。"""
    if not result.hand_landmarks or not result.handedness:
        return None
    for index, handedness in enumerate(result.handedness):
        if handedness and handedness[0].category_name == target_label:
            return result.hand_landmarks[index]
    return None


def draw_hand(frame, landmarks, mirrored: bool) -> None:
    """在显示帧上绘制关键点骨架；镜像仅影响显示坐标。"""
    height, width = frame.shape[:2]
    pixels = []
    for lm in landmarks:
        x = 1.0 - lm.x if mirrored else lm.x
        pixels.append((int(x * width), int(lm.y * height)))
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, pixels[start], pixels[end], (70, 210, 120), 2, cv2.LINE_AA)
    for location in pixels:
        cv2.circle(frame, location, 4, (30, 170, 255), -1, cv2.LINE_AA)


def reconstruct_hand(landmarks, frame: np.ndarray, depth_map: np.ndarray,
                     camera: OrbbecCamera, smoother: PointSmoother,
                     bone_constraint: BoneLengthConstraint, args,
                     now: float) -> tuple[np.ndarray, np.ndarray, int]:
    """把 MediaPipe 像素关键点反投影为稳定的相机坐标系 3D 点。"""
    height, width = frame.shape[:2]
    pixels = landmark_pixels(landmarks, width, height)
    raw_points, _ = reconstruct_points(
        pixels, depth_map, camera.intrinsics,
        args.sample_radius, args.min_samples, args.min_depth, args.max_depth,
    )
    raw_valid_count = int(np.count_nonzero(np.all(np.isfinite(raw_points), axis=1)))
    constrained = bone_constraint.update(raw_points)
    points = smoother.update(constrained, now)
    return points, pixels, raw_valid_count


def draw_status(frame, serial_link: HandSerial, angles: list[float] | None,
                raw_angles: list[float] | None, stable_frames: int,
                control_enabled: bool, debug: bool, hand_detected: bool,
                depth_valid_count: int) -> None:
    """叠加串口、跟踪质量和 11 路目标角度等运行状态。"""
    serial_color = (70, 210, 120) if serial_link.connected else (0, 170, 255)
    control_state = "ON" if control_enabled else "PAUSED"
    cv2.putText(frame, f"SERIAL: {serial_link.message}", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, serial_color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"CONTROL: {control_state}  [SPACE toggle, Q quit]", (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (235, 235, 235), 2, cv2.LINE_AA)

    if angles is None:
        hand_state = (
            f"RIGHT HAND: depth insufficient ({depth_valid_count}/21)"
            if hand_detected else "RIGHT HAND: not detected"
        )
        cv2.putText(frame, hand_state, (12, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 170, 255), 2, cv2.LINE_AA)
        return

    tracking = "ready" if stable_frames >= STABLE_FRAMES_REQUIRED else "stabilizing"
    cv2.putText(frame, f"RIGHT HAND: {tracking}  DEPTH: {depth_valid_count}/21", (12, 88),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 210, 120), 2, cv2.LINE_AA)
    for channel, (name, angle) in enumerate(zip(SERVO_NAMES, angles)):
        column = 0 if channel < 6 else 1
        row = channel if channel < 6 else channel - 6
        x = 12 + column * 330
        y = 118 + row * 25
        if debug and raw_angles is not None:
            text = f"CH{channel:02d} {name:<16} raw{raw_angles[channel]:5.1f} -> {angle:3.0f} deg"
        else:
            text = f"CH{channel:02d} {name:<16} {angle:3.0f} deg"
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (245, 245, 245), 1, cv2.LINE_AA)


def parse_args() -> argparse.Namespace:
    """定义相机、深度过滤、平滑和标定相关命令行参数。"""
    parser = argparse.ArgumentParser(description="右手 RGB-D 实时控制 11 自由度灵巧手")
    parser.add_argument("--port", default=SERIAL_PORT, help="串口，例如 COM3；默认自动探测")
    parser.add_argument("--width", type=int, default=640, help="RGB-D 彩色流宽度")
    parser.add_argument("--height", type=int, default=480, help="RGB-D 彩色流高度")
    parser.add_argument("--fps", type=int, default=30, help="RGB-D 帧率")
    parser.add_argument("--min-depth", type=float, default=0.20, help="最小有效深度（米）")
    parser.add_argument("--max-depth", type=float, default=1.50, help="最大有效深度（米）")
    parser.add_argument("--sample-radius", type=int, default=3, help="关键点深度采样半径（像素）")
    parser.add_argument("--min-samples", type=int, default=8, help="局部深度最少有效像素数")
    parser.add_argument("--bone-history", type=int, default=90, help="骨长统计窗口")
    parser.add_argument("--bone-warmup", type=int, default=15, help="骨长约束启用前样本数")
    parser.add_argument("--bone-strength", type=float, default=0.75, help="骨长约束强度 0~1")
    parser.add_argument("--smooth-tau", type=float, default=0.055, help="3D 点平滑时间常数（秒）")
    parser.add_argument("--max-jump", type=float, default=0.080, help="单帧关键点最大位移（米）")
    parser.add_argument("--hold-frames", type=int, default=2, help="深度短暂丢失保持帧数")
    parser.add_argument("--mirror", action="store_true", default=True,
                        help="镜像显示（默认开启，不改变深度/识别坐标）")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false",
                        help="关闭镜像处理")
    parser.add_argument("--debug", action="store_true", help="画面叠加原始关节角")
    parser.add_argument("--calibrate", action="store_true",
                        help="标定模式：按 1 采伸直，按 2 采握拳，按 3 输出 MP_RANGES")
    return parser.parse_args()


def validate_args(args) -> None:
    """在打开硬件前检查参数组合，尽早给出可读错误。"""
    if args.width < 1 or args.height < 1 or args.fps < 1:
        raise ValueError("width、height 和 fps 必须为正数")
    if args.min_depth <= 0 or args.max_depth <= args.min_depth:
        raise ValueError("深度范围必须满足 0 < min-depth < max-depth")
    if args.sample_radius < 0 or args.min_samples < 1:
        raise ValueError("sample-radius >= 0 且 min-samples >= 1")
    if args.min_samples > (2 * args.sample_radius + 1) ** 2:
        raise ValueError("min-samples 不能超过深度采样区域像素数")
    if args.bone_history < args.bone_warmup or args.bone_warmup < 1:
        raise ValueError("bone-history 必须 >= bone-warmup >= 1")
    if not 0.0 <= args.bone_strength <= 1.0:
        raise ValueError("bone-strength 必须在 0~1 之间")
    if args.smooth_tau <= 0 or args.max_jump <= 0 or args.hold_frames < 0:
        raise ValueError("平滑参数必须为正数，hold-frames 允许为 0")


def run_calibration(args) -> int:
    """标定模式：手动切换姿态，按键采样，最后打印 MP_RANGES 和方向提示。"""
    if not HAND_MODEL.exists():
        print(f"找不到模型文件：{HAND_MODEL}")
        return 1

    camera = None
    landmarker = None
    try:
        camera = OrbbecCamera(args.width, args.height, args.fps)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1

    try:
        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(HAND_MODEL)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        landmarker = vision.HandLandmarker.create_from_options(options)
    except Exception as exc:
        camera.close()
        print(f"无法初始化 MediaPipe：{exc}", file=sys.stderr)
        return 1
    bone_constraint = BoneLengthConstraint(
        args.bone_history, args.bone_warmup, args.bone_strength
    )
    smoother = PointSmoother(args.smooth_tau, args.max_jump, args.hold_frames)

    target_label = "Right"
    print("标定模式：使用 Gemini 336 对齐深度计算真实 3D 角度。")
    print("操作：右手'完全伸直'按 1 采样（多次）；'完全握拳'按 2 采样（多次）；按 3 输出；Q 退出。")

    start_time = time.monotonic()
    timestamp_ms = -1
    open_samples: list[list[float]] = []
    fist_samples: list[list[float]] = []
    latest_raw: list[float] | None = None

    try:
        while True:
            captured = camera.read()
            if captured is None:
                continue
            frame, depth_map = captured
            now = time.monotonic()

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            current_timestamp_ms = int((now - start_time) * 1000)
            timestamp_ms = max(current_timestamp_ms, timestamp_ms + 1)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)
            display_landmarks = find_target_hand(result, target_label)

            display_frame = cv2.flip(frame, 1) if args.mirror else frame.copy()
            latest_raw = None
            if display_landmarks is not None:
                points, _, raw_valid_count = reconstruct_hand(
                    display_landmarks, frame, depth_map, camera, smoother,
                    bone_constraint, args, now,
                )
                candidate = extract_joint_angles(points)
                if (raw_valid_count >= MIN_VALID_DEPTH_LANDMARKS
                        and np.all(np.isfinite(candidate))):
                    latest_raw = candidate
                draw_hand(display_frame, display_landmarks, mirrored=args.mirror)
            else:
                smoother.reset()

            cv2.putText(display_frame, f"OPEN samples: {len(open_samples)}  [1]",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (70, 210, 120), 2)
            cv2.putText(display_frame, f"FIST samples: {len(fist_samples)}  [2]",
                        (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 170, 255), 2)
            cv2.putText(display_frame, "3 = print MP_RANGES, Q = quit",
                        (12, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (235, 235, 235), 2)
            if latest_raw is not None:
                for channel, value in enumerate(latest_raw):
                    col = 0 if channel < 6 else 1
                    row = channel if channel < 6 else channel - 6
                    x = 12 + col * 330
                    y = 118 + row * 25
                    cv2.putText(display_frame, f"CH{channel:02d} {SERVO_NAMES[channel]:<16} {value:6.1f}",
                                (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 245, 245), 1)

            cv2.imshow("RGB-D Calibration", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key == ord("1") and latest_raw is not None:
                open_samples.append(latest_raw[:])
                print(f"采样伸直 #{len(open_samples)}")
            if key == ord("2") and latest_raw is not None:
                fist_samples.append(latest_raw[:])
                print(f"采样握拳 #{len(fist_samples)}")
            if key == ord("3"):
                if not open_samples or not fist_samples:
                    print("请先各采集至少一个样本。")
                    continue
                open_avg = np.mean(np.array(open_samples), axis=0)
                fist_avg = np.mean(np.array(fist_samples), axis=0)
                diff = open_avg - fist_avg

                print("\n# 把下面这段替换掉 MP_RANGES")
                print("MP_RANGES = (")
                for channel in range(len(SERVO_NAMES)):
                    if diff[channel] >= 0:
                        # 伸直角 > 握拳角，方向正常
                        print(f"    ({fist_avg[channel]:6.1f}, {open_avg[channel]:6.1f}),"
                              f"  # CH{channel:02d} {SERVO_NAMES[channel]}  OK")
                    else:
                        # 方向反了，交换
                        print(f"    ({open_avg[channel]:6.1f}, {fist_avg[channel]:6.1f}),"
                              f"  # CH{channel:02d} {SERVO_NAMES[channel]}  SWAPPED")
                print(")")

                invert = [c for c in range(len(SERVO_NAMES)) if diff[c] < 0]
                if invert:
                    print(f"\n# 以下通道方向反了，已在上面的 MP_RANGES 里交换端点：{invert}")
                    print("# 如果你不想交换 MP_RANGES，也可以把这句改为：")
                    print(f"# INVERT_CHANNELS: set[int] = {set(invert)}")
                else:
                    print("\n# 所有通道方向正常，INVERT_CHANNELS 保持空集即可。")

                print(f"\n# open - fist = {np.round(diff, 1).tolist()}")
                small = [c for c in range(len(SERVO_NAMES)) if abs(diff[c]) < 15]
                if small:
                    print(f"# 注意：以下通道伸直/握拳差异过小（<15°），映射会不灵敏：{small}")
    except KeyboardInterrupt:
        pass
    finally:
        if landmarker is not None:
            landmarker.close()
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    """运行 RGB-D 实时跟随主循环。"""
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"参数错误：{exc}", file=sys.stderr)
        return 2

    if args.calibrate:
        return run_calibration(args)

    if not HAND_MODEL.exists():
        print(f"找不到模型文件：{HAND_MODEL}")
        return 1

    camera = None
    landmarker = None
    serial_link = HandSerial(args.port)

    try:
        camera = OrbbecCamera(args.width, args.height, args.fps)
        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=str(HAND_MODEL)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        landmarker = vision.HandLandmarker.create_from_options(options)
        bone_constraint = BoneLengthConstraint(
            args.bone_history, args.bone_warmup, args.bone_strength
        )
        point_smoother = PointSmoother(args.smooth_tau, args.max_jump, args.hold_frames)
        angle_filter = AngleFilter(len(SERVO_MAXES))
        servo_tracker = ServoTracker(len(SERVO_MAXES))
        start_time = time.monotonic()
        timestamp_ms = -1
        stable_frames = 0
        control_enabled = True

        intrinsics = camera.intrinsics
        print(
            f"Gemini 336 已就绪：{intrinsics.width}x{intrinsics.height}，"
            f"fx={intrinsics.fx:.1f}，fy={intrinsics.fy:.1f}"
        )
        print("开始监测右手；空格键暂停/恢复控制，Q 键退出。")
        if args.debug:
            print("调试模式：raw 为深度反投影得到的真实 3D 关节角。")

        while True:
            captured = camera.read()
            if captured is None:
                continue
            frame, depth_map = captured
            now = time.monotonic()

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            current_timestamp_ms = int((now - start_time) * 1000)
            timestamp_ms = max(current_timestamp_ms, timestamp_ms + 1)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)
            display_landmarks = find_target_hand(result, "Right")

            serial_link.poll_connection(now)
            raw_angles = None
            shown_angles = None
            raw_valid_count = 0
            # 手部或深度失效时清空滤波状态，避免重现后沿用过期数据。
            if display_landmarks is None:
                stable_frames = 0
                point_smoother.reset()
                angle_filter.reset()
            else:
                points, _, raw_valid_count = reconstruct_hand(
                    display_landmarks, frame, depth_map, camera, point_smoother,
                    bone_constraint, args, now,
                )
                candidate = extract_joint_angles(points)
                if (raw_valid_count >= MIN_VALID_DEPTH_LANDMARKS
                        and np.all(np.isfinite(candidate))):
                    raw_angles = candidate
                    stable_frames += 1
                    # 角度先去尖峰和平滑，再映射到舵机并限制每帧最大变化量。
                    filtered_raw = angle_filter.update(raw_angles)
                    targets = joint_angles_to_servo(filtered_raw)
                    shown_angles = servo_tracker.step(targets)
                    if control_enabled and stable_frames >= STABLE_FRAMES_REQUIRED:
                        serial_link.send_angles(shown_angles, now)
                else:
                    stable_frames = 0
                    angle_filter.reset()

            display_frame = cv2.flip(frame, 1) if args.mirror else frame.copy()
            if display_landmarks is not None:
                draw_hand(display_frame, display_landmarks, mirrored=args.mirror)

            draw_status(display_frame, serial_link, shown_angles, raw_angles,
                        stable_frames, control_enabled, args.debug,
                        display_landmarks is not None, raw_valid_count)
            cv2.imshow("RGB-D Right Hand Robot Control", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key == ord(" "):
                control_enabled = not control_enabled
                serial_link.last_angles = None
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        serial_link.close()
        if landmarker is not None:
            landmarker.close()
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
