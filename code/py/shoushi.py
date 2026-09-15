"""使用普通摄像头识别预设手势并控制 11 自由度机械手。

MediaPipe 输出离散手势名称，程序将其转换为预设舵机姿态；稳定帧判断
和角度限速用于减少识别抖动造成的机械冲击。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BASE_DIR = Path(__file__).resolve().parent
GESTURE_MODEL = BASE_DIR / "gesture_recognizer.task"
CAMERA_INDEX = 0
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

# MediaPipe 内置 8 种手势
CANNED_GESTURES = (
    "None",
    "Closed_Fist",
    "Open_Palm",
    "Pointing_Up",
    "Thumb_Down",
    "Thumb_Up",
    "Victory",
    "ILoveYou",
)

# 手势 -> 11 路舵机角度。占位值，请用 --calibrate 采集后替换。
# 顺序与 SERVO_MAXES 一致。
# 每个手势给一组 (11,) 的目标角度。
GESTURE_TO_SERVO: dict[str, tuple[int, ...]] = {
    "Closed_Fist": (5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5),
    "Open_Palm":   (140, 170, 140, 170, 170, 170, 170, 170, 170, 170, 170),
    "Pointing_Up": (5, 170, 170, 5, 170, 5, 5, 5, 5, 5, 5),
    "Thumb_Up":    (140, 170, 140, 5, 5, 5, 5, 5, 5, 5, 5),
    "Thumb_Down":  (5, 5, 5, 170, 170, 170, 170, 170, 170, 170, 170),
    "Victory":     (5, 5, 170, 5, 170, 5, 170, 5, 5, 5, 5),
    "ILoveYou":    (140, 170, 140, 5, 170, 5, 170, 5, 5, 5, 5),
    "None":        (5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5),
}

# 每个通道的舵机运动速度限制（度/帧），防止跳变
MAX_STEP_PER_FRAME = 6.0
SEND_INTERVAL = 0.05
RECONNECT_INTERVAL = 2.0
STABLE_FRAMES_REQUIRED = 3      # 手势连续稳定多少帧才切换
GESTURE_SCORE_THRESHOLD = 0.55  # 低于此置信度视为 None

HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)


def clamp(value: float, lower: float, upper: float) -> float:
    """把数值限制在闭区间 [lower, upper]。"""
    return max(lower, min(upper, value))


def step_towards(current: list[float], target: list[float]) -> list[float]:
    """让 current 以每帧最多 MAX_STEP_PER_FRAME 的速度逼近 target。"""
    result = []
    for channel, (old, new) in enumerate(zip(current, target)):
        delta = clamp(new - old, -MAX_STEP_PER_FRAME, MAX_STEP_PER_FRAME)
        result.append(clamp(old + delta, SERVO_MIN, SERVO_MAXES[channel]))
    return result


class HandSerial:
    """自动连接机械手控制板，并增量发送舵机角度。"""

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
        """定期尝试连接，并通过 PING/PONG 确认目标设备。"""
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
        """限制发送频率，只下发相对上次发生变化的通道。"""
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
                    print(f"控制板未确认命令 {command.strip()}：{response or '超时'}")
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
    """返回指定标签手的 (display_landmarks, index)。找不到返回 (None, None)。"""
    if not result.hand_landmarks or not result.handedness:
        return None, None
    for index, handedness in enumerate(result.handedness):
        if handedness and handedness[0].category_name == target_label:
            return result.hand_landmarks[index], index
    return None, None


def draw_hand(frame, landmarks, mirrored: bool) -> None:
    """在摄像头画面上绘制 MediaPipe 手部骨架。"""
    height, width = frame.shape[:2]
    pixels = []
    for lm in landmarks:
        x = lm.x
        if mirrored:
            x = 1.0 - x
        pixels.append((int(x * width), int(lm.y * height)))
    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, pixels[start], pixels[end], (70, 210, 120), 2, cv2.LINE_AA)
    for location in pixels:
        cv2.circle(frame, location, 4, (30, 170, 255), -1, cv2.LINE_AA)


def draw_status(frame, serial_link: HandSerial, gesture_name: str,
                gesture_score: float, angles: list[float] | None,
                stable_frames: int, control_enabled: bool) -> None:
    """叠加连接状态、识别置信度和当前舵机角度。"""
    serial_color = (70, 210, 120) if serial_link.connected else (0, 170, 255)
    control_state = "ON" if control_enabled else "PAUSED"
    cv2.putText(frame, f"SERIAL: {serial_link.message}", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, serial_color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"CONTROL: {control_state}  [SPACE toggle, Q quit]", (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (235, 235, 235), 2, cv2.LINE_AA)

    if gesture_name is None:
        cv2.putText(frame, "GESTURE: no hand", (12, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 170, 255), 2, cv2.LINE_AA)
        return

    tracking = "ready" if stable_frames >= STABLE_FRAMES_REQUIRED else "stabilizing"
    cv2.putText(frame, f"GESTURE: {gesture_name} ({gesture_score:.2f}) {tracking}",
                (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 210, 120), 2, cv2.LINE_AA)

    if angles is None:
        return
    for channel, (name, angle) in enumerate(zip(SERVO_NAMES, angles)):
        column = 0 if channel < 6 else 1
        row = channel if channel < 6 else channel - 6
        x = 12 + column * 330
        y = 118 + row * 25
        text = f"CH{channel:02d} {name:<16} {angle:3.0f} deg"
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (245, 245, 245), 1, cv2.LINE_AA)


def parse_args() -> argparse.Namespace:
    """解析串口、摄像头、模型和镜像相关参数。"""
    parser = argparse.ArgumentParser(description="手势识别实时控制 11 自由度灵巧手")
    parser.add_argument("--port", default=SERIAL_PORT, help="串口，例如 COM3；默认自动探测")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="摄像头编号")
    parser.add_argument("--model", default=str(GESTURE_MODEL), help="手势模型路径")
    parser.add_argument("--mirror", action="store_true", default=True,
                        help="摄像头有镜像（默认开启）；此时实际右手会被标成 Left")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false",
                        help="关闭镜像处理")
    parser.add_argument("--calibrate", action="store_true",
                        help="标定模式：为每个手势采样一组舵机角度")
    return parser.parse_args()


def run_calibration(args) -> int:
    """标定模式：对每个手势，摆出目标手型，按数字键采样当前手势的舵机角度。

    这个模式不连串口，只采集角度值，方便你确定每个手势该给什么舵机角度。
    """
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"找不到手势模型文件：{model_path}")
        print("下载地址：https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
              "gesture_recognizer/float16/1/gesture_recognizer.task")
        return 1

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"无法打开摄像头 {args.camera}")
        return 1
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    options = vision.GestureRecognizerOptions(
        base_options=python.BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    recognizer = vision.GestureRecognizer.create_from_options(options)

    target_label = "Left" if args.mirror else "Right"
    print(f"标定模式：寻找标签为 {target_label} 的手（--mirror={args.mirror}）")
    print("操作：摆出某个手势后，用键盘输入该手势对应的舵机角度（11 个整数，空格分隔），")
    print("回车确认。输入空行跳过。按 Q 退出并打印结果。")

    collected: dict[str, tuple[int, ...]] = {}

    try:
        while camera.isOpened():
            ok, frame = camera.read()
            if not ok:
                print("摄像头读取失败")
                break

            mp_frame = cv2.flip(frame, 1) if args.mirror else frame
            rgb = cv2.cvtColor(mp_frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = recognizer.recognize(mp_image)

            gesture_name = "None"
            score = 0.0
            landmarks, _ = find_target_hand(result, target_label)
            if result.gestures and len(result.gestures) > 0:
                top = result.gestures[0][0]
                gesture_name = top.category_name
                score = top.score

            if landmarks is not None:
                draw_hand(frame, landmarks, mirrored=args.mirror)

            cv2.putText(frame, f"GESTURE: {gesture_name} ({score:.2f})",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 210, 120), 2)
            cv2.putText(frame, "Type 11 servo angles in terminal, Enter to save",
                        (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1)
            cv2.imshow("Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break

            # 用阻塞输入采一组角度
            if key == ord(" "):
                print(f"\n当前手势：{gesture_name} (score={score:.2f})")
                text = input("输入 11 个舵机角度（空格分隔，空行跳过）：").strip()
                if not text:
                    continue
                parts = text.split()
                if len(parts) != len(SERVO_MAXES):
                    print(f"需要 {len(SERVO_MAXES)} 个值，收到 {len(parts)} 个，已忽略。")
                    continue
                try:
                    values = tuple(
                        int(clamp(int(p), SERVO_MIN, SERVO_MAXES[i]))
                        for i, p in enumerate(parts)
                    )
                except ValueError:
                    print("输入包含非整数，已忽略。")
                    continue
                collected[gesture_name] = values
                print(f"已记录 {gesture_name} = {values}")
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        cv2.destroyAllWindows()

    print("\n# 把下面这段替换掉 GESTURE_TO_SERVO")
    print("GESTURE_TO_SERVO: dict[str, tuple[int, ...]] = {")
    for name in CANNED_GESTURES:
        if name in collected:
            print(f'    "{name}": {collected[name]},')
        else:
            print(f'    "{name}": (5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5),  # 未采集')
    print("}")
    return 0


def main() -> int:
    """运行手势识别、姿态映射和串口控制主循环。"""
    args = parse_args()

    if args.calibrate:
        return run_calibration(args)

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"找不到手势模型文件：{model_path}")
        print("下载地址：https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
              "gesture_recognizer/float16/1/gesture_recognizer.task")
        return 1

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"无法打开摄像头 {args.camera}")
        return 1

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    options = vision.GestureRecognizerOptions(
        base_options=python.BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    recognizer = vision.GestureRecognizer.create_from_options(options)

    serial_link = HandSerial(args.port)
    current_angles = [float(SERVO_MIN)] * len(SERVO_MAXES)
    target_angles = list(GESTURE_TO_SERVO["None"])
    stable_frames = 0
    last_gesture = "None"
    control_enabled = True

    target_label = "Left" if args.mirror else "Right"
    print(f"开始监测右手。目标标签：{target_label}（--mirror={args.mirror}）")
    print("空格键暂停/恢复控制，Q 键退出。")
    print("支持的手势：")
    for name in CANNED_GESTURES:
        print(f"  {name}")

    try:
        while camera.isOpened():
            ok, frame = camera.read()
            if not ok:
                print("摄像头读取失败")
                break

            mp_frame = cv2.flip(frame, 1) if args.mirror else frame
            rgb_frame = cv2.cvtColor(mp_frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            result = recognizer.recognize(mp_image)

            gesture_name = None
            gesture_score = 0.0
            landmarks, _ = find_target_hand(result, target_label)
            if result.gestures and len(result.gestures) > 0:
                top = result.gestures[0][0]
                if top.score >= GESTURE_SCORE_THRESHOLD:
                    gesture_name = top.category_name
                    gesture_score = top.score

            now = time.monotonic()
            serial_link.poll_connection(now)

            if landmarks is not None:
                draw_hand(frame, landmarks, mirrored=args.mirror)

            # 只有同一手势连续出现足够帧数，才更新机械手目标姿态。
            if gesture_name is None:
                stable_frames = 0
                last_gesture = "None"
            else:
                if gesture_name == last_gesture:
                    stable_frames += 1
                else:
                    last_gesture = gesture_name
                    stable_frames = 1
                if stable_frames == STABLE_FRAMES_REQUIRED:
                    target_angles = list(GESTURE_TO_SERVO.get(
                        gesture_name, GESTURE_TO_SERVO["None"]))

            # 逐帧逼近目标角度，避免姿态切换时舵机瞬间大幅运动。
            current_angles = step_towards(current_angles, target_angles)

            if control_enabled and stable_frames >= STABLE_FRAMES_REQUIRED:
                serial_link.send_angles(current_angles, now)

            draw_status(frame, serial_link, gesture_name, gesture_score,
                        current_angles, stable_frames, control_enabled)
            cv2.imshow("Gesture Robot Control", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key == ord(" "):
                control_enabled = not control_enabled
                serial_link.last_angles = None
    except KeyboardInterrupt:
        pass
    finally:
        serial_link.close()
        camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
