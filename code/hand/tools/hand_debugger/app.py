"""11 自由度机械手的 Tkinter 串口调试工具。

界面负责编辑和预览各通道角度；串口读取放在后台线程中，并通过队列
把消息交回 Tk 主线程，避免阻塞界面事件循环。
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


ANGLE_MIN = 5
ANGLE_MAX = 170
THUMB_ANGLE_MAX = 140
SERVO_COUNT = 11
BAUD_RATE = 115200
# 通道名称和角度上限与 STM32 固件中的定义保持一致。
SERVO_NAMES = (
    "拇指旋转",
    "拇指近端",
    "拇指远端",
    "食指近端",
    "食指远端",
    "中指近端",
    "中指远端",
    "无名指近端",
    "无名指远端",
    "小指近端",
    "小指远端",
)
ANGLE_MAXES = tuple(
    THUMB_ANGLE_MAX if channel in (0, 2) else ANGLE_MAX
    for channel in range(SERVO_COUNT)
)

COLORS = {
    "bg": "#F3F5F4",
    "surface": "#FFFFFF",
    "ink": "#18211E",
    "muted": "#68736F",
    "line": "#D8DEDB",
    "teal": "#087E6B",
    "teal_dark": "#056353",
    "teal_soft": "#DDEFEA",
    "amber": "#D88918",
    "red": "#B94A48",
    "palm": "#CBD8D3",
    "bone": "#263D36",
}


class SerialLink:
    """封装串口读写，并把后台线程事件投递到线程安全队列。"""

    def __init__(self, events: queue.Queue[tuple[str, str]]) -> None:
        self.events = events
        self.port = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._reader: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        """打开串口并启动后台接收线程。"""
        if serial is None:
            raise RuntimeError("缺少 pyserial，请使用 run.ps1 启动")
        self.disconnect()
        self.port = serial.Serial(port_name, BAUD_RATE, timeout=0.15, write_timeout=0.5)
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def disconnect(self) -> None:
        self._stop.set()
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None

    def send(self, command: str) -> None:
        """为命令补充行结束符后发送，并记录到界面日志队列。"""
        if not self.connected:
            raise RuntimeError("串口尚未连接")
        payload = (command.strip() + "\r\n").encode("ascii")
        with self._write_lock:
            self.port.write(payload)
        self.events.put(("tx", command.strip()))

    def _read_loop(self) -> None:
        while not self._stop.is_set() and self.connected:
            try:
                raw = self.port.readline()
                if raw:
                    self.events.put(("rx", raw.decode("ascii", errors="replace").strip()))
            except (serial.SerialException, OSError) as exc:
                self.events.put(("error", str(exc)))
                break


class HandCanvas(tk.Canvas):
    """根据 11 路关节角绘制简化的机械手姿态预览。"""

    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(master, highlightthickness=0, bg=COLORS["surface"], **kwargs)
        self.angles = [ANGLE_MIN] * SERVO_COUNT
        self.bind("<Configure>", lambda _event: self.draw())

    def set_angles(self, angles: list[int]) -> None:
        self.angles = angles[:]
        self.draw()

    @staticmethod
    def _point(origin: tuple[float, float], length: float, direction: float) -> tuple[float, float]:
        return (
            origin[0] + length * math.cos(direction),
            origin[1] + length * math.sin(direction),
        )

    def _segment(self, start: tuple[float, float], end: tuple[float, float], width: int) -> None:
        self.create_line(*start, *end, fill=COLORS["bone"], width=width, capstyle=tk.ROUND)
        self.create_oval(end[0] - 5, end[1] - 5, end[0] + 5, end[1] + 5,
                         fill=COLORS["amber"], outline=COLORS["surface"], width=2)

    def draw(self) -> None:
        """按当前控件尺寸和舵机角度重新绘制整只手。"""
        self.delete("all")
        width = max(self.winfo_width(), 360)
        height = max(self.winfo_height(), 390)
        cx = width / 2
        palm_top = height * 0.43
        palm_bottom = height * 0.83

        self.create_text(24, 20, anchor="nw", text="姿态预览", fill=COLORS["ink"],
                         font=("Microsoft YaHei UI", 13, "bold"))
        self.create_text(width - 24, 22, anchor="ne", text="5° 握拳  ·  170° 张开",
                         fill=COLORS["muted"], font=("Microsoft YaHei UI", 9))
        palm = (cx - 83, palm_top, cx + 78, palm_top - 5,
                cx + 89, palm_bottom - 28, cx + 54, palm_bottom,
                cx - 50, palm_bottom, cx - 91, palm_bottom - 42)
        self.create_polygon(palm, fill=COLORS["palm"], outline=COLORS["bone"], width=3,
                            smooth=True)

        bases = [cx - 56, cx - 18, cx + 22, cx + 58]
        finger_lengths = [(72, 57), (82, 63), (76, 59), (64, 50)]
        for finger_index, (base_x, lengths) in enumerate(zip(bases, finger_lengths)):
            a1 = self.angles[3 + finger_index * 2]
            a2 = self.angles[4 + finger_index * 2]
            flex1 = (ANGLE_MAX - a1) / (ANGLE_MAX - ANGLE_MIN)
            flex2 = (ANGLE_MAX - a2) / (ANGLE_MAX - ANGLE_MIN)
            direction1 = -math.pi / 2 + math.radians(76 * flex1)
            direction2 = direction1 + math.radians(88 * flex2)
            p0 = (base_x, palm_top + (5 if finger_index in (0, 3) else 0))
            p1 = self._point(p0, lengths[0], direction1)
            p2 = self._point(p1, lengths[1], direction2)
            self._segment(p0, p1, 18)
            self._segment(p1, p2, 15)

        thumb_flex = [
            (ANGLE_MAXES[i] - self.angles[i]) / (ANGLE_MAXES[i] - ANGLE_MIN)
            for i in range(3)
        ]
        p0 = (cx - 78, palm_top + 92)
        d0 = math.radians(205 - 40 * thumb_flex[0])
        d1 = d0 + math.radians(48 * thumb_flex[1])
        d2 = d1 + math.radians(55 * thumb_flex[2])
        p1 = self._point(p0, 55, d0)
        p2 = self._point(p1, 48, d1)
        p3 = self._point(p2, 38, d2)
        self._segment(p0, p1, 19)
        self._segment(p1, p2, 16)
        self._segment(p2, p3, 13)

        avg = round(sum(self.angles) / SERVO_COUNT)
        self.create_text(cx, height - 22, text=f"平均角度  {avg}°", fill=COLORS["muted"],
                         font=("Microsoft YaHei UI", 10))


class HandDebugger(tk.Tk):
    """组织串口连接、舵机控件、姿态文件和通信日志。"""

    def __init__(self) -> None:
        super().__init__()
        self.title("11 自由度灵巧手调试台")
        self.geometry("1240x820")
        self.minsize(1040, 700)
        self.configure(bg=COLORS["bg"])

        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.link = SerialLink(self.events)
        self.angle_vars = [tk.IntVar(value=ANGLE_MIN) for _ in range(SERVO_COUNT)]
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")
        self.live_var = tk.BooleanVar(value=True)
        self._updating = False
        self._send_jobs: dict[int, str] = {}

        self._configure_style()
        self._build_ui()
        self.refresh_ports()
        self.after(60, self._poll_events)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("Surface.TFrame", background=COLORS["surface"])
        style.configure("TLabel", background=COLORS["bg"], foreground=COLORS["ink"],
                        font=("Microsoft YaHei UI", 10))
        style.configure("Surface.TLabel", background=COLORS["surface"], foreground=COLORS["ink"],
                        font=("Microsoft YaHei UI", 10))
        style.configure("Muted.TLabel", background=COLORS["bg"], foreground=COLORS["muted"],
                        font=("Microsoft YaHei UI", 9))
        style.configure("Title.TLabel", background=COLORS["bg"], foreground=COLORS["ink"],
                        font=("Microsoft YaHei UI", 20, "bold"))
        style.configure("Accent.TButton", background=COLORS["teal"], foreground="white",
                        padding=(15, 8), borderwidth=0, font=("Microsoft YaHei UI", 10, "bold"))
        style.map("Accent.TButton", background=[("active", COLORS["teal_dark"])])
        style.configure("TButton", padding=(11, 7), font=("Microsoft YaHei UI", 9))
        style.configure("TCheckbutton", background=COLORS["bg"], font=("Microsoft YaHei UI", 9))
        style.configure("Horizontal.TScale", background=COLORS["surface"], troughcolor=COLORS["line"])

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=(24, 18, 24, 20))
        root.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(root)
        header.pack(fill=tk.X, pady=(0, 14))
        ttk.Label(header, text="灵巧手调试台", style="Title.TLabel").pack(side=tk.LEFT)
        ttk.Label(header, text="11-DOF  ·  PCA9685", style="Muted.TLabel").pack(side=tk.LEFT, padx=14, pady=(8, 0))

        connect = ttk.Frame(header)
        connect.pack(side=tk.RIGHT)
        self.status_dot = tk.Canvas(connect, width=14, height=14, bg=COLORS["bg"], highlightthickness=0)
        self.status_dot.pack(side=tk.LEFT, padx=(0, 5))
        self._draw_status(False)
        ttk.Label(connect, textvariable=self.status_var).pack(side=tk.LEFT, padx=(0, 12))
        self.port_box = ttk.Combobox(connect, textvariable=self.port_var, width=14, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(connect, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=(0, 6))
        self.connect_button = ttk.Button(connect, text="连接", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT)

        body = ttk.Panedwindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, style="Surface.TFrame", padding=1)
        right = ttk.Frame(body, style="Surface.TFrame", padding=(18, 14))
        body.add(left, weight=5)
        body.add(right, weight=7)

        self.hand_canvas = HandCanvas(left, height=445)
        self.hand_canvas.pack(fill=tk.BOTH, expand=True)

        log_header = ttk.Frame(left, style="Surface.TFrame", padding=(16, 6, 10, 4))
        log_header.pack(fill=tk.X)
        ttk.Label(log_header, text="通信日志", style="Surface.TLabel",
                  font=("Microsoft YaHei UI", 11, "bold")).pack(side=tk.LEFT)
        ttk.Button(log_header, text="清空", command=lambda: self.log.delete("1.0", tk.END)).pack(side=tk.RIGHT)
        self.log = tk.Text(left, height=8, bg="#101815", fg="#CDE3DB", insertbackground="white",
                           relief=tk.FLAT, padx=12, pady=9, font=("Cascadia Mono", 9), state=tk.NORMAL)
        self.log.pack(fill=tk.X)
        self.log.tag_configure("tx", foreground="#7FD5C0")
        self.log.tag_configure("rx", foreground="#F3C66B")
        self.log.tag_configure("error", foreground="#FF8E8A")

        toolbar = ttk.Frame(right, style="Surface.TFrame")
        toolbar.pack(fill=tk.X, pady=(0, 12))
        ttk.Label(toolbar, text="关节角度", style="Surface.TLabel",
                  font=("Microsoft YaHei UI", 13, "bold")).pack(side=tk.LEFT)
        ttk.Checkbutton(toolbar, text="实时下发", variable=self.live_var).pack(side=tk.RIGHT)

        controls = ttk.Frame(right, style="Surface.TFrame")
        controls.pack(fill=tk.BOTH, expand=True)
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)
        for channel in range(SERVO_COUNT):
            column = 0 if channel < 6 else 1
            row = channel if channel < 6 else channel - 6
            control = self._make_servo_control(controls, channel)
            control.grid(row=row, column=column, sticky="ew", padx=(0, 13) if column == 0 else (13, 0), pady=4)

        separator = ttk.Separator(right)
        separator.pack(fill=tk.X, pady=(12, 11))
        presets = ttk.Frame(right, style="Surface.TFrame")
        presets.pack(fill=tk.X)
        for column in range(6):
            presets.columnconfigure(column, weight=1, uniform="actions")
        actions = (
            ("握拳  5°", lambda: self.set_uniform(5), "TButton"),
            ("半握  90°", lambda: self.set_uniform(90), "TButton"),
            ("张开  170°", lambda: self.set_uniform(170), "TButton"),
            ("发送全部", self.send_all, "Accent.TButton"),
            ("保存姿态", self.save_pose, "TButton"),
            ("载入姿态", self.load_pose, "TButton"),
        )
        for column, (text, command, style) in enumerate(actions):
            ttk.Button(presets, text=text, style=style, command=command).grid(
                row=0, column=column, sticky="ew", padx=(0 if column == 0 else 4, 0 if column == 5 else 4)
            )

    def _make_servo_control(self, parent: ttk.Frame, channel: int) -> ttk.Frame:
        frame = ttk.Frame(parent, style="Surface.TFrame")
        frame.columnconfigure(1, weight=1)
        badge = tk.Label(frame, text=f"CH{channel}", width=4, bg=COLORS["teal_soft"], fg=COLORS["teal_dark"],
                         font=("Cascadia Mono", 9, "bold"), padx=4, pady=3)
        badge.grid(row=0, column=0, rowspan=2, padx=(0, 9))
        ttk.Label(frame, text=SERVO_NAMES[channel], style="Surface.TLabel").grid(row=0, column=1, sticky="w")
        value = ttk.Spinbox(frame, from_=ANGLE_MIN, to=ANGLE_MAXES[channel], width=5,
                            textvariable=self.angle_vars[channel], command=lambda c=channel: self._angle_changed(c))
        value.grid(row=0, column=2, sticky="e")
        ttk.Label(frame, text="°", style="Surface.TLabel").grid(row=0, column=3, padx=(2, 0))
        scale = ttk.Scale(frame, from_=ANGLE_MIN, to=ANGLE_MAXES[channel], variable=self.angle_vars[channel],
                          command=lambda _value, c=channel: self._angle_changed(c))
        scale.grid(row=1, column=1, columnspan=3, sticky="ew", pady=(2, 0))
        scale.bind("<ButtonRelease-1>", lambda _event, c=channel: self._slider_released(c))
        value.bind("<Return>", lambda _event, c=channel: self.send_channel(c))
        value.bind("<FocusOut>", lambda _event, c=channel: self._normalize_angle(c))
        return frame

    def refresh_ports(self) -> None:
        ports = [] if list_ports is None else [item.device for item in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.link.connected:
            self.link.disconnect()
            self._set_connected(False)
            self._append_log("串口已断开", "error")
            return
        if not self.port_var.get():
            messagebox.showwarning("没有串口", "请连接 USB 转串口设备后刷新端口列表。")
            return
        try:
            self.link.connect(self.port_var.get())
            self._set_connected(True)
            self._append_log(f"已连接 {self.port_var.get()} @ {BAUD_RATE}", "rx")
            self.link.send("PING")
            self.link.send("GET")
        except Exception as exc:
            self._set_connected(False)
            messagebox.showerror("连接失败", str(exc))

    def _set_connected(self, connected: bool) -> None:
        self.status_var.set("已连接" if connected else "未连接")
        self.connect_button.configure(text="断开" if connected else "连接")
        self._draw_status(connected)

    def _draw_status(self, connected: bool) -> None:
        self.status_dot.delete("all")
        color = COLORS["teal"] if connected else COLORS["muted"]
        self.status_dot.create_oval(2, 2, 12, 12, fill=color, outline="")

    def _normalize_angle(self, channel: int) -> int:
        try:
            angle = int(float(self.angle_vars[channel].get()))
        except (ValueError, tk.TclError):
            angle = ANGLE_MIN
        angle = max(ANGLE_MIN, min(ANGLE_MAXES[channel], angle))
        self.angle_vars[channel].set(angle)
        self._update_preview()
        return angle

    def _angle_changed(self, channel: int) -> None:
        if self._updating:
            return
        self._update_preview()
        if self.live_var.get() and self.link.connected:
            # 拖动滑块时合并密集事件，减少串口命令堆积。
            old_job = self._send_jobs.pop(channel, None)
            if old_job:
                self.after_cancel(old_job)
            self._send_jobs[channel] = self.after(90, lambda c=channel: self.send_channel(c))

    def _slider_released(self, channel: int) -> None:
        if self.live_var.get():
            self.send_channel(channel)

    def _update_preview(self) -> None:
        angles = []
        for channel, variable in enumerate(self.angle_vars):
            try:
                angles.append(max(ANGLE_MIN, min(ANGLE_MAXES[channel], round(float(variable.get())))))
            except (ValueError, tk.TclError):
                angles.append(ANGLE_MIN)
        self.hand_canvas.set_angles(angles)

    def send_channel(self, channel: int) -> None:
        self._send_jobs.pop(channel, None)
        if not self.link.connected:
            return
        try:
            self.link.send(f"SET {channel} {self._normalize_angle(channel)}")
        except Exception as exc:
            self._handle_link_error(str(exc))

    def send_all(self) -> None:
        if not self.link.connected:
            messagebox.showwarning("尚未连接", "请先选择串口并连接。")
            return
        try:
            for channel in range(SERVO_COUNT):
                self.link.send(f"SET {channel} {self._normalize_angle(channel)}")
                time.sleep(0.003)
        except Exception as exc:
            self._handle_link_error(str(exc))

    def set_uniform(self, angle: int) -> None:
        self._updating = True
        for channel, variable in enumerate(self.angle_vars):
            variable.set(min(angle, ANGLE_MAXES[channel]))
        self._updating = False
        self._update_preview()
        if self.link.connected:
            try:
                for channel in range(SERVO_COUNT):
                    self.link.send(f"SET {channel} {self.angle_vars[channel].get()}")
                    time.sleep(0.003)
            except Exception as exc:
                self._handle_link_error(str(exc))

    def save_pose(self) -> None:
        """把当前 11 路角度保存为带版本标识的 JSON 姿态文件。"""
        destination = filedialog.asksaveasfilename(
            title="保存姿态", defaultextension=".json", filetypes=[("姿态文件", "*.json")]
        )
        if not destination:
            return
        angles = [self._normalize_angle(i) for i in range(SERVO_COUNT)]
        data = {"format": "hand11-pose-v1", "angles": angles}
        Path(destination).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    def load_pose(self) -> None:
        """校验并载入 JSON 姿态文件；载入本身不会自动发送。"""
        source = filedialog.askopenfilename(title="载入姿态", filetypes=[("姿态文件", "*.json")])
        if not source:
            return
        try:
            data = json.loads(Path(source).read_text(encoding="utf-8"))
            angles = data["angles"]
            if len(angles) != SERVO_COUNT or any(
                not isinstance(value, int) or not ANGLE_MIN <= value <= ANGLE_MAXES[channel]
                for channel, value in enumerate(angles)
            ):
                raise ValueError("姿态必须包含 11 个整数；拇指旋转、远端为 5–140°，其余为 5–170°")
            self._apply_state(angles)
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            messagebox.showerror("载入失败", str(exc))

    def _apply_state(self, angles: list[int]) -> None:
        self._updating = True
        for variable, angle in zip(self.angle_vars, angles):
            variable.set(angle)
        self._updating = False
        self._update_preview()

    def _poll_events(self) -> None:
        """在 Tk 主线程中消费串口事件并同步控制器状态。"""
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            self._append_log(("> " if kind == "tx" else "< ") + payload, kind)
            if kind == "rx" and payload.startswith("STATE "):
                try:
                    angles = [int(value) for value in payload[6:].split(",")]
                    if len(angles) == SERVO_COUNT and all(
                        ANGLE_MIN <= value <= ANGLE_MAXES[channel]
                        for channel, value in enumerate(angles)
                    ):
                        self._apply_state(angles)
                except ValueError:
                    self._append_log("状态数据格式无效", "error")
            elif kind == "error":
                self._handle_link_error(payload)
        self.after(60, self._poll_events)

    def _append_log(self, text: str, tag: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"{stamp}  {text}\n", tag)
        self.log.see(tk.END)

    def _handle_link_error(self, message: str) -> None:
        self.link.disconnect()
        self._set_connected(False)
        self._append_log(message, "error")

    def _close(self) -> None:
        self.link.disconnect()
        self.destroy()


if __name__ == "__main__":
    HandDebugger().mainloop()
