# 11 自由度仿生机械手

这是一个基于 STM32F103、PCA9685 和 MediaPipe 的 11 自由度右手机械手项目。项目包含机械结构文件、嵌入式控制固件、串口调试工具，以及普通摄像头手势控制和 RGB-D 连续动作跟随程序。

![机械手实物展示](<3d/右手图纸/展示图/InShot_20190627_113636994.jpg>)

## 功能

- 通过 PCA9685 驱动 11 路舵机
- 通过 USB 转串口独立调试每个关节
- 使用 MediaPipe 识别 8 种常用手势并映射为机械手姿态
- 使用 Orbbec Gemini 336 深度相机估算真实 3D 关节角
- 对识别结果进行稳定帧判断、滤波和舵机速度限制
- 保存和载入 JSON 格式的机械手姿态
- 提供 STL、SolidWorks 源文件、结构图和组装手册

## 项目结构

```text
robot_hand/
├─ 3d/右手图纸/                 # STL、SolidWorks、结构图和组装手册
├─ code/hand/                   # STM32F103 固件
│  ├─ Core/                     # 应用代码与外设初始化
│  ├─ Drivers/                  # STM32 HAL 与 CMSIS
│  └─ tools/hand_debugger/      # Python 串口调试台
└─ code/py/
   ├─ shoushi.py                # 普通摄像头手势识别控制
   ├─ main.py                   # RGB-D 关节角连续跟随
   ├─ gesture_recognizer.task   # MediaPipe 手势识别模型
   └─ hand_landmarker.task      # MediaPipe 手部关键点模型
```

## 硬件

- STM32F103 控制板
- PCA9685 16 路 PWM 驱动板，默认 I2C 地址 `0x40`
- 11 个舵机及机械手结构件
- 3.3 V TTL USB 转串口模块
- 独立舵机电源
- 普通 USB 摄像头，或 Orbbec Gemini 336 RGB-D 相机

> 舵机电源不要直接取自 STM32。舵机电源、STM32 和 PCA9685 必须共地，首次调试建议先卸载拉线或断开舵机负载，确认角度方向和限位正确后再组装。

### USB 转串口接线

| STM32F103 | USB 转串口 |
| --- | --- |
| PA9 / USART1_TX | RX |
| PA10 / USART1_RX | TX |
| GND | GND |

串口参数固定为 `115200, 8N1`。

## 固件编译与烧录

固件工程位于 `code/hand`，由 STM32CubeMX 生成基础工程，并使用 CMake 构建。需要提前安装 ARM GNU Toolchain、CMake 和 Ninja。

```powershell
cd code\hand
cmake --preset Debug
cmake --build --preset Debug
```

生成的固件位于 `code/hand/build/Debug/hand.elf`。可使用 STM32CubeProgrammer、ST-Link 或支持 ELF 文件的调试器烧录。修改 CubeMX 配置时请使用 `code/hand/hand.ioc`。

## 串口调试台

Windows 下可直接双击：

```text
code\hand\tools\hand_debugger\run.cmd
```

也可以在 PowerShell 中运行：

```powershell
cd code\hand
.\tools\hand_debugger\run.ps1
```

脚本首次启动会在工具目录创建虚拟环境并安装 `pyserial`。连接串口后，可以分别调整 11 路关节角、实时下发命令、预览姿态，以及保存或载入姿态文件。

通道定义如下：

| 通道 | 关节 | 角度范围 |
| --- | --- | --- |
| CH0 | 拇指旋转 | 5-140° |
| CH1 | 拇指近端 | 5-170° |
| CH2 | 拇指远端 | 5-140° |
| CH3-CH4 | 食指近端、远端 | 5-170° |
| CH5-CH6 | 中指近端、远端 | 5-170° |
| CH7-CH8 | 无名指近端、远端 | 5-170° |
| CH9-CH10 | 小指近端、远端 | 5-170° |

## 视觉控制

建议使用 Python 3.10 创建独立环境：

```powershell
cd code\py
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

RGB-D 模式还需要按 [Orbbec SDK 文档](https://github.com/orbbec/pyorbbecsdk) 安装与设备、系统匹配的 `pyorbbecsdk`。

### 普通摄像头手势控制

`shoushi.py` 使用 MediaPipe 内置手势分类器，将 `Closed_Fist`、`Open_Palm`、`Victory` 等手势映射为预设的 11 路舵机角度。

```powershell
python shoushi.py --port COM20 --camera 0
```

使用其他串口或摄像头时修改对应参数。运行中按空格键暂停或恢复控制，按 `Q` 退出。执行下面的命令可以进入姿态标定模式：

```powershell
python shoushi.py --calibrate --camera 0
```

### RGB-D 连续动作跟随

`main.py` 面向 Orbbec Gemini 336。程序使用彩色图像检测 21 个手部关键点，再利用对齐的深度图反投影到相机坐标系，经过骨长约束和平滑处理后映射到舵机角度。

```powershell
python main.py --port COM20 --debug
```

关节范围标定：

```powershell
python main.py --calibrate
```

标定窗口中，右手完全伸直时按 `1`，完全握拳时按 `2`，各采集多组数据后按 `3` 输出新的 `MP_RANGES`。

> 当前仓库缺少 `code/py/rgbd_hand_3d.py` 源文件，`main.py` 在补回该模块前无法从全新环境运行。该模块应提供 Gemini 336 采集、深度反投影、骨长约束和关键点平滑功能；请不要用 `__pycache__` 中的 `.pyc` 文件代替源码上传。

## 串口协议

每条 ASCII 命令以回车或换行结束：

```text
PING          -> PONG
GET           -> STATE 5,5,5,5,5,5,5,5,5,5,5
SET 0 90      -> OK
ALL 5         -> OK
```

- `SET <通道> <角度>` 设置单路舵机。
- `ALL <角度>` 设置全部舵机；CH0 和 CH2 最高会限制在 140°。
- 参数或通信异常时，控制板返回以 `ERR` 开头的错误信息。


