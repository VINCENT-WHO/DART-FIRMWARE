# DART-FIRMWARE

制导飞镖镖体固件：F103 负责读取 IMU，H750 负责相机、机载绿灯识别和 TF 卡记录。上电后待机，检测到发射加速度后自动记录约 10 秒，落地后用 USB 读卡器取出文件。

当前只采集数据，不做姿态控制。

## 功能

| 芯片 | 版本 | 职责 |
| --- | --- | --- |
| STM32F103C8T6 | 2.0.0 | 读取 BMI088（±24g 量程），把原始数据发给 H750；转发电脑串口命令；四路舵机 PWM |
| STM32H750VBT6 | 2.2.2-rgb-roi3ms | OV5640 采集 320×240 图像；逐帧识别绿灯；硬件 JPEG 编码；把结果写入 TF 卡 |

一次记录的流程：

1. 上电后相机和 TF 卡完成初始化，进入待机。
2. 加速度合量超过 2g 并持续 20ms 时触发（每次上电只触发一次），同时保留触发前的 200 个 IMU 样本。
3. 记录 10 秒：IMU 数据、逐帧识别结果、带标注的录像。
4. 写出文件，之后可以把数据线翻到 H750 方向，电脑会把 TF 卡识别为 U 盘。

TF 卡上每次记录是一个 `REC#####` 目录：

| 文件 | 内容 |
| --- | --- |
| `VIDEO.AVI` | 320×240 MJPEG 录像，检出时画黄圈并标出中心坐标 |
| `DETECT.CSV` | 每帧的中心坐标（像素）和检出标志 |
| `IMU.CSV`、`IMU.DLG` | IMU 数据和原始记录 |
| `TIMING.CSV` | 每帧检测、标注、编码、保存耗时，含识别内部 7 个步骤及搜索框/回退模式 |
| `INFO.TXT` | 本次记录的汇总信息 |

## 目录结构

```
f103/                   F103 固件源码和链接脚本
h750/                   H750 固件源码和链接脚本
  green_detector.c/.h     机载绿灯识别
  green_annotate.c        在图像上画圈和坐标
  camera.c, camera_slots.h  相机采集与帧槽管理
  jpeg_hw.c, jpeg_mcu.c   硬件 JPEG 后端
  jpeg_enc.c              软件 JPEG 参考实现（固件不链接）
  record_profile.c        逐帧耗时记录
common/                 两块芯片共用的协议、自动触发、AVI 头
green_light_detector/   电脑端识别工程（C++17 + OpenCV）
vendor/                 第三方库：CMSIS、STM32 HAL、FatFs、USB、OV5640 驱动
cmake/                  交叉编译工具链和清单脚本
tests/                  主机端测试和验证脚本
tools/                  烧录、串口、解码、舵机标定工具
docs/                   说明文档
```

## 编译

需要 CMake 3.21 以上、Ninja 和 Arm GNU Toolchain。

```powershell
$env:ARM_GCC_BIN = '<arm-none-eabi-gcc 所在目录>'
cmake --preset dart
cmake --build --preset dart
```

产物在 `build/native/`：

| 芯片 | 文件 | 大小 | Flash 上限 |
| --- | --- | ---: | ---: |
| F103 | `f103/dart_f103.bin`、`.hex`、`.elf` | 12224 字节 | 64KB |
| H750 | `h750/dart_h750.bin`、`.hex`、`.elf` | 104464 字节 | 128KB |

同目录下的 `manifest.json` 记录版本号、大小和校验值。H750 的 Flash 余量约 26KB，加功能时要留意。

编译时会出现几条 `_close is not implemented` 之类的链接警告，来自 newlib 的系统调用桩，可以忽略。

`tools/build.py` 是早期直接调用 gcc 的脚本，还没有加入识别和 JPEG 模块，H750 请用上面的 CMake 命令编译。

## 烧录

两个烧录脚本都会先把芯片原有的全部内容备份到仓库旁边的 `固件备份/` 目录，烧录后再回读核对。备份目录在仓库外，不会进入版本管理。

### F103

数据线接到 **F103/DartDAP 方向**。需要 OpenOCD：

```powershell
python tools/flash_f103.py --openocd <openocd可执行文件> --scripts <openocd的scripts目录> --program
```

### H750

数据线翻到 **H750 方向**，然后让板子进入 DFU：按住 BOOT 不松，短按一下 RST，再松开 BOOT。需要 dfu-util：

```powershell
python tools/flash_h750_dfutil.py --dfu-util <dfu-util可执行文件> --build-dir build/native --program
```

固件必须整版烧录，没有增量升级。

## 串口与调试

电脑与 F103 之间是 115200 波特的串口，经板载 DartDAP 转接。安装依赖后可以用串口工具发命令：

```powershell
python -m pip install -r requirements.txt
python tools/console.py --port <串口号> --command <命令> --seconds 20
```

Ozone 调试工程：

| 文件 | 用途 |
| --- | --- |
| `f103_dartdap_ozone.jdebug` | 经板载 DartDAP 调试 F103，需要先启动 OpenOCD 服务（`debug/f103_dartdap_server.cfg`，端口 3333） |
| `f103_external_jlink.jdebug` | 用外接 J-Link 调试 F103 |
| `h750_external_jlink.jdebug` | 用外接 J-Link 调试 H750 |

## 绿灯识别

当前机载采用固定约3 ms曝光和RGB绿色区域识别；电脑端保留历史HSV方案：

- **机载**（`h750/green_detector.c`）：G通道二值化、去白、中值、绿色像素面积筛选；检出后在上帧外接矩形的1.5倍范围搜索，每侧至少扩展10像素、越界裁边，丢失/内部触边时回退全图。录像画外接矩形，结果写进 `DETECT.CSV`。
- **电脑端**（`green_light_detector/`）：历史双掩膜离线程序，与当前机载算法不再等价。

入口、阈值对应关系和测试命令见 [docs/识别程序说明.md](docs/识别程序说明.md)，电脑端的使用说明见 [docs/绿灯识别程序.md](docs/绿灯识别程序.md)。

## 测试

测试都在电脑上运行，不连接板子。需要本机的 gcc、Python，部分脚本还需要 OpenCV 和 Pillow 的 Python 库。

```powershell
python tests/run_tests.py
```

这会验证协议编解码、AVI 头、自动触发、耗时记录容量、RGB识别合成输入和固定曝光寄存器逻辑。识别和 JPEG 的验证脚本列在 [docs/识别程序说明.md](docs/识别程序说明.md) 里。

这些测试只说明算法在给定输入下的行为。实际检出率和帧耗时以飞行记录里的 `DETECT.CSV` 和 `TIMING.CSV` 为准。

## 其他工具

| 文件 | 用途 |
| --- | --- |
| `tools/decode_log.py` | 把 `IMU.DLG` 导出为 CSV |
| `tools/validate_capture.py` | 检查一次实际记录的 DLG 和 AVI |
| `tools/servo_calibrate.ps1` | 舵偏角标定的 PWM 输出工具 |
| `tools/servo_neutral.ps1` | 四路舵机回中（1500 微秒），用于安装零位 |
