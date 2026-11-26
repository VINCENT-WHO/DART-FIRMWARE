# DART绿色发光灯识别

独立的电脑端C++17/OpenCV工程，使用CMake构建。不会访问串口、控制舵机或更新STM32固件。

完整中文说明：[绿灯识别程序.md](../docs/绿灯识别程序.md)。机载识别与电脑端的对应关系见 [识别程序说明.md](../docs/识别程序说明.md)。

在VSCode单独打开本目录，运行任务“识别REC00032并保存到新目录”即可编译并处理原视频。结果保存在 `C:\RMDART\数据记录` 下新建的带时间目录，原视频不变。

源码入口为 `src/main.cpp`，算法为 `src/detector.cpp`，阈值为 `config/detector.yml`。两项CMake预设命令为：

```powershell
cmake --preset msys2-local
cmake --build --preset msys2-local
```

处理 `REC00038` 这类灯芯已发白、附近有绿色反射的录像时，可改用 `--config=config/rec00038_white_core.yml`。该配置只将亮度 V≥252、饱和度 S≤35 的像素纳入灯芯掩膜，最小轮廓面积是280像素²，外围仍须有绿色。它适合这次的距离和曝光；换成更暗、未发白或更小的灯时，应先用原配置确认检出情况。默认配置的识别行为不变。

本机预设依赖 `D:\CppDev\msys64\ucrt64` 的GCC和OpenCV。更换电脑需调整工具链路径，详见完整说明。
