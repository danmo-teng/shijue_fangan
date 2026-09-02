# T265 独立调试工具

这个目录只用于 T265 单机排查，不依赖 UART/F207，也不修改上级目录中的联合调试程序。程序固定链接已编译的官方 librealsense **2.50.0**。

## 编译

```bash
cd /home/sunrise/文档/ChatGPT/T265/t265_standalone_debug
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

检查实际加载的库：

```bash
ldd build/t265_debug | grep librealsense
```

## 建议调试顺序

日常可直接用统一入口；它在发现 `03e7:2150` 时先自动引导，已是 `8087:0b37` 时则直接读取位姿：

```bash
./run_t265.sh --duration 10 --print-rate 10 --csv t265_pose.csv
```

1. 先检查 USB 枚举和 SDK 识别：

   ```bash
   ./build/t265_debug --list --debug-sdk
   ```

2. 如果日志显示官方 2.50 的 1000 ms 固件传输超时，用长超时引导工具下发同一个官方固件：

   ```bash
   ./build/t265_boot
   ```

   工具只处理 `03e7:2150 -> 8087:0b37` 的一次性内存固件加载，不刷写 T265 的持久存储。默认超时为 15 秒。

   如果之前的传输超时导致引导端点卡住，可只重置这个 USB 设备后重试：

   ```bash
   ./build/t265_boot --reset-usb
   ```

   如果连 1 KiB 首块都显示 `transferred=0`，说明 Movidius 引导端点本身已无响应。请拔掉 **T265 本身** 5 秒后重插，重插后不要先运行其他 RealSense 程序，直接执行：

   ```bash
   ./build/t265_boot --chunk-kib 256
   ./build/t265_debug --duration 10 --print-rate 10
   ```

   不建议重置 T265 所在的整个 USB Hub：当 Hub 使用组合供电（ganged power）时，会同时断开其他 USB 外设。

3. 连续读取 10 秒位姿，终端每秒打印 10 次：

   ```bash
   ./build/t265_debug --duration 10 --print-rate 10
   ```

4. 保存每一帧原始位姿：

   ```bash
   ./build/t265_debug --duration 30 --csv t265_pose.csv
   ```

## 二维轨迹和朝向调试

`t265_trajectory_debug`在RDK本地显示T265的二维本地轨迹、实时车头方向和累计路程；不需要连接F407或视觉程序。T265第一帧tracker confidence达到默认阈值2的位姿会自动作为原点，平面坐标定义为`向右=原生X`、`向前=-原生Z`，单位米。低于阈值的帧会显示为橙色且不计入轨迹或距离，避免LOST时的跳变污染标定数据。

```bash
export DISPLAY=:0
./run_trajectory.sh --min-confidence 2 --scale 150 --csv t265_trajectory.csv
```

按`R`在当前下一帧可靠位姿处重置原点、轨迹和累计距离；`F`切换全屏；`+`/`-`缩放，`Q`或`Esc`退出。屏幕左上角的`path`是相邻可靠T265位姿的平面弧长累加，适合与地面尺量距离比较；它不是F407编码器距离。若起点不是希望的位置，到位后按`R`再开始测试。

## USB 状态含义

- `03e7:2150 BOOTLOADER`：T265 的 Movidius 引导态。SDK 应将内置的 `0.2.0.951` 固件下发到相机，然后相机重新枚举。
- `8087:0b37 RUNNING`：T265 已进入正常工作态，librealsense 才能列出序列号并开始 pose 流。
- 两者都没有：优先检查 USB 线、供电、接口和内核枚举日志。

如果一直停留在 `03e7:2150`，执行 `--list --debug-sdk` 后查看是 `Failed to open T265 zero interface` 还是 `Error booting T265`。前者多为 USB 访问权限或接口被占用；后者是固件 bulk 传输失败。本机已观察到 2.50 的 1000 ms 超时，可先用 `t265_boot`；如果 15 秒仍失败，再优先更换短的数据线或主机 USB 口，避免无源 Hub。

## 输出约定

T265 原生坐标系为 `+X 向右、+Y 向上、+Z 向后`，位置单位为米。`conf=tracker/mapper`，范围 0–3，3 最高。这里不做底盘坐标系转换，便于先验证 T265 本体。
