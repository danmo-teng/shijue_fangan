# T265 + F407 三轮全向定位

该目录是独立定位工程，不修改现有 T265/F207 联调工程和单目视觉代码。程序固定链接本机已编译的官方 librealsense 2.50.0。

## 融合结构

```text
F407 三路累计编码器 (100 Hz)
  -> 15字节 TYPE=0x15 + Modbus CRC
  -> 三轮正运动学: 车体前向/左向/可选旋转增量
  -> 减速带区域、速度和 T265 速度残差门控
  -> EKF 预测

T265 6DoF pose (200 Hz)
  -> T265原生坐标 -> 车体中心 -> 3m×3m场地坐标
  -> 按 tracker confidence 选择测量协方差
  -> EKF 校正（T265为主定位）
  -> localization_result.json
  -> localization_result.json供地图和任务程序读取
  -> 默认不向F407连续回传TYPE=0x16
```

减速带的单条尺寸为 300×60×10 mm，三条间隔 50 mm。默认在起步后前 0.70 m 以及四个场地角落排除区中禁用编码器，仅使用 T265；进入平地后才融合编码器。

## 为什么不直接发“三个轮子”给 T265

T265 wheel-odometry API 的输入是 velocimeter 三维平移速度，配置最多两个 velocimeter。三轮全向轮的单轮转速不是同一坐标系中的车体速度，所以先使用 F407 已验证的运动学解算：

```text
forward = (M3 - M1) / sqrt(3)
left    = (M1 + M3 - 2*M2) / 3
rotate_tangent = (M1 + M2 + M3) / 3
```

`wheel_center_radius_m=0` 时不使用轮子航向，航向完全由 T265 约束。实测轮子接地点到车体旋转中心的距离后，才可填写非零值并校准旋转符号。

## UART 兼容性

GitHub 中的 F407 仓库当前定义了 115200 8N1、`A3 B3 ... C3` 固定 15 字节帧和 Modbus CRC，但仓库中 F407 TX 只有 4 字节配置 ACK，还没有编码器上报类型。本工程在不改外层协议的前提下分配 `TYPE=0x15`：

```text
A3 B3 15 SEQ M1_H M1_L M2_H M2_L M3_H M3_L DT STATUS CRC_LO CRC_HI C3
```

详见 [docs/uart_protocol.md](docs/uart_protocol.md)。F407 参考打包代码在 `firmware/f407_odom_protocol.[ch]`。如果实车下位机的 `P0..P7` 已使用另一种定义，必须先同步字段表，不能只凭帧头相同就开始融合。

## 编译与测试

```bash
cd /home/sunrise/文档/ChatGPT/T265/t265_omni_localization
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

## 运行

只验证 T265 和场地坐标：

```bash
./run_localization.sh --duration 10 --rate 10
```

本机是 RDK X5 V1.0，40Pin 默认 UART1 对应 `/dev/ttyS1`，使用 3.3 V IO：

- RDK 物理 8 脚 UART1 TX -> F407 PD9 / USART3 RX；
- RDK 物理 10 脚 UART1 RX <- F407 PD8 / USART3 TX；
- RDK GND 与 F407 GND 共地。

```bash
./run_localization.sh \
  --uart /dev/ttyS1 \
  --baud 115200 \
  --rate 20 \
  --tx-rate 0 \
  --command-file ../rescue_map/runtime/uart_command.bin \
  --stm-status ../rescue_map/runtime/stm32_status.json \
  --csv localization.csv
```

`--tx-rate 0`是默认设置：T265/编码器融合和地图显示继续运行，但不向F407发送实时位置。仅在旧协议调试时才显式设置非零值。

任务测试时，定位程序仍是 `/dev/ttyS1` 唯一所有者。`--command-file` 只转发新出现且通过长度、TYPE和CRC校验的 `0x11/0x12/0x18` 帧；`--stm-status` 把F407的 `0x17` 状态帧原子写成JSON，供 `mission_test` 读取。不要再让视觉Python进程直接打开同一串口。

`run_localization.sh` 在 T265 为 `03e7:2150` 时会先调用长超时引导器，进入 `8087:0b37` 后再启动定位。

如果终端持续显示 `uart=stale` 且退出时 `UART frames=0`，表示 RDK 没有收到任何合法 `TYPE=0x15` 帧。依次检查 F407 是否上电、TX/RX 是否交叉、是否共地、两端是否均为 115200 8N1，以及 F407 是否真正每 10 ms 调用发送队列。

## 旧版可选：F407实时位姿回传

RDK 回传帧使用同样的 15 字节外层，消息类型为 `TYPE=0x16`：

```text
A3 B3 16 SEQ X_H X_L Y_H Y_L YAW_H YAW_L STATUS CONF_SIG CRC_LO CRC_HI C3
```

- `X/Y`：融合后场地坐标，有符号 mm；
- `YAW`：`0..35999`，单位 0.01°；
- `STATUS`：包含位姿有效、T265良好、编码器正在融合、减速带门控、里程计新鲜、场地内以及 T265 跃变拒绝等位；
- `CONF_SIG`：T265 tracker/mapper confidence 和融合位置标准差。

F407 解码代码在 [f407_fused_pose.h](firmware/f407_fused_pose.h) 和 [f407_fused_pose.c](firmware/f407_fused_pose.c)。现有 `Vision_ParseBytes()` 应增加 `0x16` 合法类型，共用原有帧头、CRC 和重同步状态机；CRC 通过后调用：

```c
F407FusedPose fused_pose;

/* payload = P0..P7, sequence = frame[3] */
F407_FusedPoseDecodePayload(payload, sequence, HAL_GetTick(), &fused_pose);
```

下位机使用前必须同时判断：

```c
if (F407_FusedPoseIsFresh(&fused_pose, HAL_GetTick(), 150U) &&
    ((fused_pose.status & F407_POSE_T265_GOOD) != 0U)) {
  /* 允许使用 x_mm / y_mm / heading_cdeg 执行位置闭环 */
} else {
  Motor_Stop();
}
```

相同 `SEQ` 的重复帧不应刷新看门狗；超过 150 ms、`VALID=0`、CRC 错误或 T265 `LOST` 时不能继续沿用旧坐标执行动作。完整位定见 [UART 协议](docs/uart_protocol.md)。

## 输出与视觉接入

`localization_result.json` 使用临时文件替换，导航进程不会读到半帧 JSON。坐标系与 F407 `Location.c` 一致：场地中心为原点，`+X` 向图纸右，`+Y` 向上，航向从 `+X` 逆时针增加。

单目视觉的 `runtime_result.json` 是车体相对地面坐标（X向右、Y向前，mm）。下列程序把目标转换为场地绝对坐标：

```bash
python3 tools/merge_vision_pose.py \
  --vision /home/sunrise/RDK_X5/traditional_rescue_vision/runtime_result.json
```

输出 `navigation_world.json`，任一上游文件超过 150 ms、T265 `LOST` 或单目未标定时，`valid=false`，导航应停车而不是沿用旧位置。

## 实车标定顺序

1. 填写 T265 tracking origin 相对车体旋转中心的 `camera_offset_forward_m/left_m`。
2. 校正T265固定安装角：正值把T265平面位移逆时针旋转，负值顺时针旋转。本车实测地图方向比实际方向逆时针90°，因此`camera_to_robot_yaw_deg=-90.0`。
3. 架空车轮，分别转动 M1/M2/M3，确认原始计数和 `encoder_sign` 一致。
4. 平地前进 1 m、横移 1 m，核对轮径和 1768 counts/rev；不要用减速带路段标定轮径。
5. 原地旋转 360°，测量 `wheel_center_radius_m`；未标定前保持 0。
6. 从四个出发区分别越过/绕过减速带，确认 JSON 中 `wheel.gate` 显示 `startup_obstacle` 或 `corner_obstacle`。
7. 在平地复测闭合路线，用 CSV 比较 T265、轮式预测和融合输出，再调整协方差和速度残差门限。
