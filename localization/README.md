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
  -> localization_result.json供地图和任务程序读取
  -> 默认不向F407连续回传TYPE=0x16
```

场地中心为`(0,0)`、边界为±1.500 m，四个300×300 mm出发区的车体中心固定为
`(-1.350,+1.350)`、`(+1.350,+1.350)`、`(-1.350,-1.350)`、`(+1.350,-1.350) m`。
配置中的`start_center_m`必须保持`1.350`。

减速带的单条尺寸为 300×60×10 mm，三条间隔 50 mm。默认在起步后前 0.70 m 以及四个场地角落排除区中禁用编码器，仅使用 T265；进入平地后才融合编码器。

## 返航阶段编码器补偿

`NAVIGATE_WAYPOINT`和`RETURN_CENTER`有效时进入返航融合模式；SEARCH/APPROACH仍保持原融合权重。
返航中三轮编码器以100 Hz预测平移，
T265航向继续逐帧约束，但T265位置只按20 Hz、增大后的测量协方差进行缓慢校正。mapper
confidence为0时进一步降低T265位置权重，避免`SLAM_ERROR Speed`期间T265少算路程后把轮式进度
反复拉回。

距离目标大于300 mm、mapper为0且仅速度残差超限时，允许物理速度仍在上限内的编码器预测继续
参与，门控显示`navigation_encoder_override`。距离目标不超过300 mm时，如果轮速至少0.10 m/s
而T265速度不超过0.05 m/s，则视为围栏接触/空转，冻结该轮式增量并显示
`navigation_near_target_slip`，防止编码器把地图推过围栏。

`localization_result.json`新增`navigation`段，记录当前命令、剩余距离、航向、本段轮式累计进度、
T265位置是否在本帧参与校正、位置权重倍率和创新距离。输出`quality`现在同时参考tracker和
mapper；`tracker=3/mapper=0`显示`DEGRADED`而不是虚假的高精度`GOOD`，但仍可供任务规划使用。

`camera_offset_forward_m/left_m`表示T265 tracking origin相对车体旋转中心的前向/左向距离，
单位m，必须按实车测量填写，不能混用105 mm推板距离或130 mm轮子运动学半径。它们保持0时，T265不在车体旋转中心造成的
原地转向圆弧仍会被误认为车体平移；程序不会根据单次日志猜测并写入机构尺寸。

## 为什么不直接发“三个轮子”给 T265

T265 wheel-odometry API 的输入是 velocimeter 三维平移速度，配置最多两个 velocimeter。三轮全向轮的单轮转速不是同一坐标系中的车体速度，所以先使用 F407 已验证的运动学解算：

```text
forward = (M3 - M1) / sqrt(3)
left    = (M1 + M3 - 2*M2) / 3
rotate_tangent = (M1 + M2 + M3) / 3
```

`wheel_center_radius_m`是车体旋转中心到全向轮滚动作用线的垂直距离，当前实测初值使用
`0.130 m`，不是推板/前拨板距离。必须通过架空原地旋转360°的三轮编码器累计量复核。

## 镜头朝上的三维姿态投影

配置项`camera_robot_forward_axis`和`camera_robot_up_axis`表示“机器人轴在T265 Pose本体坐标中的
坐标”。当前实际安装为机器人车头`-T265 X`、右侧`+T265 Y`、上方`+T265 Z`，配置为：

```ini
camera_robot_forward_axis = -x
camera_robot_up_axis = +z
```

程序从完整`translation.xyz / velocity.xyz / quaternion.xyzw / angular_velocity.xyz`构造相机到
世界旋转。车头世界向量按`f_W=R_WC*f_C`计算，底盘原始航向为
`atan2(-f_W.x,-f_W.z)`；不再使用俯仰接近90°时有奇异性的绕Y欧拉角。

第一帧保存三维位置、车头/左向世界向量和原始航向。后续位移分别点乘初始车头和左向量，
相对航向通过连续四元数航向差累计，再叠加所选出发区135°/45°/225°/315°锚点。yaw rate
使用连续帧航向差分低通滤波，不假定`angular_velocity.y`就是底盘偏航速度。

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

任务测试时，定位程序仍是`/dev/ttyS1`唯一所有者。串口接收、任务命令转发和T265取帧现在是
相互独立的执行线程：T265短暂停帧不会再阻断任务命令。`TYPE=0x11/0x12`只在命令文件更新时
转发；`TYPE=0x18`任务命令由串口线程以100 Hz重新生成SEQ和CRC后持续发送。视觉命令文件停止
更新250 ms后停止代发，让F407自身看门狗安全停车，避免规划线程或位姿失效后无限维持旧命令。

`--stm-status`生成的JSON增加`relay.tx_frames/tx_errors/last_sequence/last_tx_age_ms`，可直接确认
真实串口发送是否持续；这些值不是视觉窗口的去重日志。

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
2. 按实装填写机器人前轴和上轴在T265本体坐标中的方向，禁止用固定±90°二维补偿猜测镜头朝上的姿态。
3. 架空车轮，分别转动 M1/M2/M3，确认原始计数和 `encoder_sign` 一致。
4. 平地前进 1 m、横移 1 m，核对轮径和 1768 counts/rev；不要用减速带路段标定轮径。
5. 原地旋转 360°，测量 `wheel_center_radius_m`；未标定前保持 0。
6. 从四个出发区分别越过/绕过减速带，确认 JSON 中 `wheel.gate` 显示 `startup_obstacle` 或 `corner_obstacle`。
7. 在平地复测闭合路线，用 CSV 比较 T265、轮式预测和融合输出，再调整协方差和速度残差门限。
