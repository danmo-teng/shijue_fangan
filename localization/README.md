# T265 + F407 三轮全向定位

该目录是独立定位工程，不修改现有 T265/F207 联调工程和单目视觉代码。程序固定链接本机已编译的官方 librealsense 2.50.0。

## 融合结构

```text
F407 三路累计编码器 (100 Hz)
  -> 15字节 TYPE=0x15 + Modbus CRC
  -> 三轮正运动学: 车体前向/左向/轮式旋转诊断增量
  -> T265 Pose角速度提供高频机器人偏航增量
  -> 减速带区域、速度和 T265 速度残差门控
  -> EKF 预测

T265 6DoF pose (200 Hz)
  -> T265原生坐标 -> 车体中心 -> 3m×3m场地坐标
  -> 用T265 Pose角速度投影得到机器人垂直轴陀螺航向增量
  -> 按 tracker confidence 选择测量协方差
  -> EKF 校正（T265为主定位）
  -> localization_result.json供地图和任务程序读取
  -> 可选全速率 localization_debug.csv，保存T265与轮式里程计对照数据
  -> 默认不向F407连续回传TYPE=0x16
```

场地中心为`(0,0)`、边界为±1.500 m，四个300×300 mm出发区的车体中心固定为
`(-1.350,+1.350)`、`(+1.350,+1.350)`、`(-1.350,-1.350)`、`(+1.350,-1.350) m`。
配置中的`start_center_m`必须保持`1.350`。

减速带的单条尺寸为 300×60×10 mm，三条间隔 50 mm。默认在起步后前 0.70 m 以及四个场地角落排除区中禁用编码器，仅使用 T265；进入平地后才融合编码器。

## 返航阶段编码器补偿

`NAVIGATE_WAYPOINT`和`RETURN_CENTER`有效、编码器已启用且最近存在新鲜有效轮式增量时进入
返航融合模式；SEARCH/APPROACH仍保持普通T265主导融合。仅T265模式全程逐帧1:1跟随T265
投影位姿，不因NAV/RETURN降低权重或触发跳变冻结。

融合返航中三轮编码器以100 Hz预测平移，T265航向继续逐帧约束。编码器权重为100%时保留
20 Hz弱位置校正；权重低于100%时T265位置逐帧校正，并按编码器权重平方缩小原NAV位置sigma
倍率，避免低编码器权重仍被旧的`×8/×12`配置压制。

距离目标大于300 mm、mapper为0且仅速度残差超限时，允许物理速度仍在上限内的编码器预测继续
参与，门控显示`navigation_encoder_override`。距离目标不超过300 mm时，如果轮速至少0.10 m/s
而T265速度不超过0.05 m/s，则视为围栏接触/空转，冻结该轮式增量并显示
`navigation_near_target_slip`，防止编码器把地图推过围栏。

`localization_result.json`新增`navigation`段，记录当前命令、剩余距离、航向、本段沿目标方向的
编码器距离补偿、T265位置是否在本帧参与校正、位置权重倍率和创新距离。输出`quality`现在同时参考tracker和
mapper；`tracker=3/mapper=0`显示`DEGRADED`而不是虚假的高精度`GOOD`，但仍可供任务规划使用。

`camera_offset_forward_m/left_m`表示T265双目成像器中心 tracking origin 相对三轮运动学旋转中心的
前向/左向距离，单位m；当前实测为后方29.6 mm、右侧30.1 mm，因此配置为
`camera_offset_forward_m=-0.0296`、`camera_offset_left_m=-0.0301`。正方向是车头forward和车体left，
不能混用T265外壳中心、105 mm推板距离、150 mm前拨板距离或130 mm轮子运动学半径。位置和速度
lever-arm只补偿一次，轮式中心里程计不再叠加该偏置。

原地逆时针90°时，tracking origin的理论假位移约为(+59.7,+0.5) mm，修正后的robot-center
位移应接近0；CSV和JSON会同时记录raw tracking-origin位置、修正后的机器人中心位移、偏置参数，
以及T265 robot-center与wheel odometry的逐帧位置差。JSON还分别记录原始编码器增量和加权后的融合增量。

## 定位调试日志

使用 `--csv FILE` 可保存全速率 CSV。日志每行对应一个 T265 Pose 帧，同时带有最近的 F407
编码器帧、三轮运动学解算增量、T265 陀螺航向速率/累计航向、使用时间同步航向的轮式里程计
累计位姿、T265 场地投影位姿和 EKF 融合位姿。T265 陀螺航向使用 T265 自己的时间戳积分，
并按 `t265_gyro_pose_resync_period_s` 定期用姿态 yaw 重同步；每个 F407 ODOM 增量按其
主机接收时间从 T265 航向时间线插值得到中点场地航向。这样可以直接对比 `t265_*_m`、`odom_*_m` 与
`fused_*_m`，分析方向、里程和定位漂移；日志还记录 `wheel_gate`、T265 置信度、创新距离
及导航阶段。原始编码器导航距离补偿由独立的
`navigation_distance_compensation_enabled` 控制，默认关闭；它不受
`encoder_fusion_weight` 的数值暗中启用或关闭。

`odom_increment_yaw_deg` 是实际用于轮式里程计/EKF预测的时间同步 T265 航向增量，
`wheel_kinematic_yaw_deg` 是三轮公式原本给出的角度，`gyro_yaw_delta_deg` 是同一增量的
同步航向变化，`t265_yaw_at_increment_deg` 是用于平移旋转的增量中点场地航向。
`increment_field_yaw_valid=1` 表示该增量确实使用了 T265 时间线；`gyro_pose_sync_error_deg`
记录最近一次重同步以来姿态 yaw 与陀螺累计 yaw 的差异。该组合是“按 T265 时间戳积分的
陀螺高频增量 + 周期 T265 姿态重同步”；日志中的
`fused_vs_wheel_odom_*` 只是两种传感器的一致性差异，不是带真值的绝对误差。

例如：

```bash
./run_localization.sh --csv /tmp/localization_debug.csv --duration 10 --rate 10
```

`rescue_map/run_rescue_map.sh` 会自动把日志写入
`rescue_map/runtime/localization_debug.csv`，地图同时显示融合轨迹和轮式里程计轨迹，并显示两者的
位置/航向差异（这是传感器间的一致性指标，不等同于带真值的绝对误差）。

## 为什么不直接发“三个轮子”给 T265

T265 wheel-odometry API 的输入是 velocimeter 三维平移速度，配置最多两个 velocimeter。三轮全向轮的单轮转速不是同一坐标系中的车体速度，所以先使用 F407 已验证的运动学解算：

```text
forward = (M1 - M2) / sqrt(3)
left    = (M1 + M2 - 2*M3) / 3
rotate_tangent = (M1 + M2 + M3) / 3
```

`wheel_center_radius_m`是车体旋转中心到全向轮滚动作用线的垂直距离，当前实测初值使用
`0.130 m`，不是推板/前拨板距离。必须通过架空原地旋转360°的三轮编码器累计量复核。

当前上位机按F407 `Location.c`的实际轮序解算：M1为右轮、M2为左轮、M3为后轮，因此默认
`encoder_to_robot_yaw_deg=0`。该参数仅保留给实测安装角微调，不能再用固定90°旋转掩盖轮序
错误。轮式旋转增量保留到日志作为对照，实际轮式里程计和EKF预测的航向增量由T265陀螺仪
按T265自身时间戳积分并定期由T265四元数姿态重同步，不能用最新速率乘F407 ODOM间隔。

`encoder_fusion_weight`范围0～1，默认0.25：0表示编码器平移不进入EKF，1表示完整使用编码器
平移。原始紫色轮式里程计不缩放，便于和T265对照。地图启动页可按5%或10%调节该值。

## 镜头朝上的三维姿态投影

配置项`camera_robot_forward_axis`和`camera_robot_up_axis`表示“机器人轴在T265 Pose本体坐标中的
坐标”。实车验证上一版`-T265 X`会使地图轨迹与实际运动相反，因此当前校准轴为车头
`+T265 Pose X`。实测“车向右而地图向左”说明横向轴反了；镜头朝上时机器人上方对应
`-T265 Pose Z`，因此左向由`up×forward`得到`-T265 Pose Y`：

```ini
camera_robot_forward_axis = +x
camera_robot_up_axis = -z
```

程序从完整`translation.xyz / velocity.xyz / quaternion.xyzw / angular_velocity.xyz`构造相机到
世界旋转。车头世界向量按`f_W=R_WC*f_C`计算，底盘原始航向为
`atan2(-f_W.x,-f_W.z)`；不再使用俯仰接近90°时有奇异性的绕Y欧拉角。

第一帧保存三维位置、车头/左向世界向量和原始航向。后续位移分别点乘初始车头和左向量，
相对航向通过连续四元数航向差累计，再叠加所选出发区135°/45°/225°/315°锚点。程序同时
保留四元数差分yaw rate和把Pose角速度投影到机器人世界上方向得到的gyro yaw rate；轮式
预测采用后者的高频增量，四元数姿态负责绝对校正。

## F407/T265 专用对照调试

F407 `feat/uart-motion-debug` 分支（当前 `9774dac`）已按固定15字节帧通过USART3发送编码器累计
位置：`TYPE=0x15`为100 Hz左右的三路累计编码器位置，`M1=右轮、M2=左轮、M3=后轮`，三个
编码器符号均为`-1`；`TYPE=0x17`为上位机运动命令，`TYPE=0x18`为F407运动状态。接口定义以
[F407运动调试交接](https://github.com/gandizm/F407-Rescue-Robot/blob/9774dac/docs/f407_motion_debug_handoff.md)
和`Main/Src/DebugMotion.c`为准。本调试采集程序不发送运动命令，不会主动启动F407任务：

```text
A3 B3 15 SEQ M1_H M1_L M2_H M2_L M3_H M3_L DT STATUS CRC_LO CRC_HI C3
```

运行前必须关闭`rescue_map`/任务程序，避免占用同一串口：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
python3 localization/tools/t265_f407_debug.py \
  --uart /dev/ttyS1 --label rotate_90_180_360 --trial rotate
```

程序会生成一份临时定位配置，把`encoder_fusion_weight`设为0，同时保留编码器原始帧和三轮轮式
轨迹记录；编码器不会反过来影响T265主定位。每次运行自动创建独立目录：
`rescue_map/runtime/history/t265_f407_debug/<时间>_<标签>/`，保存：

- `localization_debug.csv`：每帧T265、F407累计计数、三轮原始/融合增量、raw tracking-origin、修正后的robot-center、轮式轨迹和差值；
- `localization_result.json`、`stm32_status.json`、`localizer.log`：实时快照、状态帧和运行输出；
- `metadata.json`、`analysis.json`：接口版本、命令行和自动对照结果。

建议按以下顺序分别采集并用不同`--label`保存：静止5秒；以三轮旋转中心原地逆时针90°、180°、
360°；车体forward直行1 m；车体left横移1 m；最后做一次平移同时旋转。既有日志可单独分析：

```bash
python3 localization/tools/t265_f407_debug.py \
  --analyze rescue_map/runtime/history/t265_f407_debug/<一次运行目录> \
  --trial rotate
```

`analysis.json`的判定重点：`initial_reference_check`检查T265 raw tracking origin和修正后
robot-center的初始物理偏置；`rotation_check`比较90/180/270/360°理论圆周位移与修正残差；
`wheel_to_t265_fit`给出`T265车体增量 = A × F407轮式增量`，对角线偏离1主要表示尺度误差，
非对角线较大表示轮序、符号或平面轴混用；`interface_observation`检查F407帧是否连续、有效和
状态位是否正常。原始数据仍以CSV为准，自动结论不能替代实际原地旋转和直线标定。

如果需要像原地图程序一样观察轨迹并主动发送`feat/uart-motion-debug`分支的F407运动命令，运行：

```bash
python3 localization/tools/t265_f407_motion_map.py
```

窗口只提供该分支的`TURN_REL`、`MOVE_DISTANCE`和`STOP`，对应原地旋转、forward/left定距和
安全停车；不发送旧救援任务协议的`TYPE=0x11/0x12/0x18`命令。F407运动命令直接通过USART3发送一次，
F407用本地IMU/三轮里程计闭环完成，返回的`TYPE=0x18`状态会显示进度/剩余/健康位。地图同时显示
T265修正中心、raw tracking origin和轮式中心三条轨迹，运行日志保存在独立目录。

四项定位实验的实际操作：

1. 点击`原地90°/180°/360°`，程序分别发送`TURN_REL`，F407自动使用陀螺仪闭环旋转，状态变为`DONE`后实验自动结束。
2. 点击`直行1m`，程序发送相对车体forward方向的`MOVE_DISTANCE(0°,1000 mm)`，F407用本地三轮里程计定距。
3. 点击`横移1m`，程序发送相对车体physical-left方向的`MOVE_DISTANCE(90°,1000 mm)`，这是当前分支支持的真正横移测试。
4. 点击`转向返航`，程序先按T265当前位置计算指向场地中心的相对转角，收到转角`DONE`后自动发送forward方向定距移动，形成一次转向加直线返航。

每次实验的发送、`RUNNING`、`DONE`、`FAULT`和`STOPPED`状态会追加写入运行目录的`events.jsonl`和
`f407_motion_status.jsonl`，分析报告会把这些状态与CSV对照。F407分支接口定义见
`f407_motion_protocol.py`；程序退出时会自动发送STOP。

安装桌面启动图标：

```bash
bash localization/tools/install_desktop_launcher.sh
```

## UART 兼容性

F407和上位机均使用115200 8N1、`A3 B3 ... C3`固定15字节帧和CRC-16/Modbus：

```text
A3 B3 15 SEQ M1_H M1_L M2_H M2_L M3_H M3_L DT STATUS CRC_LO CRC_HI C3
```

上位机救援任务仍使用`localization/src/f407_protocol.*`中的旧任务协议；本运动调试地图使用独立
的`localization/tools/f407_motion_protocol.py`，不能把旧任务的`TYPE=0x18`命令当作本分支的运动命令。

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

保持UART任务通信但禁止编码器进入融合：

```bash
./run_localization.sh \
  --uart /dev/ttyS1 \
  --ignore-encoders \
  --command-file ../rescue_map/runtime/uart_command.bin \
  --stm-status ../rescue_map/runtime/stm32_status.json
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
5. 原地旋转 360°，对比轮式运动学角度与T265陀螺角度并测量 `wheel_center_radius_m`；该参数
   当前主要用于日志诊断，未标定前可保持 0；同时检查 `gyro_pose_sync_error_deg` 是否周期归零。
6. 从四个出发区分别越过/绕过减速带，确认 JSON 中 `wheel.gate` 显示 `startup_obstacle` 或 `corner_obstacle`。
7. 在平地复测闭合路线，用 CSV 比较 T265、轮式预测和融合输出，再调整协方差和速度残差门限。
