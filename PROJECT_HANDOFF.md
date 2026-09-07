# shijue_fangan 项目交接说明

更新时间：2026-09-07

## 1. 新对话先做什么

请先读取本文件，再检查当前仓库状态：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
git status --short --branch
git log -8 --oneline
```

当前基准提交：

```text
dca0c13 Align encoder axes and enable T265-only missions
```

GitHub：<https://github.com/danmo-teng/shijue_fangan>

用户希望助手直接修改、检查并推送这个上位机仓库。除非用户明确改变要求，否则：

- 只修改RDK X5上位机的视觉、地图、定位和任务代码；
- 不修改F407电控仓库；
- 不生成新的F407补丁；
- 可以只分析F407代码，并在文档中备注电控侧需要配合的协议语义；
- 不要反复进行耗时的完整编译，按改动风险做一次有针对性的检查即可。

电控仓库仅供协议分析：
<https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot>

## 2. 硬件与基础环境

- 主控：RDK X5。
- 定位：Intel RealSense T265，镜头朝上安装。
- 下位机：STM32F407。
- 摄像头：`/dev/video0`，MJPEG `1280×1024 @ 180 FPS`。
- UART：RDK `/dev/ttyS1`，115200 8N1，3.3 V TTL。
- RDK物理8脚TX → F407 PD9/RX。
- RDK物理10脚RX ← F407 PD8/TX。
- 两板必须共地。
- librealsense版本：2.50，本机依赖根目录通常是
  `/home/sunrise/文档/ChatGPT/T265`。

## 3. 一键启动

桌面已经安装：

```text
/home/sunrise/桌面/智能救援一键启动.desktop
```

命令行入口：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/rescue_map
./run_rescue_map.sh
```

重新安装桌面图标：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/rescue_map
./install_desktop_launcher.sh
```

启动界面选择出发区、红/蓝方、定位方式，点击开始后同时启动：

1. T265定位或T265+编码器融合；
2. UART收发与任务命令转发；
3. YOLO识别窗口；
4. 连续搬运任务状态机。

## 4. 两种定位模式的准确含义

### T265+编码器

- 打开UART；
- 接收F407的三轮编码器帧；
- 编码器参与EKF预测；
- T265参与位置和航向校正；
- 启动完整视觉任务。

### 仅T265

- 仍然打开UART；
- 仍然发送赛前配置、视觉报告和任务命令；
- 仍然启动完整视觉任务，小车可以正常出发；
- 定位程序增加`--ignore-encoders`，只是不让编码器进入EKF。

不要把“仅T265”改回检测演示模式，也不要关闭UART，否则F407收不到配置，小车不会出发。

## 5. 场地坐标

- 场地中心：`(0,0)`。
- 场地边界：X/Y均为`±1.500 m`。
- `+X`：地图右侧。
- `+Y`：地图上方。
- yaw：从`+X`开始逆时针增加。

四个300×300 mm出发区的车体中心固定为：

| 出发区 | X | Y | 初始yaw |
|---|---:|---:|---:|
| 1 左上 | -1.350 m | +1.350 m | 135° |
| 2 右上 | +1.350 m | +1.350 m | 45° |
| 3 左下 | -1.350 m | -1.350 m | 225° |
| 4 右下 | +1.350 m | -1.350 m | 315° |

地图保留的角点径向参数默认是：

```text
150×sqrt(2) = 212.132 mm
```

定位配置必须保持：

```ini
start_center_m = 1.350
```

任务启动前会检查第一帧有效融合位置。与理论出发点平面误差超过20 mm时输出：

```text
START_POSE_MISMATCH
```

并禁止发送赛前配置命令。

## 6. T265镜头朝上坐标解算

不要重新引入`heading_y()`欧拉角，也不要使用固定
`camera_to_robot_yaw_deg`猜测补偿。镜头朝上时欧拉角处于奇异区域。

当前代码使用完整：

```text
translation.xyz
velocity.xyz
quaternion.xyzw
angular_velocity.xyz
```

当前经实车前后、左右方向校准后的T265 Pose轴配置：

```ini
camera_robot_forward_axis = +x
camera_robot_up_axis = -z
```

由此得到机器人左向为`-T265 Pose Y`。注意T265 Pose坐标和T265内部IMU子坐标不是同一套轴。

核心算法：

```text
f_W = R_WC * f_C
yaw = atan2(-f_W.x, -f_W.z)
relative_yaw = 连续帧yaw差的解缠累计
field_yaw = start_zone_yaw + relative_yaw

delta = p - p0
forward = dot(delta, f_W0)
left = dot(delta, l_W0)
```

yaw rate来自连续yaw差分和低通滤波，不直接假定
`angular_velocity.y`是底盘偏航速度。

相关文件：

- `localization/include/fusion.hpp`
- `localization/src/fusion.cpp`
- `localization/src/main.cpp`
- `localization/config/localization.example.conf`
- `localization/tests/test_omni_localization.cpp`

## 7. 编码器坐标修正

用户实测：旧三轮运动学中，物理前进被解算成左移。当前增加了独立轮式平面旋转：

```ini
encoder_to_robot_yaw_deg = -90.0
```

当前关系：

```text
corrected_forward = raw_left
corrected_left = -raw_forward
```

这个参数只影响编码器，不影响已经正确的T265四元数坐标。若后续发现：

- 仅T265轨迹正确；
- T265+编码器轨迹才偏转；

应优先检查`encoder_to_robot_yaw_deg`、三个`encoder_sign`和三轮公式，不要再修改T265轴配置。

其他当前参数：

```ini
wheel_diameter_m = 0.070
counts_per_wheel_revolution = 1768
encoder_sign_m1 = -1
encoder_sign_m2 = -1
encoder_sign_m3 = -1
wheel_center_radius_m = 0.130
```

`wheel_center_radius_m`是车体旋转中心到轮子滚动作用线的垂直距离，不是105 mm推板距离或150 mm前拨板距离，仍需通过原地旋转360°实测校准。

## 8. 返航融合策略

SEARCH/APPROACH维持普通T265主导融合。NAV/RETURN进入轮式进度优先模式：

- 编码器以100 Hz预测平移；
- T265航向持续校正；
- T265位置只按20 Hz、较低权重纠偏；
- mapper confidence为0时进一步降低T265位置权重；
- 远离目标且只有速度残差冲突时，可保留合理编码器增量：
  `navigation_encoder_override`；
- 距目标300 mm内，如果轮子在转而T265几乎不动，冻结轮式增量：
  `navigation_near_target_slip`。

当前配置：

```ini
navigation_t265_position_sigma_multiplier = 8.0
navigation_t265_position_correction_rate_hz = 20.0
mapper_zero_position_sigma_multiplier = 12.0
navigation_near_target_m = 0.30
navigation_slip_wheel_speed_mps = 0.10
navigation_slip_t265_speed_mps = 0.05
```

相机tracking origin相对底盘旋转中心的偏置尚未实测，目前是：

```ini
camera_offset_forward_m = 0.0
camera_offset_left_m = 0.0
```

不要把机构的105/130/150 mm参数填入这两个字段。

## 9. 视觉识别

默认模型：

```text
vision/models/best_bayese_320x320_nv12.bin
```

YOLO类别映射：

```text
conmon -> green_supply
kernel -> core_black
risk   -> danger_cyan
wound  -> injured_orange
safe_blue
safe_red
```

任务状态机只使用四类物资/伤员，不使用安全区视觉类别判断完成。

性能路径：

```text
1280×1024 MJPEG
→ JPU NV12
→ VSE缩放到320×256并填充320×320
→ X5 BPU YOLOv8s
```

默认置信度阈值：0.50。传统视觉仍作为显式备用，不应删除。

## 10. 连续搬运任务流程

1. 开局只寻找并搬运一次普通物资。
2. 摄像头下压后，物资在画面任意位置连续确认3帧。
3. 持续发送`GRAB_CONFIRMED`，等待新鲜`GRIPPER_CLOSED=1`。
4. 第一件普通物资完成后，允许普通、核心、危险物资和伤员。
5. 普通/核心/危险物资送往物资半区中心；伤员送往伤员半区中心。
6. 投送完成后张爪退出，靠近场地中心到600 mm圆周附近，再搜索下一件。

分区X坐标：

- 红方物资：`x=-150 mm`；红方伤员：`x=+150 mm`。
- 蓝方物资：`x=+150 mm`；蓝方伤员：`x=-150 mm`。

## 11. 安全区几何与握手

机构参数必须严格区分：

```text
robot_body_radius_m = 0.130
push_plate_offset_m = 0.105
front_pusher_offset_m = 0.150
zone_center_x_abs_m = 0.150
```

安全区：

```text
外宽660 mm，外深360 mm
内宽600 mm，内深300 mm
内边界 |y|=1200 mm
高围栏场地侧表面 |y|=1140 mm
```

推板理论贴栏车中心：

```text
1140 - 105 = 1035 mm
```

当前NAV停车点再保留7.5 mm余量：

```text
红方 y=+1027.5 mm
蓝方 y=-1027.5 mm
```

NAV到ALIGN：

- 轴向误差≤30 mm；
- 横向误差≤50 mm；
- 航向必须朝向围栏；
- 或收到新鲜的`mode=10 + GRIPPER_CLOSED + DISTANCE_DONE`并满足合理横向/朝向条件；
- 达到后锁存`delivery_arrival_confirmed`。

ALIGN到ENTER：

- F407状态新鲜；
- `mode=11`；
- 上位机yaw误差≤2°；
- 连续至少2帧且保持100 ms；
- ALIGN阶段只发送对正，不允许前进。

完成投送：

- 必须已锁存到达；
- 只接受新鲜`mode=15`；
- 位姿在25 mm范围内稳定0.8秒；
- `mode=14`不能完成；
- mode 14稳定堵转超过0.5秒时发送STOP并进入FAULT。

## 12. UART协议与线程

固定15字节帧：

```text
A3 B3 TYPE SEQ P0..P7 CRC_LO CRC_HI C3
```

主要类型：

- `0x11`：赛前配置；
- `0x12`：视觉目标；
- `0x15`：F407三轮编码器；
- `0x16`：旧版可选实时位置，当前默认关闭；
- `0x17`：F407任务状态；
- `0x18`：RDK任务命令。

线程关系：

- 视觉推理：新摄像头帧触发；
- 任务规划：独立50 Hz，读取最新定位和STM32状态；
- UART任务心跳：独立100 Hz，刷新SEQ与CRC；
- 规划文件超过250 ms没有更新时停止代发，等待下位机看门狗停车。

`TYPE=0x18`导航载荷：

```text
P0      COMMAND
P1      FLAGS
P2/P3   最新剩余距离mm
P4/P5   0
P6/P7   最新绝对航向，0.01°
```

上位机不再持续发送X/Y实时位置坐标。

## 13. 运行诊断文件

都位于`rescue_map/runtime/`，通常不提交Git：

- `session.json`：出发区、红蓝方、定位模式、初始位姿；
- `localization.conf`：本次自动生成的定位配置；
- `localization_result.json`：融合位置、T265、轮式门控、NAV融合诊断；
- `uart_command.bin`：视觉任务写入的最新15字节帧；
- `stm32_status.json`：F407状态及实际UART转发统计；
- `mission_diagnostics.json`：50 Hz任务规划详细状态；
- `delivery_contact_pose.json`：围栏接触参考，仅记录，不自动重置EKF。

识别窗口重点字段：

```text
stm mode/fault/ack
planner pose_age/valid/command_age
command heading/remaining
relay seq/age/tx/err
```

定位终端重点字段：

```text
wheel=accepted
wheel=velocity_mismatch
wheel=navigation_encoder_override
wheel=navigation_near_target_slip
nav=wheel_primary
wprog=...m
t265pos=correct/yaw_only
```

如果车辆运动但剩余距离500 ms基本不变化，任务窗口会输出警告。

## 14. 当前最需要实车验证的事项

最新修改后应依次做短距离低速测试：

1. 仅T265模式能否发送配置并正常出发。
2. 仅T265模式下，实际前/后/左/右是否与地图完全一致。
3. T265+编码器模式下，实际前进0.5 m是否主要改变地图前向，而不是左向。
4. 横移右0.5 m时编码器融合方向是否正确。
5. 若仅融合模式方向错误，只调整编码器轴/符号，不再改T265轴。
6. 原地360°复核`wheel_center_radius_m=0.130`。
7. 测量T265 tracking origin相对底盘旋转中心的前向/左向偏置。
8. 返航接近围栏时确认`navigation_near_target_slip`能冻结空转增量。

## 15. 测试命令

针对定位/地图：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
cmake --build localization/build -j4
ctest --test-dir localization/build --output-on-failure
python3 rescue_map/tests/test_field_model.py
python3 rescue_map/tests/test_map_app.py
```

针对任务/视觉：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
PYTHONPATH=vision:mission_test python3 mission_test/tests/test_state_machine.py
PYTHONPATH=vision:mission_test python3 mission_test/tests/test_mission_protocol.py
python3 vision/tests/test_vision_protocol.py
python3 vision/tests/test_native_resolution.py
python3 vision/tests/smoke_test.py
```

用户倾向于修改完成后只做一次与风险匹配的检查，不希望无意义地反复完整编译。

## 16. 最近关键提交

```text
dca0c13 Align encoder axes and enable T265-only missions
20249a4 Add one-click desktop rescue launcher
56e2ea4 Correct lens-up T265 lateral axis
6b6b062 Flip lens-up T265 planar body axes
e4b16e6 Project lens-up T265 pose with full quaternions
36122d8 Use wheel-primary fusion during return navigation
b3783c8 Harden safe-zone arrival and planning handshakes
bb8a138 Raise mission command heartbeat to 100 Hz
0048186 Continuously update navigation heading and remaining distance
c135ccc Decouple mission heartbeat from T265 and inference
```

仓库历史中存在早期F407补丁文档，但用户已经明确要求以后只处理上位机。除非用户再次明确授权，
不要更新、应用或推送这些电控补丁。

## 17. 给新对话的推荐开场提示

```text
请先完整阅读：
/home/sunrise/RDK_X5/shijue_fangan/PROJECT_HANDOFF.md

上位机仓库：
/home/sunrise/RDK_X5/shijue_fangan
GitHub：https://github.com/danmo-teng/shijue_fangan

请先运行git status和git log确认当前状态，再继续我的新任务。
只修改RDK X5上位机的视觉、地图、定位和任务代码，不修改F407电控仓库，也不要生成F407补丁。
修改完成后做一次与改动风险匹配的检查并推送GitHub，不要反复完整编译。
```
