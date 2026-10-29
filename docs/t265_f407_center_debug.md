# T265/F407 中心起点定位调试

## 2026-10-29：新竖装外参候选（仅调试，尚未实车确认）

本节覆盖下文历史桌面默认外参/覆盖规则；正式比赛的模板、四角起点和
135/45/225/315°航向保持原值，必须在本阶段实测确认后另行同步。
本轮不接雷达、主yaw或官方轮速输入，不改变比赛状态机、mode51及UART任务配对。

用户本对话确认：T265仍竖装；自身绕车体竖直轴俯视顺时针30°；测量点为SDK
定位中心，已考虑厚度。车中心到定位中心69.221mm，右后、与正后方夹角60°：
`forward=-0.0346105m, left=-0.0599471445m`。没有额外叠加SDK内部IMU偏移。
这些是测量初值，不是已通过精度验收的标定。

配置轴表示车体轴在T265 Pose坐标中的分量，`R_pose_body`以forward/left/up为列。
旧前向+x、上向-y，左向由`up × forward`得到+z。相机自身顺时针转30°后，
同一车体前向在新相机坐标中为`(cos30,0,sin30)`；候选左向为
`(-sin30,0,cos30)`，上向仍为`(0,-1,0)`，不是改地图初始航向。
SDK世界坐标的+y指向天空；静置水平时应检查`up_world≈(0,1,0)`。
这能核查竖装与重力对齐，不能仅凭数学计算宣称实物轴向已验证。

解析器继续接受±x/±y/±z，并增量接受三个逗号分隔分量，先归一化，再拒绝
非有限、零向量或不正交的forward/up；允许`0.8660254,0,0.5`这样的舍入值。
独立候选配置为`localization/config/t265_debug_new_mount_candidate.conf`。
桌面默认读取该文件；显式`--config`时尊重所选文件的轴与偏置，已去掉旧硬编码覆盖。
只保留调试专用中心起点、原45°显示初始偏移，并强制比例禁用/1.000、轮式融合权重0、
导航距离补偿关闭。T265单源与pose jumping继续启用。

投影先通过完整四元数旋转车体轴与已知平面杆臂：
`r_world=R(q) R_pose_body [forward,left,0]`，
`delta_center=delta_camera-(r_world-r_world_start)`；
速度使用同一采样的`v_center=v_camera-omega_world×r_world`，再投影到车体轴。
只对最终位置取水平地图分量，保留完整姿态；不将每个体轴分别压平后代替完整旋转。
未提供竖直杆臂测量值，以上不声称已标定竖直偏移；本轮按给定平面偏置处理。

### 启动与人工实测

先确认实际已烧录提交和MotionDebug模式。UART状态不能证明烧录提交；
`expected_lower_firmware`仍只是配套功能参考。默认未确认时入口只接收，运动按钮不发命令。
停止其他占用UART/T265的入口后，将实际提交写入`F407_COMMIT`，通过原入口启动：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
cmake --build localization/build --target t265_omni_localizer -j2
: "${F407_COMMIT:?请先将F407_COMMIT设为实际已烧录提交}"
localization/tools/desktop_launcher.sh \
  --actual-lower-firmware "$F407_COMMIT" --actual-lower-mode MotionDebug \
  --reference-description '填写车中心卷尺零点、地面直线及角度基准布置'
```

保持场地和速度一致，先静置记录重力轴/漂移，再按以下顺序逐段人工触发；
每段都在地面标记车体旋转中心和朝向，独立量起止点。不要用相机外壳点代替车中心。

1. 距离1000mm，V前进→B后退；再B后退→V前进，各段停稳后测量。
2. L左移→H右移；再H右移→L左移，同样逐段记录，保留返程闭合差。
3. O/P/0分别正向90/180/360°；1/2/3分别反向90/180/360°。
   正向为车体俯视逆时针；线上TURN仍按既有符号取反。每段独立检查实际地面角度，
   用完整圈数记录±360°，不要把整圈写成0°，也不要用“经过90°”的样本冒充停稳终点。
4. DONE只结束命令等待。人工确认静止后按M标记；按G在原窗口输入
   `前向毫米,左向毫米,有符号转角度`，未知填`?`，回车保存，Esc取消输入。
   位移沿该动作开始时的车体前/左轴测量。例如只测转角时前两项填`?`。
   `--reference-description`必须记录真实基准，示意文字不能冒充实测布置。
   停稳、测量和录入完毕再开始下一段；基准变化时另开会话。

现有Y转向返航仍可用，但不是独立卷尺基准，不用于本轮自动定距精度判定。
操作者确认固件后，所有动作仍由用户按钮/键盘触发；本阶段实施过程中没有自动发车。

### 日志与报告接口

日志直接保存在RDK的`rescue_map/runtime/history/t265_f407_motion_map/`，每会话独立目录：

- `t265_raw_pose.csv`：每个新SDK Pose的帧号、原始采集时间及时间域、接收单调时间、
  可判断时的采集年龄、三维位置/速度/角速度、完整四元数和双confidence。
  这条日志在置信度与启动门禁之前保存；未知时钟域的年龄为inf，不能视为已对时。
- `localization_debug.csv`：原相机轨迹与修正车中心、完整姿态、未展开/累计航向、
  原始编码器及轮式参考；追加帧号、时间域、采集年龄、接收时间和up_world。
- 原`localization_events.jsonl`继续记录跳变、丢帧、SDK选项回读与定位事件。
- `commands.jsonl`、`f407_motion_status.jsonl`、`uart_rx.jsonl`保留指令/回执及原始串口。
  `events.jsonl`用test_id对应动作起止、请求参数、定位快照、人工停稳点和录入。
- `ground_truth.jsonl`只保存操作者独立实测，未知项为null；指令目标不会填成真值。
- `analysis.json`逐动作给出T265减实测、实际运动减指令两组差值与confidence/样本年龄。
  无实测时不生成精度结论；`accuracy_verified=false`。不再拟合轮式比例，原全程角度
  穿越统计改名为`angle_crossings_not_settled_measurements`，仅供诊断。

`--raw-pose-csv FILE`为现有localizer新增的可选诊断接口，不启动额外设备。
RDK只保留最近五次明确归属本模块且完整保存的会话：新会话日志/分析/停止事件保存后
写完成标记，再清理超出的旧会话，当前会话计入五次且始终保护。
历史会话须有本模块metadata和末尾session_stopped事件才能判完整；无法确认、活动、
含未知用户文件/子目录或符号链接的目录跳过。只删除已知模块日志文件，不递归清目录；
不会触及其他模块日志、模型、标定、代码、vision/456或雷达与融合资料。

### 本阶段验证边界

复用`test_omni_localization`和`test_f407_motion_protocol.py`，只扩展已有检查。
新几何案例是合成四向/往返、正反90/180/360°、倾斜姿态及完整杆臂速度检查；
Python用临时目录检查入口不覆盖外参、反向操作、真值录入/误差拆分及日志清理边界。
它们不能证明新安装的真实重力、轴向、尺度、杆臂误差、跟踪漂移或下位机动作精度。
实际固件/模式和上述实车测量仍待用户确认完成；不因此同步正式比赛外参。
本次实际执行：localizer与test_omni_localization编译成功；
ctest仅运行omni_localization_tests，1/1通过；原MotionDebug Python检查通过；
两个入口--help、Python语法和git diff --check通过；原地图无硬件渲染通过。
未打开T265/UART、未启动比赛、未发送实车指令，也没有清理任何已有运行会话。

坐标依据：[librealsense v2.50.0 T265 Sensor origin and coordinate system](https://github.com/IntelRealSense/librealsense/blob/v2.50.0/doc/t265.md#sensor-origin-and-coordinate-system)。


桌面 `T265-F407-调试地图.desktop` 运行
`localization/tools/desktop_launcher.sh` → `t265_f407_motion_map.py`。
本入口的地图与实际定位日志初始位置都为 `(0,0)`；本轮配置使用
`start_center_m=0.0`、`start_heading_offset_deg=45.0`。默认初始航向由原4号315°
逆时针旋转45°成为0°，朝地图右方、与安全区长边平行；图标、定位和日志一致。旋转按钮仍为
逆时针90°、180°、360°。正式比赛的配置、出发位和坐标系不受影响。

## 距离与使用

- 直行、向车体左侧横移共用面板选定的距离，默认1000 mm。
- 2026-10-15按F407 4071c71的物理车体接口，发送直行 `MOVE_BODY(+距离,0)`、左横移 `MOVE_BODY(0,+距离)`，
  期望逆时针角度取负数发送；转向返航使用相同修正。
  期望方向与实际串口方向/转角分别记录，`metadata.json`保存
  `forward_command_sign=+1`、`left_command_sign=+1`、`turn_command_sign=-1`。
- 轮式参考采用F407物理公式：`F=-(m1-m2)/sqrt(3)`、`L=(m1+m2-2*m3)/3`，
  默认`encoder_to_robot_yaw_deg=0`。旧180°同时翻转左右，已移除；不修改历史日志。
  原始累计计数、每轮-1符号、1061.268 counts/rev及T265轴/航向保持。
- 粗调±100 mm，细调±10 mm，范围1～10000 mm；点击移动不会重置成1 m。
- `I/K`减少/增加100 mm，逗号/句号减少/增加10 mm。
- `V`直行、`L`左横移，`O/P/0`分别旋转90°/180°/360°。
- `U/J`速度±50 mm/s，初始300 mm/s，沿用50～700 mm/s范围。
- `S`或STOP停车；正在进行的实验再次点击可中止；`Q`退出并保存分析。
- 转向返航保持已有方式，目标为地图中心，使用独立的返航距离，
  不覆盖面板下次直行/横移所选的距离。
  已在中心10 mm内时无需发送动作。

2026-10-11根据直行实测方向相反，桌面测试默认前向轴由`-x`修正为`+x`，
随后依据20:47:58会话的重力方向和陀螺/四元数对比，仅将测试上向轴修正为`-y`。
当前测试轴为 `forward=+x, up=-y`，按原规则派生`left=+z`。
完整四元数的相对yaw增量保持原方向，初始航向仍为0°；横移方向后续实测确认。
2026-10-15正式比赛已同步实测安装轴和偏置，参见`t265_side_mount_baseline.md`；
正式比赛保留各出发区位置、航向，不使用调试中心起点与额外45°。
保留完整四元数投影。桌面默认偏置为后20mm、右70mm，平移比例禁用、比例值1、
编码器融合权重0。轮子样本仍记录，不能用轮式结果反过来修正本轮T265。
本工具不会在取样前宣称轴向已校准。

需要对比另一份安装参数时复制配置文件，使用：

```bash
/home/sunrise/RDK_X5/shijue_fangan/localization/tools/desktop_launcher.sh \
  --config /绝对路径/测试定位.conf
```

使用默认模板时，测试入口覆盖前向轴为`+x`、上向轴为`-y`；通过`--config`指定其他文件时，
保留该文件的安装轴、轮式安装角和偏置。默认测试轮式安装角按上文使用0°，
偏置覆盖为`forward=-0.020m, left=-0.070m`；比例与其余参数继续沿用，同时把起点放在中心、
融合权重清零、导航距离补偿禁用、开局与角点轮式记录门禁解除。
初始航向可在启动前用 `--zone 1/2/3/4` 指定，调试入口加45°后依次对应180°/90°/270°/0°；
运行中不提供只改变图标而不改变定位参考的虚假锚点切换。

## 2026-10-12位置连续性与候选补偿

`76e828b`曾在桌面入口强制关闭位置跳变。后续实车记录出现明显距离少算，
当前桌面入口改为自动传入`--enable-pose-jumping`，明确将SDK的
`enable_pose_jumping`恢复为1，保留相同诊断用于单项对照。使用同一pipeline的
pose sensor，不另开USB设备。原`--disable-pose-jumping`仍可用于明确指定的对照运行。
mapping、relocalization和map preservation在此调试路径只读取，不额外改变。
实际读取的选项前后值记录在本轮`localization_events.jsonl`的`t265_debug_option`中；
`localizer.log`同时打印实际值。`metadata.json`的requested字段只表示请求，不能代替SDK回读。

本轮原始CSV、UART计数流、命令、状态和事件记录保持。新增诊断仅记录：

- `t265_pose_gap`：设备姿态时间间隔超过50ms。
- `t265_position_discontinuity`：间隔不超过20ms，原始三维位置增量与梯形积分速度的差超过50mm。

诊断不裁剪原始数据、不伪造固定位置、不增加运动故障锁存或自动联停。
恢复位置跳变后仍检查这些事件及距离误差，不能只凭轨迹连续与否判断精度。

2026-10-15桌面入口增加`--t265-only-output`，每个有效T265位姿经过已有完整
四元数投影和相机偏置修正后直接作为地图位置、航向输出，跳过EKF预测与观测更新，
不再被600mm创新门限拒绝。T265置信度为0时仍保留最后有效位置，不伪造新定位。
此选项与`--ignore-encoders`不同：继续接收、解算、绘制和记录轮式参考轨迹，
轮式参考使用三轮位移及T265航向，但不参与最终定位。没有向T265 SDK注入轮速。
桌面默认采用用户确认的初步偏置：后20mm、右70mm，即
`camera_offset_forward_m=-0.020`、`camera_offset_left_m=-0.070`。
绿色raw原点仍表示相机位置，橙色T265中心表示外参修正后的车体中心；紫色为轮式参考。
历史日志保持原值。正式默认入口现已同步此T265直通输出路径及安装参数。

以下两份为保留的历史候选配置，不是当前桌面默认参数。只有显式通过
`--config`选择时才使用其偏置与比例：

```bash
# A：仅候选偏置，后29mm、右79mm，比例仍为1.000。
/home/sunrise/RDK_X5/shijue_fangan/localization/tools/desktop_launcher.sh \
  --config /home/sunrise/RDK_X5/shijue_fangan/localization/config/t265_debug_offset_candidate.conf

# B：相同候选偏置，开启1.058比例。
/home/sunrise/RDK_X5/shijue_fangan/localization/tools/desktop_launcher.sh \
  --config /home/sunrise/RDK_X5/shijue_fangan/localization/config/t265_debug_offset_scale_candidate.conf
```

两份候选都沿用当前测试的+x/-y轴、中心起点、初始0°、轮式0°附加旋转、MG310计数和
零融合权重。实测偏置得到后应替换候选值，再决定是否保留比例；本次未把候选写入正式比赛配置。
2026-10-12旧测试中停止后转角曾超过目标3～5°，不是4071c71的实测结论。
不通过修改T265航向比例处理，也不用自动TURN的最终偏差直接标定IMU；本仓库未修改下位机。

## F407 应烧录的版本（可直接交给下位机助手）

2026-10-15已只读核对公开仓库：
`https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot`。
本工具配套代码提交为：

```text
87aaba26f2c028680027b54df08d6ba825b047e9
```

本轮按该提交的接口修正符号；旧实测记录未确认烧录版本，需用该固件复核前进和左移。
实际烧录提交需操作员确认，不能把Git版本当作烧录证明。
IMU采样恢复不代表旧角度自动找回；静止时通过既有下位机调试入口发送RESET建立局部参考，
随后由操作者发起新测试，不自动续跑旧动作。上位机兼容RESET_ODOM回执，不增加新按键。
如需从此公开提交部署，选择已有 **MotionDebug** 配置，重新编译并烧录
`build/MotionDebug/WWW.elf`；不要烧录NormalRun或Gamepad产物。
该配置通过 `APP_ACTIVE_MODE_OVERRIDE=APP_MODE_MOTION_DEBUG_TASK` 选择调试模式。
CLion可使用 `DAPLink MotionDebug`。命令行等价为：

```bash
cmake --preset MotionDebug
cmake --build --preset MotionDebug
```

该提交已有MG310：70 mm轮径、20.409减速比、13线四倍频、
1061.268 counts/rev、60%驱动占空比上限，速度PID为5/0.12/0、直行航向PID为2/0.005/0。
请确认烧录的是此次MotionDebug产物并反馈提交和模式；无需额外修改
LCD、测试系统或添加新的堵转/反向计数联停。

对应 `Main/Inc/vision.h`、`Main/Src/vision.c`、`Main/Src/Debug.c`：

- USART3 / RDK `/dev/ttyS1`，115200 8N1；固定15字节，A3 B3头、C3尾，
  CRC16 Modbus覆盖TYPE..P7，CRC低字节在前。
- `0x15`原始16位累计编码器计数，保留回绕；不能先缩放计数。
- 命令 `TYPE=0x19`，状态 `TYPE=0x1A`。
- STOP opcode0、flags=0x09，其余参数0。
- TURN_REL opcode2、flags=0x09，arg1为有符号0.1°，arg2=0，speed为0.1°/s。
  上位机按固件130 mm转弯半径把既有面板车轮切向速度换算为角速度。
- MOVE_BODY opcode3、flags=0x0B，arg1/arg2为有符号车体forward/left毫米，
  speed为mm/s；本测试入口发送物理直行=(距离,0)、左横移=(0,距离)。
- 状态P0为命令SEQ、P1为状态、P2为fault、P3为opcode、P4/P5为有符号进度、
  P6/P7为IMU绝对航向0.01°。旋转进度单位0.1°，移动进度单位mm。
- 上位机按命令SEQ、opcode、本次发送后的时间与新鲜度识别DONE/FAULT/STOPPED；
  不把状态外层SEQ当命令SEQ。1秒无匹配状态取消自动后续段；迟到DONE不会恢复。
- TURN超过10秒可能被下位机超时；界面会提示理想时间已达到该限制的请求，不改固件时限。
- 0x1A航向来自F407 Location，是局部复位原点、逆时针为正、模360的航向，
  不是原始IMU积分角。T265是所选场地起点参考的逆时针航向，不直接比较绝对数值。
- 可用`--actual-lower-firmware <实际烧录提交> --actual-lower-mode MotionDebug`记录
  操作员确认信息；默认unconfirmed，不通过UART推测。FIFO细分诊断不在0x1A内。

旧 `gandizm@9774dac` 的0x17/0x18调试协议、`fa6000c`的0x1B/0x1C协议
均不是此版本的MotionDebug接口；不能只根据旧调试文档的帧编号烧录。
公开源码及编译配置可核对，实机烧录提交需要操作员确认。

## 保存的数据

每次启动新建独立目录，退出不会覆盖或清除：

```text
rescue_map/runtime/history/t265_f407_motion_map/<时间>_interactive/
```

- `localization_debug.csv`：全速T265原始translation、velocity、quaternion、
  angular velocity、投影轴、原始/中心轨迹、累计计数与轮式增量等。
- `localization.conf`、`metadata.json`：本轮实际配置、中心起点、初始航向、
  上位机提交、预期下位机提交和协议；实际下位机提交不能由UART推断。
- `commands.jsonl`：发送时间、原始十六进制帧、SEQ、opcode、flags、真实参数。
- `events.jsonl`：实验编号、实验开始/完成/中止、选定距离和速度、位姿及状态快照。
- `uart_rx.jsonl`：完整接收字节流和单调时钟接收时间，包括原始计数帧、
  状态帧、噪声或CRC错帧；T265启动失败时也保留F407数据。
- `f407_motion_status.jsonl`及JSON：全部运动状态历史和最新快照。
- `localizer.log`、`localization_result.json`、`analysis.json`：运行输出、
  最新位姿及自动对照。自动分析读取本轮真实偏置，不写死旧朝上安装偏置。
- `localization_events.jsonl`：SDK选项回读及T265采样空档、位置不连续诊断。

退出后可重复分析同一次目录；默认从该目录的配置读取偏置，保留当时协议：

```bash
python3 localization/tools/t265_f407_debug.py \
  --analyze rescue_map/runtime/history/t265_f407_motion_map/<时间>_interactive/
```

建议每组先静止几秒，再分别做直行、左横移和原地旋转；用卷尺、地面标记
记录实际距离、实际方向和实际角度。F407的DONE表示它认为动作完成，
不能替代外部测量。后续应按实验事件的时间与编号截取CSV，
用原始三维数据判断安装轴、yaw符号、距离尺度及旋转中心偏置。
当前轮式轨迹的航向也使用T265，因此二者航向一致不能作为独立校准证明。
