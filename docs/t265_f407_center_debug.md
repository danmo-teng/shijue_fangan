# T265/F407 中心起点定位调试

## 2026-10-31：官方轮速输入调试路径，严格对时门控

基线f52164d；保留当前安装候选，未同步正式比赛。用户确认下位机尚无新的采样时刻/epoch接口，
本阶段先完成SDK接入与严格门控。**当前0x15/0x1F不会通过门控，实际注入样本应为0。**
不能把“开启请求”、配置加载成功或本地IPC发送成功说成已经融合，更不能说实车精度已通过。

### 入口、设备所有权与A/B

沿用原桌面地图、ChassisTestClient及原C++定位进程。父进程唯一读取/控制底盘UART，
遥测订阅器通过有界的本机Unix数据报通道将合格速度交给该C++进程；不新开pyrealsense设备，
不启动第二个T265进程。SDK wheel_odometer从原pipeline.resolve获得的同一个pose_sensor构造，
调用load_wheel_odometery_config在pipeline.start之前，之后才调用send_wheel_odometry。

桌面入口不变，默认`--wheel-input off`；命令行通过原启动器选择本次SDK会话：

```bash
/home/sunrise/RDK_X5/shijue_fangan/localization/tools/desktop_launcher.sh --actual-lower-mode NormalDebug --actual-lower-service MOTIONDEBUG --wheel-input off
# 先退出上一会话，确认其进程已退出，再另开：
/home/sunrise/RDK_X5/shijue_fangan/localization/tools/desktop_launcher.sh --actual-lower-mode NormalDebug --actual-lower-service MOTIONDEBUG --wheel-input on
```

上述命令会打开调试设备，启动不发送运动；本轮实现过程中没有执行它们。on是请求开启SDK输入，
不是绕过时间门控；现有下位机仍因未合格而拒绝。固件actual继续可为unconfirmed，expected不作身份认证。
运行中没有on/off热切换。每次都有新的sdk_session_id；两种模式均显式关闭map preservation，
不导入地图。停止轮速发送不代表清除了SDK中的依赖/污染，停止/重启也没有被认证为完全擦除融合状态；
严格A/B需在设备可靠重启、没有USB占用者后独立启动并记录重启事实，勿复用已融合过轮速的轨迹充当纯T265。

### 坐标和输入定义

保持以下测试配置，正式配置不变：

```text
camera_offset_forward_m = -0.0346105
camera_offset_left_m = -0.0599471445
camera_robot_forward_axis = 0.8660254,0,0.5
camera_robot_up_axis = 0,-1,0
t265_translation_scale_enabled = false
t265_translation_scale = 1.000
```

车体B为X前/Y左/Z上。R_CB的列为forward、up×forward、up，经原配置解析器归一化；
输入按`v_C^C=R_CB*(v_B^B+omega_B^B×r_BC^B)`处理一次。主gyro原生CCW正，
deg/s转为rad/s；平面假设omega_B=(0,0,omega_z)、v_B.z=0，不能声称测到了滚转/俯仰角速度。
输出仍由既有完整姿态投影器处理C→B一次，未添加SDK内部IMU偏移或修改地图初始航向。

| 独立换算检查输入 | 期望C轴速度m/s | 反向检查 |
| --- | --- | --- |
| 前进+1m/s | (0.8660254,0,0.5) | 整向量取负 |
| 左移+1m/s | (-0.5,0,0.8660254) | 整向量取负 |
| 原地CCW+1rad/s | (0.069221,0,约0) | 整向量取负 |

输入已在C点/C轴，SDK唯一速度源index=0，scale_and_alignment=I、T=0、W=0。
会话内生成wheel_sdk_config.json，C++再次拒绝非零T/W或非单位比例/旋转，防止双重补偿。
frame_num是本SDK会话内单调uint32序号，不是采样时间，不随reset_reference重置，溢出需新会话。

### 时间门控与当前下位机边界

0x15原始轮反馈继续按已核对的新F405公式复算；旧F407横向公式没有用于输入。
当前forward/left_mps仅是主机接收区间估计；0x1F的age是打包时两项最大年龄，不能还原角速度采样时刻。
批量到达、相同SEQ、名义10ms或主机UUID均不证明同时采样/板端epoch。
不发送目标速度，不使用T265速度或轮式运动学omega替代主gyro，也不把缺数据补成零。

新增适配器只消费未来经审核解码器提供的本机标准事件（不是新线上TYPE/opcode）：

- `measured_body_velocity`：实测v_body_mps三维向量、interval_start_ns/end_ns、receive_monotonic_ns、
  sample_id、mcu_epoch、measurement_valid、capture_time_qualified、time_uncertainty_ns，
  source必须为measured_wheel_feedback。原始轮计数应继续另存核对；可由有采样时间的实测轮计数换算。
- `imu`：额外明确angular_rate_capture_ns、receive_monotonic_ns、相同mcu_epoch、
  capture_time_qualified/time_uncertainty_ns、wire_valid及原生yaw_rate_deg_s。
  使用角速度自己的采样时刻，不能冒用yaw采样时刻或重复包装旧rate作为新测量。
- capture时刻必须已经通过有误差界的时钟映射进入主机CLOCK_MONOTONIC；当前解码器不产生这些合格事件。

轮区间最大50ms；源时间不确定度各不超过5ms。要求gyro覆盖完整轮区间、样本间隔不超过20ms，
线性插值后在相同区间积分得到平均角速度，再计算杆臂速度。首个轮采样到实际SDK调用的年龄加不确定度
不得超过50ms；没有前后gyro包围、异常状态、丢失区间、过期、倒序、重复、未知/变化epoch均拒绝。
这是初始接入预算，尚未用新固件测量验证；不能把合格布尔位凭接收时间置真。
有界队列溢出丢弃积压、不重放。实测零速度可发送，缺少反馈不会产生零速度。

SDK拒绝或抛异常记录真实结果、标记轨迹依赖可疑并停止该SDK输入会话的后续发送，
由新SDK会话恢复；这仅隔离该输入流，不锁存底盘控制，不恢复ESCAPE或覆盖mode51恢复。
单个暂时过期/缺数据事件不锁存，之后的新合格数据可正常通过。
下位机配套任务见[f405_normaldebug_interface_followup.md](f405_normaldebug_interface_followup.md)。

### 噪声、日志与验证

`--wheel-noise-std-mps`默认0.10m/s，SDK noise_variance为平方0.01(m/s)^2。
这是保守工程初值，不是实测标定：当前每计数约0.207mm，10ms差分约0.0207m/s/count，
还存在轮胎/运动学、全向滑移、gyro杆臂项和时间误差。取0.10不能保证包住打滑偏差；
有错误/打滑证据仍应拒绝输入。没有照抄演示为突出效果而设的极小噪声。
SDK不支持逐样本时间或协方差；不通过缩放速度降权，不逐帧重载配置。

日志仍在RDK原模块会话目录，保留最近5次完整会话的原机制；只增加本模块文件白名单，
当前/不完整/含用户额外文件的会话受保护。本次没有运行真实日志清理。

- `wheel_input.jsonl`：原轮反馈、v_B、对应gyro样本/平均omega、杆臂项、v_C、区间与时间差、
  拒绝原因、动作的主机上下文或IPC发送结果；不具备的量为null，不伪造数据。
- `wheel_sdk.jsonl`和`wheel_sdk_status.json`：SDK配置加载结果、逐样本attempt/result、frame_num、
  调用耗时、API成功计数与依赖状态。IPC成功≠SDK成功；SDK成功≠设备融合回执或精度成功。
  本机v2.50的tm-device.cpp配置加载实现明确没有固件成功确认，load/send均在stream_write后返回true；
  日志固定result_scope=host_SDK_call_return_only、device_fusion_confirmed=false。
- 既有`t265_raw_pose.csv`/`localization_debug.csv`继续记录SDK原始相机轨迹/完整姿态与车中心轨迹，
  与wheel日志按SDK会话、主机单调时钟对应。commands/events/chassis_actions保留动作身份；
  GUI上下文明确是接收时关联，不冒充MCU捕获时动作ID。
- metadata/analysis记录输入模式和依赖来源。凡开启并使用过轮速，轮速和主gyro均成为该SDK轨迹的依赖，
  之后不得当完全独立观测再次融合或作独立精度真值。未知是否使用时按可能依赖处理。

复用原检查：C++构建、原ctest、原test_f407_motion_protocol.py扩展数学/门控/本地IPC/fake SDK检查。
这些检查不连接真实底盘或T265，不证明设备配置已加载、实际轮速已融合或地面精度已通过。
本轮实际结果：cmake构建成功，ctest为2/2通过，Python回归PASS，py_compile与git diff --check通过；
两个原程序的--help可显示新增选项。只运行离线/本机伪终端检查，没有打开真实UART/USB、启动电机或清理实际日志。

待时间接口具备后，人工逐段做off/on独立会话：前后左右1m及正反90/180/360°，每段M冻结停稳点、
用卷尺/地面角度基准G录入；D记录短时局部视觉受扰的开始/结束，具体遮挡情况另作文字记录。
组合运动须等下位机线上合同，不能把顺序平移/旋转当作已验证同时运动，本轮未自动执行任何动作。
官方示例明确完全遮挡可退化为3DoF并冻结平移，不承诺轮速能解决完全视觉失效。

官方核对来源：
[v2.50.0轮速示例/生命周期与遮挡边界](https://github.com/realsenseai/librealsense/blob/v2.50.0/wrappers/python/examples/t265_wheel_odometry/t265_wheel_odometry.py)、
[SDK接口](https://github.com/realsenseai/librealsense/blob/v2.50.0/include/librealsense2/hpp/rs_sensor.hpp)、
[配置格式](https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md#wheel-odometry-calibration-file-format)。

## 2026-10-30：桌面按钮未发送的修复与架空接口实测

用户反馈点击测试无响应，核对20261030_202942_interactive会话发现：
actual_lower_service=unconfirmed、actual_lower_firmware=unconfirmed、listen_and_control=false，
没有动作发送记录。上一版桌面Exec没有传入服务确认参数，且软件过度要求无法从UART4读回的
完整固件提交号，导致按钮被本地拦截；当时串口0x15/0x1F/0x1A接收正常。

本轮依据用户已确认的F405 NormalDebug与架空测试授权，更新原桌面入口：
Exec传入NormalDebug/MOTIONDEBUG操作者测试配置，启动不运动，点击按钮才发送。
允许固件提交号保持unconfirmed，并明确记录无法由UART4认证；expected不写进actual。
若操作者填写了已知但与14157e6合同不兼容的提交，仍拒绝动作并要求先核对。
直接不带服务配置运行Python时仍为仅监听；界面顶部显示“测试已启用”或“仅监听”。
这不是新增身份查询，也不能保证未来换板/重新烧录后的运行服务仍未改变。

实际安装更新：运行现有install_desktop_launcher.sh，已更新
/home/sunrise/桌面/T265-F407-调试地图.desktop；原文件名和入口保留。
不要求用户把expected提交号填成已确认actual才能操作。

用户确认车已架空后，本轮使用现有ChassisTestClient连接真实/dev/ttyS1，
执行一次STOP、50mm/100mm/s定距、最后STOP，没有执行整套往返或陀螺定角：

| 请求 | 线上SEQ | 实际回执 |
| --- | ---: | --- |
| STOP | 103 | 匹配STOPPED，fault=0 |
| move_body(+50,0,100) | 104 | 见RUNNING后匹配DONE，progress=50，fault=0 |
| STOP | 105 | 匹配STOPPED，fault=0 |

原始计数差为[+216,-246,0]，符合前进计数符号；收到0x15/0x1F各70帧、0x1A 13帧，
CRC错误和对应序号缺口均为0。已关闭客户端释放串口；末次状态为STOPPED。
原始字节、动作与回执保存在
rescue_map/runtime/history/t265_f407_motion_map/20261030_203626_interactive/，
analysis.json为摘要，metadata明确raised_chassis_uart_only及实际固件未确认。
本次完整测试会话纳入同模块五次留存；没有删除已有会话或用户文件。

这些结果确认该次实际请求/回执和编码器反馈有效，不能证明地面走了50mm、物理急停时延、
T265距离精度或车身转角精度。架空时轮子转动不等于车身转动，所以未以定角动作等待gyro到角，
也没有把同一gyro或DONE进度当独立真值。

改动为桌面模板、地图服务启用条件、原测试脚本与本文；原检查含桌面Exec参数解析通过，
重开原桌面入口后即可点击测试。正式比赛路径、T265候选、比例及用户文件不改。


## 2026-10-30：沿用7e25a81的新安装外参复核

复核时当前分支为bbf747c。与7e25a81比较，定位投影源码、配置解析器与新安装候选配置
没有后续改动。本轮不重写外参实现，不更改候选值、比例、T265单源/pose jumping或正式比赛。

候选保持：`r_BC=(-0.0346105,-0.0599471445,0)m`，半径69.221mm；
车体前向在T265坐标中为`(0.8660254,0,0.5)`，上向`(0,-1,0)`。
归一化后以forward/left/up为列的`R_CB`为：

```text
 0.866025403  -0.500000002   0
 0            0           -1
 0.500000002   0.866025403   0
```

left=up×forward；三轴正交、右手、det约1。它与原竖装基础上相机自身俯视顺时针30°
的定义一致；不是给地图初始航向加30°。纯T265、pose jumping、比例禁用/值1.000保持。

位置使用`Δp_B=Δp_C-[R_WB(t)-R_WB(0)]r_BC`；
速度使用`v_WB=v_WC-ω_W×R_WB r_BC`，再投影到车体forward/left。
杆臂由完整四元数旋转，保留滚转/俯仰，且只补偿一次。
SDK Pose定位中心沿用给定的测量点，没有再加一套内部IMU偏移。
世界+y重力上向依据[官方T265坐标说明](https://github.com/IntelRealSense/librealsense/blob/v2.50.0/doc/t265.md#sensor-origin-and-coordinate-system)。
[官方姿态预测示例](https://github.com/IntelRealSense/librealsense/blob/v2.50.0/examples/pose-predict/rs-pose-predict.cpp)
将角增量左乘姿态；据此核对当前按世界系角速度计算叉乘的用法。

### 已有原始记录的复核结果

来源仍是RDK原目录`rescue_map/runtime/history/t265_f407_motion_map/`，只读分析，没有改写原文件：

| 会话 | SDK原始帧 | 处理帧 | 车体上向相对SDK重力上向倾角：中位/最大 | 最大杆臂位移修正项 | 最大速度杆臂修正项 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20261030_180152_interactive | 12514 | 12514 | 1.82° / 4.68° | 98.68mm | 0.12553m/s |
| 20261030_180325_interactive | 6191 | 6190 | 1.81° / 41.68° | 56.94mm | 0.09890m/s |

逐帧从SDK原始四元数、位置、速度和角速度独立重算，共对照18704处理帧。
定位中心投影、车中心平移及车体系速度与程序输出的最大算术差均约5×10^-10，
符合CSV保留9位小数的舍入量级。**这只是数据变换一致性，不是实车亚纳米精度或外参标定精度。**
速度修正项不是速度误差，也不能从该数值推出真实速度准确。

第一段tracker confidence为2/3；第二段第一张原始帧为0，后6190张为2。
第一段有4次pose_gap事件，第二段没有记录pose_gap；两段均未记录position_discontinuity事件。
无事件仅表示未触发原诊断阈值，不证明没有漂移或跳变。
第二段最大倾角41.68°需要由操作者说明当时是否倾斜、搬动或出现跟踪异常，不能据此改写安装轴。
重力方向只能支持上向约定，不能单独确认前/左轴的平面旋转符号或30°实物安装角。

两段事件只有启动/停止/日志留存，test_id均为0，无commands.jsonl或ground_truth.jsonl，
reference_description为空。因此前后左右1m往返及正反90/180/360°全部仍为**待实测**，
不能从全程轨迹、指令进度或同一陀螺仪补造动作真值。
当前结论：计算链和给定候选一致；实物方向、杆臂测量、距离/转角精度尚未验收，候选保持不变。

### 修复原有实测记录的终点衔接

发现原M键只写事件，G输入回车时又取实时位姿，分析终点会混入测量/输入期间的漂移。
本轮仅在原地图和分析器增量修正，没有新增独立测试程序：

- 动作结束后，由操作者确认物理停稳，再按M，冻结该test_id的定位快照和主机标记时间；
  拒绝未结束动作、过期/丢失T265及跨动作复用标记。
- 再使用卷尺/地面标线测量并G输入，保存录入时间和已冻结的独立终点时间，
  输入期间的实时漂移不会修改已标记的终点。M只表示操作者声明，不从DONE或gyro推导物理静止。
- 分析用该终点的定位样本；旧数据优先恢复已有M事件，没有M时标注legacy输入时刻回退。
  同时输出原始定位中心与补偿后车中心的动作增量；只有后者与车中心的外部测量比较。
- 原test_f407_motion_protocol.py扩展检查缺M拒绝、冻结后位姿变化不改终点、
  晚录入不混入漂移、旧M事件兼容；检查通过。已有C++几何/速度杆臂检查1/1通过。

### 实测顺序与结论边界

使用当前F405客户端确认过的运动服务，保持原T265候选和比例1.000。
每段开始前，在地面标记车体旋转中心与真实朝向；每段结束后人工确认停稳，
先M冻结，再独立测量/G录入，完成本段记录后才开始下一段。
位移测量沿该段开始时的车体前/左轴；转角用地面角度基准和完整圈数，不能从同一gyro读取真值。

1. V前进1m→B后退1m；再B后退1m→V前进1m，逐段记录，共四段。
2. L左移1m→H右移1m；再H右移1m→L左移1m，逐段记录，共四段。
3. O/1：逆时针90°/顺时针90°；P/2：逆时针180°/顺时针180°；0/3：正反360°，共六段。
4. G格式仍为`前向mm,左向mm,转角deg`，未知填`?`。完整一圈记录+360或-360，不能写成0。
   指令目标和下位机进度仅作为请求/动作误差的对照，不作为独立测量值。

先检查前/左正方向、实际刚体旋转与相机杆臂弧线，再区分：安装旋转错、平移偏置错、
跟踪漂移/跳变和下位机真实动作偏差。转动中车中心的实际平移也须通过地面基准记录，
不能把所有中心残差都拟合成相机偏置。本轮不拟合比例、角度倍率或其他误差补偿，
不以gyro与自身指令一致宣称精度通过；实际完成矩阵及精度结论等待操作者提供测量。


## 2026-10-30：F405 NormalDebug测试接口适配

本节覆盖下文旧F407/MotionDebug控制、轮式参考和启动说明。原桌面入口和
新T265候选外参保留，正式比赛入口、默认正式定位配置和比赛状态机没有改动。
未应用融合补丁，未接雷达、主yaw定位或官方轮速注入，没有执行实车动作。

接口依据固定为下位机固件实现`14157e6bf2600ff0480b133ab763298e12ce400b`，
文档提交`a54ee465bf6ac7c35dc6236c61ee580e71fbcfc0`。
[完整合同](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/a54ee465bf6ac7c35dc6236c61ee580e71fbcfc0/docs/F405_NORMALDEBUG_CONTRACT_20261030.md)
及原始示例JSON的副本保存在`localization/tools/contracts/`。
文档修订号1不是线上协议版本；历史烧录/发送计数不作为当前实机身份。

### 本轮修改与公共接口

- `localization/tools/f405_test_client.py`：共用底盘测试客户端，独占UART；
  提供`move_body(forward_mm,left_mm,speed_mm_s)`、`turn_ccw(angle_deg,rate_deg_s)`、
  `stop()`、`reset_reference(stationary_confirmed=True)`、`telemetry(callback)`。
  订阅返回取消订阅函数；订阅者复用同一个实例，不再打开UART。
- `move_spin_angle(speed_mm_s,direction_deg,turn_deg,max_rate_dps)`与
  `query_capabilities()`明确抛出UnsupportedCapability，不发送字节。
  `capabilities()`只返回冻结源码合同的能力清单，标注来源，不是板端查询结果。
  当前没有版本化实测前/左车速帧、线上boot/reference epoch或可靠大动作ID。
- `f407_motion_protocol.py`保留历史helper兼容，新增F405专用向量/转角编码及0x1F解析。
  地图只调用新客户端。应用前/左/逆时针为正；仅协议编码对旧TURN角度取负，
  转速直接以度/秒传入，取消旧轮缘mm/s与130mm半径的UI转速换算。
  TURN回执进度单独转为应用CCW度，原始gyro不反号。
- `t265_f407_motion_map.py`继续复用原地图、T265进程、人工实测记录与日志留存。
  T265子进程改为无UART并显式ignore-encoders，旧F407轮式公式不再进入测试参考。
  正式localizer代码及正式配置不修改。新文件均为测试路径或文档。

当前UART4支持0x19请求、0x1A状态、0x15原始计数、0x1F真实gyro。
0x1A没有IMU健康位，Location heading=0还可能表示无效；只按实际字段记录。
0x15与0x1F独立SEQ，不按相同SEQ配对；同一0x1F中的yaw/rate也不是同一传感器采样时刻。
无效gyro仍记录，重复有效包不刷新新鲜度；源age加主机接收年龄到150ms后显示过期。
它没有包含UART/滤波未知延迟，不因此宣布具备算法级采样时间资格。

### F405轮式复算与显示

先做16位计数模差，反馈轮位移仅翻转一次：`di=-pi*0.070/1061.268*raw_delta_i`；
`F=(d2-d1)/sqrt(3)`，`L=(2*d3-d1-d2)/3`。三轮顺序右前/左前/后轮。
与旧板相比横向符号相反；没有为此修改T265轴或地图初始航向。

原始计数始终显示/记录；下位机实测前/左速度明确显示“未提供”，不以0、目标或
上位机复算代替。上位机速度用计数差除以主机接收间隔，明确标为估算，
不使用SEQ差乘10ms伪造采样间隔。同一read批次只复算一次，原始字节和逐帧计数仍保留。
重复、明显乱序、半计数范围歧义、reset/非法标志、超过250ms接收间隔会使该估算不可用并重建基准。
零计数变化可是真实静止，不能仅据此宣布冻结。

紫色参考仅用复算增量和最近T265航向画诊断轨迹，时序未标定，不是独立真值，
也不送入T265或地图橙色定位。断流/数据不可用明确显示无效；完整原始记录留在日志。
原localizer CSV仍有历史轮式列，但本轮无UART，所以报告将这些轮式比较值置null，
不能把全零列当作有效车轮轨迹。F405复算值以chassis_telemetry.jsonl为准。

### 动作身份、回执与停止

每个客户端有host_session_id，每动作有主机唯一host_action_id；线上仍仅8位SEQ/opcode。
记录请求/回执的单调接收时间、主机reference epoch，MCU epoch与采样tick均为null。
SEQ短时间避免复用，并避开最近回执/当前动作SEQ；显式STOP优先于普通复用等待。
只关联发送后、SEQ/opcode匹配且状态序号可接受的回执；同一时刻仅一动作。
STOP可取消当前动作，必须等匹配STOPPED才能提交下一动作；不把HOLD当暂停。

超过接收/动作时限会结束主机等待、记录机械状态未知并取消自动后段，
不自动重连、重发、STOP或重新归零。后续运动需人工STOP并等匹配回执；
这是可恢复的动作交接状态，没有新增永久故障锁存。STOPPED/DONE不证明物理已停稳。
RESET按钮/R键的文字明确要求操作者先确认停稳，命令只重建下位机Location参考；
不清原始轮计数、gyro或T265，不是安全区自动分段清零。
旧Y转向返航只有匹配终态且见过本动作RUNNING才续发后段；只有直接DONE时结束等待并取消自动后段。

**缺少线上boot epoch/nonce，以上只能降低误认，无法严格排除重启前或SEQ回绕后的迟到回执。**
日志的correlation与stale_reply_exclusion_guaranteed=false保留这一限制。
后续要补齐的下位机接口见[f405_normaldebug_interface_followup.md](f405_normaldebug_interface_followup.md)，
本轮没有修改下位机或分配新TYPE/opcode。

### 启动及本轮验证

保持原桌面“T265-F407运动定位调试”入口名称；默认只监听、显示和记录。
操作前须由操作者确认实际提交、NormalDebug及SERVICE=MOTIONDEBUG；仅MODE=5不足以区分比赛服务。
当前合同只审核14157e6，其他功能版本需要先核对合同，不能用纯文档a54ee465当固件版本。

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
# 仅当操作者确认现在板上仍是该实现和运动服务时，填写以下实际身份：
localization/tools/desktop_launcher.sh \
  --actual-lower-firmware 14157e6bf2600ff0480b133ab763298e12ce400b \
  --actual-lower-mode NormalDebug --actual-lower-service MOTIONDEBUG \
  --reference-description '填写本次卷尺零点和地面角度基准'
```

不带实际身份确认参数也可监听，但不发运动。没有能力探测包、程序器文本探测或启动运动。
原四向、正反90/180/360°、M停稳标记、G实测录入均保留。
转动默认60°/s，`[`/`]`调节角速度，U/J只调平移速度；S停车，R人工确认停稳后重设参考。
客户端close()本身不发字节；原地图在有过实际发送的会话退出时仍请求一次STOP并记录结果。
停止其他占用同一底盘UART的程序；互斥打开检查拒绝第二个测试客户端。
未来若多验证程序需同时运行，应复用同一owner的订阅，不建立第二个物理串口读取者。

日志仍在RDK原会话目录，最近五次完整会话策略覆盖新增
`chassis_telemetry.jsonl`、`chassis_actions.jsonl`、`imu_telemetry.jsonl`。
保留当前会话、用户文件、模型与标定；本轮没有清理已有日志。

只扩展原`test_f407_motion_protocol.py`：合同hex逐字节、参数边界、F405方向与回绕、
接收间隔与批次合并、0x1F符号/status/重复、单动作与STOP等待、错配/迟到回执、
超时不重发、RESET保留gyro、unsupported无发送，以及本地PTY单owner/收发检查。
原测试脚本、Python语法、入口help、地图无硬件渲染及git diff --check通过。
保留原始UART历史的离线回放解析到0x15/0x1F各3282帧、0x1A 547帧，CRC/序号缺口为0。
这些都是离线/本地PTY检查，没有连接实车UART/T265，没有发送实车动作，
不等于当前固件身份、电气停止时延、定距/定角精度或传感器算法验收。本轮不进入融合算法验证。


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
