# 一小时MotionDebug：延迟、力臂、轮速停送对照

本轮场地采集已于2026-11-09完成，实际结果与下一步见[MOTIONDEBUG_FIELD_CAPTURE_20261109.md](MOTIONDEBUG_FIELD_CAPTURE_20261109.md)。以下为当时准备/操作说明，不是重放动作授权；正式比赛入口、去畸变来源、100Hz传播基础和判滑动作均未改。

## 要回答的四个问题

1. 相同运动下，纯T265 F与官方轮速输入A的角度/角速度、线速度响应有何差异？分别列主机传输/排队与运动信号相对响应，不把全部差值叫SDK延迟。
2. 三轮计数与独立主gyro的累计转动，是否支持当前130mm有效平均旋转力臂？正反方向/速度是否一致？独立留出动作检验，不能把滑移拟合成新L。
3. 已启用输入后，运动期间**真停发**会不会表现得像零速、末值保持？错误已经进入后停发，能否减少后续影响？
4. 哪些数据可用于以后评估T265去畸变/100Hz传播基础，哪些仍缺独立位置参考？本轮只提供证据，不切换实现。

## 一小时安排及故障隔离

计时从用户确认“MotionDebug已烧录、地面就绪、在旁看车”开始；本文件是候选动作安排，不是自动发车授权。实际动作逐组由同一个入口发出。

供电转换先做：保存并关闭桌面采集→代码提交→用户换车载电池/RDK重启→重连后读实际身份及USB状态→用户确认地面和人在旁→开始一小时。桌面预热不冒充现场就绪，不要求重复烧录；当前会话183601只是未运动的准备检查，不能混入实车F组。

重启后先检查`rdk-rescue-buttons.desktop`自动启动的正式按键/预热进程是否占设备。仅关闭本次测试中冲突的实际进程，保留正式配置和自启文件，不让正式预热与测试同时持有SDK/UART；不按正式3号键开赛。再启动下文唯一测试入口。若T265卡启动，结束失败SDK owner后转chassis-only，不能同时启动两个UART程序。

| 预算 | 项目 | 最小产出 | 某项失败怎么办 |
| --- | --- | --- | --- |
| 0～8分钟 | actual身份、轮/gyro收数；原地转动 | 实际固件/服务/波特率、三轮计数、主gyro；先判断符号/动作是否正常 | T265 USB出问题则直接继续无SDK的底盘组，不耗整场排查USB |
| 8～18分钟 | F全程无注入 | 正反转动及短直行/横移，原SDK Pose/velocity | 若无Pose，保留失败日志，完成轮/gyro/L组；不伪造F组成功 |
| 18～28分钟 | A正常官方输入，新SDK会话 | 同动作响应、真实API最终向量/时间/返回、中心闭合 | 若接口失败，F及轮/gyro成果仍可分析；不改阈值放行假输入 |
| 28～48分钟 | 优先B真停发与C定量偏置；再错误→停发→恢复 | 同一路径、已知窗口、运动中断输入/实际停车两工况 | 单组失败记录原因，继续不依赖它的组，不重复追一个问题 |
| 48～56分钟 | 有余量做D显式零速、E重复末值及必要复测 | 区分停发/零/末值；主条件有余量做3次 | 不强求首轮完成所有组×方向×速度×3；未重复的只称初筛 |
| 56～60分钟 | 停车、人工端点测量、日志完整性核对、收尾 | 原始证据、缺项及最初步结果 | 不把收尾时间用完后继续扩矩阵 |

准备阶段尽量完成编译、假接口检查、日志与控制入口验证。现场只需要摆车/观察、必要的卷尺/地面角度记录和用户STOP。GUI不参与高频轮速输入，测试不用T265自动生成返航距离。

## 分组定义：沿用transfer，避免混名

| 名称 | 实际SDK行为 | 会话/用途 |
| --- | --- | --- |
| F | 全程不加载/不使用wheel输入组件 | 独立SDK会话，优先采集，记录设备是否真正重启；不能等同B |
| A | 实测07＋08换算到C点/C轴，正常调用API | 独立SDK会话；所有实验组之外的基准 |
| B / drop | 窗口内不调用`send_wheel_odometry` | 不发零、不重放，消耗原实测样本；恢复仅发送当前新样本 |
| C / bias_body | 只在发送副本中叠加明确B轴偏置，经当前R_CB变到C轴 | 先用前向+0.20m/s、0.5s作为初筛候选；不是更改电机速度或真实反馈 |
| D / zero_camera | 每个新实测样本对应一次调用，最终C三分量全0 | 真正的API零速；旋转时仅把B平移设0仍有杆臂项，不能当D |
| E / hold_last_camera | 对新实测样本重复窗口前最后一次实际提交的float32 C速度 | 明确标记实验末值重复；匀速时可能恰好仍正确，必须观察停车/变速 |

0.5s是实验窗口，不是推断固件超时。B/C/D/E每组使用独立SDK会话优先，至少记录实际session；已输入错误后新建pipeline也不声称设备状态完全清空。F优先于错误输入，错误组不作为随后纯VIO精度基线。必要的真实USB重启由用户实际操作并记录，不能每组一律安排插拔拖慢现场。

输入关系保持：`v_C^C=R_CB(v_B^B+omega_B^B×r_BC^B)`；SDK T/W=0。噪声0.1m/s是原工程初值，外参和比例不变。实验只变最终SDK输入副本，原始轮/gyro、电机目标和实际反馈全部保留；收到的真实反馈不以年龄/未知时间质量拒绝。

## 候选动作：先短、再重复，不一次连续转很多圈

现场确认可用空间后，原地单次最多±360°，复用已实现TURN，不编造多圈opcode。初筛可用：+360°@60°/s、−360°@60°/s、+360°@120°/s、−360°@120°/s，再留一对±180°作交叉检查。动作目标不是真实转角；L使用同区间gyro累计角作旋转参考，并由地面标线核对实际整圈/方向。

短平移候选为前/后各500mm、左/右各300mm，先200mm/s，空间与底盘正常后再400mm/s。所有距离以用户现场确认范围为准，不用T265闭环控制测试路线。预留车轮打滑和制动距离，不把DONE当卷尺真值。

输入窗口至少覆盖两种独立现象：

- **仍在运动时停送**：最后输入非零，随后车还在走；检查T265是否少算/趋零。
- **窗口内真正停车**：最后输入非零，而车身随后停稳；检查额外位移及末值残留。

第三种横移/变速在前两种可重复后补。正常→短偏置→停送→恢复作为额外序列，不能拿纯B组代替已被错误影响后的结果。具体窗口按实际动作反馈安排并记录实际调用边界，不能只记Python“已发设置”。

## 判读与测量边界

- 轮07是区间均值，gyro08的yaw/rate各有自己的源时刻，先对齐同MCU区间；主机收到时刻用于主机链路对照。下位机已上传实测角速度，不再对角度粗差分代替它。
- SDK原始Pose/velocity按真实frame_number、采集时间域和receive时刻记录。跨时钟域的映射假设/不确定性必须写明；时间资格未知不变成运行门。
- 匀速平台的相关性无法唯一辨认时延，主要用起停/变速边沿与多次动作核对；保留正负lag和相关峰形，低激励时报告不可识别，不能强行给一个“精准毫秒”。
- gyro、T265内部惯性、轮反馈和注入后的T265有依赖关系。T265/轮一致不能当两份独立平移真值；主gyro用于角运动参考不等于地面绝对角度证书。
- L拟合只在接地/平地且动作正常的样本上给候选及方向/速度一致性；若CW/CCW明显不一致，先保留当前参数，不追求平均后好看。实际不等臂/轮向误差也可能导致残差。
- 真值优先卷尺、地面角度和实际停稳标记；若有围栏只评价可观测方向。无独立连续位置参考时，只报告响应/端点/跳变，不能给窗口内“最大真实位置误差”。
- 接口返回true只表示API返回。真停发没有逐样本API结果；需记录skip窗口和成功调用计数不增长，不能把静止Pose连续当运动精度通过。

## 软件准备与操作入口

已复用`t265_f407_motion_map.py`增加显式`--motion-lab-stdio`：启动仅采集，逐条JSON才触发动作；不另开UART/SDK。`--motion-lab-chassis-only`允许同一客户端独占UART完成轮/gyro/L组，不依赖T265启动。原测试地图/按键行为保留。

下面三条入口**顺序使用**，不能同时运行。正常情况下先F，再A/其它输入组；USB失败用底盘组。

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
# F：无注入，纯T265；启动本身不动车。
python3 -u localization/tools/t265_f407_motion_map.py \
  --motion-lab-stdio --wheel-input off --timed-measurements \
  --baud 921600 --chassis-profile timed --reference-description '车中心/车头地面标线，卷尺独立测量'

# A/受扰组：正常实测输入启动，收到显式experiment命令才改变输入副本。
python3 -u localization/tools/t265_f407_motion_map.py \
  --motion-lab-stdio --wheel-input on --wheel-experiment --timed-measurements \
  --baud 921600 --chassis-profile timed --reference-description '同一地面基准；按事件区分实验组'

# 无SDK独立轮/gyro组：USB失败仍可做，不启动T265。
python3 -u localization/tools/t265_f407_motion_map.py \
  --motion-lab-stdio --motion-lab-chassis-only --baud 921600 --chassis-profile timed \
  --reference-description '车中心/车头地面标线，实际整圈数及终点角度'
```

stdin每行一个JSON。**下列动作只在用户确认地面且人在旁后发送**；本文件不是待自动重放脚本。`submitted`仅指请求发出，`status`另看匹配回执，不等于真实距离/停稳。

```json
{"op":"status"}
{"op":"event","name":"physical_group","details":{"group":"F","reference":"地面车头线"}}
{"op":"turn","angle_deg":90,"rate_deg_s":60}
{"op":"turn","angle_deg":360,"rate_deg_s":60}
{"op":"move","forward_mm":500,"left_mm":0,"speed_mm_s":200}
{"op":"experiment","phase_id":"B_drop_1","mode":"drop","duration_ns":500000000}
{"op":"experiment","phase_id":"C_bias_1","mode":"bias_body","bias_body_mps":[0.2,0,0],"duration_ns":500000000}
{"op":"experiment","phase_id":"D_zero_1","mode":"zero_camera","duration_ns":500000000}
{"op":"experiment","phase_id":"E_hold_1","mode":"hold_last_camera","duration_ns":500000000}
{"op":"experiment","phase_id":"normal_restore","mode":"normal","duration_ns":0}
{"op":"mark"}
{"op":"ground_truth","text":"?,?,360"}
{"op":"stop"}
{"op":"close"}
```

前一动作结束后才可下一动作；操作者实际确认停稳才做mark，ground_truth可用`?`保留未知项。窗口默认收到后按当前新实测样本应用，可指定`start_monotonic_ns`；单个活动profile，新的替换旧的，不是队列。stop始终是明确的底盘STOP，close仅在本会话曾发动作时沿用原收尾STOP；纯采集退出不发STOP。

## 离线快速判读与已完成准备

```bash
python3 localization/tools/t265_f407_debug.py \
  --motion-response-analyze /完整会话目录 \
  --motion-response-output /原会话外的新输出目录
```

输出`motion_response.json`及`motion_response_signals.csv`，保留原SDK/轮/gyro、未经角速度IIR的完整姿态投影与中心速度、实际API调用及全部实验事件。平移及角速度用同宽窗口在主机接收时钟上估计带符号相对响应；低激励/多epoch不强行给数。独立L标定仍按明确动作及独立实测整圈角度计算，不自动猜边界。当前输出不直接回写任何参数或运行门控。

已完成：C++缓存构建与现有目标中的五态fake-sink检查，CLI默认/正式隔离检查，原Python检查文件中的stdio/纯离线变换与合成+40ms检查。桌面183601真实无注入Pose正常、未发动作，完整收尾；184039在同一真实SDK owner内验证五态API/日志路径，无电机命令，不能据此评价运动精度或停发优劣。准备回执见`reports/motion_lab_preflight_20261109.json`。

没有运行全套回归。额外尝试的旧`test_timed_v13_feedback`在其原DynamicMapApp.drain_yaw期望处失败：fixture仍期待父进程解析/发送yaw，当前HEAD已经转发feedback_context；该运行代码不在本轮修改内，未扩大任务去改它。上述新stdio、C++实验及纯离线检查分别通过，不能把它写成全仓检查全绿。

实板自报`ac61480cf44e24c314988f2be4ff3996d9b1ee93/dirty1`、MotionDebug、921600。GitHub相关diff已核到ac61480，MotionDebug/07/08及编码器/gyro公式与此前兼容，实板dirty仍不能凭远端证明。重启后再次记录actual，不能把本次桌面身份当现场永久有效。

官方依据：[librealsense 2.50输入实现](https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.50.0/src/tm2/tm-device.cpp)把时间字段置0由固件接收时填写，frame number是身份，API true不是融合ACK；[ROS官方实现](https://raw.githubusercontent.com/IntelRealSense/realsense-ros/2.3.2/realsense2_camera/src/t265_realsense_node.cpp)没有消息时不补零/末值，但未提供停发后旧状态多久消退的保证。因此必须测运动响应，不能根据主机停调用推断固件已经清空影响。
