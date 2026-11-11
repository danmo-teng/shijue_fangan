# 两轮正式比赛取证（2026-11-11；只读）

基线：两场上位ed07e32，F405实际e85c738/dirty1 NormalRun/TASK/921600。本文分析原始记录及设备自报提交对应的仓库参考源码；dirty1下未证实与烧录字节一致；后续main的mode39相机变更不作为已部署证据。事件/检测帧是抽样日志；Replay仅事件片段，不能补造开局录像或地面真值。

## 会话及时间

|运行|事件目录|配套原视频/传感器|competition_started单调秒|
|---|---|---|---|
|约14:02|history/20261111_141241_competition|history/competition_wheel_input/20261111_140243_competition_wheel|118.801902996|
|约14:12|history/20000101_080032_competition_15|history/competition_wheel_input/20261111_141241_competition_wheel|716.035873096|

下表t+均相对competition_started。第二轮目录年2000由重启未校时引起；detections末行3134空/不完整；UART RX末行同样有1条不完整；没有competition_stopped正常摘要。该目录不能视为无记录或完整会话。

## 主要发现

1. 用户看到“爪内绿蓝停滞”至少有一个直接录像证据：第二轮frame9436/capture1036.045s=t+320.009，绿蓝并排在爪内，model两类都识别到（green_supply0.90/danger_cyan0.96），actual下位mode39 flags6，task38126/action12813。这是扫障抓取后的机械/逆迹链等待，不是普通运输审核把绿蓝混装卡住。t+318.891进入closed39，直到t+348.723人工停止附近才退出。对应PNG在images目录。蓝=labels class2 risk→danger_cyan；黑=class1 kernel→core_black，不可因用户说蓝/黑就合并类别。
2. 真正“看到框但不靠近”的一例：第二轮frame4502/capture869.732s=t+153.697，mode42/pitch120、safe_sweep_target_wait。danger_cyan335 bbox(644,155,136,155)置信0.85/hits8可见；安全区bbox(352,4,926,245)，目标中心(712,232.5)在安全区框内，但脚点(712,310)在框外。原cargo_in_front_region只要中心或脚点任一落safe_bbox就拒绝，导致该贴边目标暂不被选。现场灰框也由front_check的这套几何排除条件绘制，不代表YOLO没检测。不能据此去掉真正区内物的排除。
3. 第一轮core_black长缺框：t+342.924→356.437 TRANSPORT_AUDIT13.51s。frame10261新图黑884 overlap0.805已被原ROI接纳；10262起漏框，之后load_empty_gap及frame0占位、原复看循环；10426黑892 overlap0.736再次接纳。frame10450/capture467.745s/t+348.943真实图里黑三棱锥仍在图左爪边缘，但模型没有黑框。这段有真实模型漏检，不可把mode48flags2的早到140°旧图ROI未开启误当ROI永久排除。用户“车体最右”与图左右的物理映射待核，不凭图左就否定其描述。
4. 两轮开局在上位INITIAL_OBSERVE后都很快开始靠近，不支持“上位在中心无限停”的唯一推断。第一轮t+7.897中心观察→7.969靠近→12.528140°审核→13.547运藏点；第二轮30.495观察→30.570靠近→34.894140°审核→36.062运藏点。第二轮实际UART显示mode1/pitch90/motor active从t+7.140一直到30.423（23.283s），主要长等发生在下位自动出发阶段。更具体地，第二轮t+10s控制XY(−0.1104,+0.0516)、距心122mm，t+15/20/25/30.4仍近(−0.107,+0.053)；10.0→30.4s的2040轮包中2009包三delta全零，三轮总绝对计数仅[0,15,22]。actual Task.c1906/1983/2010的中心末段到达阈80mm、最小车体110mm/s、Motor_IsBraking直接return；APP_START_TIMEOUT_MS=30000时末段直接task_finish_start(false)。30秒退出规则与长等时序吻合，但不能确定不动根因；6个10ms制动周期自身不能解释20秒。最低轮PWM/方向分量及实际输出仍未读回，不擅自扩大到达阈值。没有这段开局视频，不判定舵机物理姿态或场景是否不可见。
5. 两轮上位SEARCH本身通常<2s，多次很快重新发APPROACH；连续“转圈找不到”主要集中在CLEAR内的42搜索，以及39执行/回退。不能只给SEARCH加超时来修这些链。

## CLEAR长段时间线

|场次|t+起终|持续|关键事实|
|---|---|---:|---|
|1|114.026→147.909|33.883s|danger_cyan触发第1次CLEAR；119.905起42；136.944起39回退；最终未找回原物资返中|
|1|466.249→590.226|123.977s|injured_orange触发第1次CLEAR；470.950起42；489.430起39；520.732下位上下文清0转mode0；521.330起mode18/fault1直至结束。后约69s不是39逆迹算法继续执行|
|2|148.748→226.649|77.902s|danger_cyan触发第1次；153.809起42；163.10243/reject12762，166.65842/reject12763，168.46843/reject12764；168.89439开始抓，169.122closed39；206.501open39，209.57444找原物资；214.68039/status5/reject12767；最终失败返中|
|2|309.597→349.762|40.165s|danger_cyan触发第1次；314.67842；318.28843/reject12811；318.61839抓；318.891closed39；348.723下位上下文清0，349.309mode18/fault1|

这里的第1次是上位本趟扫障次数，内部可能反复42↔43；不要误写成上位重复新建无限N次CLEAR。

## 39真实反馈（不是独立地面位移）

|窗口t+|wheel数量|轮平移速度中位/P95|轮累计path/车前向积分|主yaw表现|
|---|---:|---|---|---|
|第一轮489.430→520.700|3128|13.8/60.2mm/s|0.589m/−0.251m|净−886.4°，多次旋转纠偏|
|第二轮169.122→206.501|3738|见mechanical_motion.json|见派生JSON|169后先低速/静止，之后多次旋转，再松爪|
|第二轮318.891→348.723|2983|0/63.3mm/s|0.376m/−0.300m|净−0.181°，但中途10.7→358→38.4→10.5°来回调整|
|第二轮343.891→348.723|483|0/0mm/s|0.00124m/−0.00108m|+0.22°，98.14%轮平移<2mm/s；UART仍mode39 flags6，是真停滞|

这些窗口控制位姿来源均T265_PHASE_NO_WHEEL，SDK既往轮输入影响不因此立刻清除。可见真实轮反馈、主yaw及控制位姿并非恒定，但没有独立测量，不能认定控制XY跨度就是车体真实横移。

## 内部trace不可观测边界

actual e85 Task.c的task_sweep_reverse_trace确实存在remaining负仍以最小20mm/s倒行、cross门控完成等潜在几何问题。原始A3/B3帧只见0x15、0x1F、0x1E、0x17；0x17只含mode/flags/pitch/ack/fault，0x1E只含task/action/opcode/status/reject。第二轮CRC有效数量分别36484、36424、6081、6081；没有trace phase/target/remaining/cross、没有TSTAT。不能从现有日志证明某时刻remaining<0且cross超差，也不能唯一指认39卡在哪个子相位。源码数值演算可以证明候选边界，不能冒称是现场数值。

## 派生文件

- *_timeline.txt：原事件的完整非capture摘要时间线。
- *_audit_rois.json：审核ROI判定与最近抽样真实检测框；near_frame不等于同帧时已显式记录。
- *_status.jsonl / *_status_segments.json：原始0x17/0x1E CRC解析结果。
- *_mechanical_motion.json：39窗口总统计及5s分桶，速度来自实测type7，yaw/rate来自type8，controlXY来自控制发布记录。
- images/*.png：事件AVI按video_frame_index原帧提取，未重画框/伪造场景。

没有修改源码、配置、原始日志；未开设备/电机、未运行清理、未修改F405或烧录。

图表：clear_mechanical_evidence.png对三段CLEAR分别绘制控制XY、主yaw累计变化、真实轮车体系速度；不是地面轨迹精度证明。
