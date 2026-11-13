# 两端最新修改要求

更新：2026-11-13。只维护当前相关请求与回执，历史按Git取证，不自动恢复。
发布或读取不等于接收方已审核/实施；先核对实际代码，向人说明接受、调整或拒绝的理由。
重要范围变化由人决定，已批准范围不重复审批。规则见AGENTS.md和CHANGE_WORKFLOW.md。

## 固定位置

```text
RDK：danmo-teng/shijue_fangan，codex/gamepad-teleop
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md
F405：kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot，main
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md
```

双方固定入口已建立，RDK连接仍只读F405；两端更新自己仓库并读取对方最新要求，不要求用户常规手动转交。
这不是后台执行服务，不能把本端发布说成对端已收到/实施，更不在此窗口修改或烧录下位机。

## 当前请求MATCH-BULLDOZER-20261113：新扫障分段执行，待F405审核实现

人已选择上位编排、下位分段执行；扫障点X/Y各±3cm。同区分支最新明确为：右转90°前进200mm放货，
后退200mm再左转90°，正常抓取/整爪审核/投送门前同区目标，然后找回原载荷。首次任何障碍均先放货走扫障。
异类/危险分支：放货→先后退到转后车后围栏94cm→左移到左围栏54cm（均车中心）→全开前进1m→抬爪→先后退再右移返原预备点→找回。
全开可扫贴栏是本轮实物确认，不恢复旧23cm限制；找回一轮无目标回普通SEARCH，不无限追旧ID、不嵌套寄存。

RDK已实现分流/几何/同动作H/D、寄存与正常抓取衔接、原货返回及日志；新功能暂默认关闭，待配套后启用。
对端当前vision.c最高只接受0x13，旧宏不支持新路线；本轮只读Task/vision/mechanism接口，未改或烧录F405。
请求新增0x18内部opcode0x14的13种**分段动作**，复用原0x1D/0x1E和mode25/3，不加握手/时间门/遥测/故障或额外测试程序。
完整人可读任务（含对端实际源码blob、字段表、入口/完成/幂等、放货后审核清理与原恢复边界）及机器合同：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_BULLDOZER_20261113.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/contracts/f405_bulldozer_step_20261113.json
```

请接收方先核对实际代码，讲清接受、调整或拒绝的理由；接口编号/字段若要调整先回传，不能单端改。
本轮用户批准目标/分工不代表对端已审核、已部署；两端各自commit/push并回传actual/构建/烧录边界，人保留知情、拒绝、选择权。
仅复用必要构建和已有检查，不要求新增下位测试代码/程序。原mode51/人工STOP、YOLO、正常整爪审核/任务配对不被新流程覆盖。
本端未启设备或实车，软件检查不当作±3cm/1m及贴栏扫除实车通过；下方旧“本轮无新固件任务”仅指各自已交付项。

## MATCH-SAFE-BOX-20261113：RDK后续失框复看已修，无新下位任务

用户已批准修复锁框后再次失框。基于25c48ac增量：视觉横移及已锁框的入口检查/阻挡等待，
失框时清旧横差，同task/action的0x04视觉形式请求115°，重新识别后继续，不退回初始定位形式。
复看请求arg_a=0、arg_b=11500、aux=0、vision_frame=0仅表示零平移/无新测量；
RDK不会把此时的COMPLETED当对准成功。复用原新视图/3帧识别，再发送实际横差和真实帧号。
修复runner原已记录却漏写入0x1D的ALIGN source_vision_frame；原协议布局及帧号排序规则不变。
事件接原异步视频回放；没有新增握手/年龄门/故障或额外延时，也未改正在进行的本地直推。

本轮已读对端main固定入口（blob c05ba53bfdf7e34528594fcf84e79f3be40e17e5）；其ALIGN帧号遗漏提示已按上位实际代码核实并在本任务内修复，
RETURN配对建议是另一个排队任务，本轮不自动实施。接口核查基于最近实跑实际身份6c2ce2d的Task.c
（blob 6bb899d1566358ce071b7fc775ae16247cff550c）：视觉ALIGN同动作支持115°和frame0，flags不变；无需新增下位代码/协议/测试程序。
dirty1说明实跑仍可能有未提交差异，不把远端源码或expected当实际完整固件快照；本轮没有主动开UART或烧录。
7项必要检查、原12421帧离线旧PAUSE→新复看对照及回放编码回读通过，实际抬镜/重新识别尚待实车。
下位无需为本请求添加保护/握手/时间限制；115°仍看不到时继续持货观察，不承诺必恢复。
实现及原日志证据：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_SAFE_ZONE_REOBSERVE_20261113.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/safe_zone_stall_20261113_161039/README.md
```

## 保留交付：双方安全区候选过滤与识别窗置前（无新固件任务）

用户要求排除对方安全区内部物块；RDK现以当前全部红/蓝框过滤搜索/接近候选，沿用原面积/脚点规则。
本方框独立用于对准/过道和投送确认；对方物块不算己方送达，已夹载荷仍完整爪内审核，不借过滤漏掉蓝/超量。
定位窗口增加“识别窗口”按钮，恢复并置前已有画面，不重启或另启摄像头/SDK/底盘进程。
6项相关逻辑检查及静态桌面恢复置前验证通过，未实车；原0x04/0x05/0x09/0x12和task/action保持。
本轮F405无需新增代码、协议、握手或测试程序，仍只读；新绕行/清障未实施。

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_ZONE_FILTER_AND_VISION_WINDOW_20261113.md
```

## 保留交付：固定爪框矩形过道（F405无配套改动）

用户明确批准借现有爪框轮廓作固定图像模板，不按115°/120°重新确定爪框；靠车端选择两侧切点靠下的y727。
RDK从加载模板推导本次x201～1067、near727，far取放置区色框下沿，物资/伤员共用恒宽矩形。
修正完成前不形成过道候选，旧像素障碍自持/重新添回取消；载荷身份/类别政策/整爪审核保持。
12项必要软件检查及原9031帧几何核对通过，未实车、未开设备或发动作；不将图像模板说成物理碰撞标定。
沿用原0x04/0x05/0x12及task/action、下位锁航向本地直推；本轮不要求F405改代码、协议、速度或新增握手。
旧清障仍停用，矩形内真实阻挡仍保留载荷，新绕行/清障尚未接入。

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_CLAW_RECTANGLE_20261113.md
```

## 保留交付：P0完整爪内审核、停用旧清障（无新下位固件任务）

用户当前要求已从“先分析”改为先修锁定单个目标后漏做整爪审核/分离，并明确直接停用旧清障。
RDK已移除sweep_capture仅保留锁定ID的过滤，所有审核计算人工ROI内真实完整类别/数量；
普通/聚堆接近后复用原140°整爪审核，再走合法GRAB或原分离/危险物释放，不能把单目标锁定当成整爪合法。
正式RDK不再生成0x13旧清障，旧发起/执行/Planner恢复出口均停用；该编号仅保留历史识别，不要求下位删除协议。
当前无新的取障/放货/找回原货物动作请求。0x09/0x12/原分离释放、task/action布局不变，本轮无需新增下位功能或握手。
10项必要检查及原帧同ROI计数核对通过，尚未实车，未写/烧录F405。

真实爪外障碍仍阻挡时沿用原阻挡状态保留载荷，绕行/新清障未实现，不能称有障碍也能完整比赛。
用户要求其他问题在此P0后先分析排优先级；新路径/动作、同侧蓝绿保绿及返中误交接不得由对端AI自动实施。
完整交付和边界：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_WHOLE_CLAW_AUDIT_20261113.md
```

## 最近实跑与对端回执（保留实际/expected边界）

最近实跑身份以上方161039配对为准；此前134947为上位a88b031、下位13b5cc7/dirty1，不覆盖最新回包。
此前已读下位安全区连续回正实施回执，不恢复旧c7a9e7a待实施请求；对端main后续调参/烧录须另按本次实际身份核对，
不把旧“未烧录/用户要求”当此刻状态或在本窗口自动调参的授权。
原清障漏审已按此前范围修复，蓝绿机械分流和RETURN误交接仍排队，旧证据见reports/competition_flow_20261113/README.md。

## 上轮批准配套：MATCH-SAFE-VISUAL-20261112（对端后续已有实施回执，下方旧基线供追溯）

用户本轮明确要求实施三条，不重复审批目标：

1. 先原地垂直对正，再横移。
2. 横移中明显偏航，先停止平移、原地回正，再继续同一横移动作。
3. 横差与航向同时达标才完成，修掉像素达标就完成的旁路。

保留现有安全区框/1/3、2/3算法，边界提取/四边形暂缓；RDK窗口仍只读F405，不代改/烧录固件。
只读比较main与c7a9e7a为identical；不把dirty实跑内容当作已全部审完。
第一趟初始yaw约91°、横移中偏到105°，所以修复不能只限开始前对正。
Task.c:3593横移和3602 done早退均跳过后方主yaw判断；5829又仅按像素设置safe_align_done，完成可绕过yaw。

**本轮卡料讨论的重要边界（先审，不机械实施）：**严格先回正也可能因单轮不跟随而停滞，不能称已解决机械卡料。
现普通堵转/反转停机已关闭，但原turn_to→Motor_TurnAngle仍有10秒超时→TURN_FAULT→task_stop路径。
因此不能机械前移旧turn_to；在同ALIGN内优先复用已有连续主yaw控向，不引入新的定角超时停任务。
若现有API不能同时表达零平移/主动回正，先回传最小改法与影响；不能只置零速度然后被动等待yaw变化。
Fine＋Heading可作为有限阻力补偿候选，硬卡可能需退让，但均未在本次讨论获准实施；
不全局提PWM、不新增检测/故障，不擅自改原ENTER；纵向退让后不能直接沿原670mm投送。
新说明、运动学例子及图：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/safe_zone_terminal_20261112/BLOCKED_WHEEL_AND_PILE.md
```

最小配套是先处理法向/回正，再横移或完成；明显偏航时只回正，同task/action恢复横移，不重开相机/载荷审核。
复用原3°到位/6°保持：初次/回正后≤3°再横移，横移中>6°暂停平移回正；
最终done/COMPLETED需yaw≤3°及原64/32px滞回到位，像素已到但yaw未到时只回正。
收包不能单独宣布完成；mode10/11、0x1E COMPLETED和ENTER准入一致，新偏航/大横差清旧完成。
本次只涉及Task.c安全区ALIGN顺序/完成与同动作衔接；不要求Fine/PWM/PID、STAGE或其他导航改动。
ENTER沿统一ALIGN完成条件准入，原直推距离/航向/控制方式不改。
上位暂不叠加新的yaw握手/等待，也不改几何。无新增协议/ISR/遥测/测试程序、年龄门、固定等待或永久故障。
接收方先审实际代码，可提出缩小/替代/拒绝具体实现并说明理由；超范围重要取舍由人决定。
必要检查复用现有构建/夹具及原入口，不要求新增测试代码/程序；回传实际diff、结果及构建/烧录身份。
用户已批准目标不等于对端已实施或烧录，不能把API/mock或上位检查说成实际控向通过。

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/SAFE_ZONE_YAW_FIRST_20261112.md
```

## 本轮上位已实现：MATCH-PILE-ALIGN-20261112（无需新增下位接口）

用户批准接近时先锁一个物资，对准/测距始终跟同一锚点，不再追物资堆联合框中心。
RDK state_machine.py已修改：CLUSTER_TARGET像素X/Y取锚点当前中心；mode52测距用同一查找/重关联的track_id。
邻居及联合框仍保留作后续聚堆/分离上下文，不改变方向点；已发定距命令保持，类别/数量/ROI规则不变。
0x18/opcode09、CLUSTER_TARGET bit5、0x1D task/action及mode52/53布局不变，现F405消费X/Y可兼容。
不要求下位重写聚堆控制、新增握手/资格检查；聚堆近中心减旋候选未纳入本轮。
两项新增回归旧代码失败，修后加四项已有直接检查共6项通过，未实车；下次重开正式入口加载。

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_ANCHOR_FIX_20261112.md
```

## 相关证据（不自动恢复下方旧候选）

续查补充：RDK用原帧9731/9758/9781复现围栏外橙色802被safe bbox标成区内，目标选择返回None；
这是独立未完成项；原真实前沿/四边形建议当前暂缓，不作为本次F405任务。
原MATCH-PILE-ALIGN证据：首绿7次进入mode52、6次回20；旧上位动态邻居联合框中心与单锚点测距不一致。
本轮上位已按上述同锚点修改；下位普通目标近中心减旋本已存在，聚堆排除该优化，属于未纳入本轮的候选。
不把本轮锚点修复当作所有摆头/流程等待已经根治，也不增加时间门、等待、握手、测试程序/遥测或ISR。
详细原帧、实际状态交接与抽样重算边界：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/safe_zone_terminal_20261112/FOLLOWUP.md
```

最新运行已配对：业务20000101_080033_competition_04，传感器20261112_165835_competition_wheel；
上位42d668b，下位实际回包c7a9e7a/dirty1、NormalRun/TASK、921600。已读c7a9e7a源码，运行文件仍0d52ef8；
dirty内容未知。下位最新固定入口认可800ea44，后续提交只改文档，没有新的驱动修复，不将expected当actual。

第一趟74.369s预备点结束时X=-0.161/Y=0.495m，新旧容差均已到位；随后0x04视觉横移
控制位置估计先右18.9cm再左25.4cm，轮反馈方向支持。第二趟141.619—176.141s同一视觉动作中，
主yaw范围75—106°、上位像素偏差正负切换，已见非零指令与实际往返；不是0x03在到位后重开。
另有142.50—151.95s三轮全零85.5%，回放41帧中29帧仍为非零横移，不能全归上位PAUSE；
298.64—300.46s的12帧全部非零横移，三轮全零81.3%。用户说明部分时刻底盘卡物块，驱动力需求更高。

RDK可确定的问题：安全区右裁切框仍按可见宽度1/3重算目标，车头摆动也被解释成横差；
F405可核对的差异：视觉Task调用MoveAngleHeading→普通MoveAngle(fine=false)，
STAGE末端则MoveAngleFine，视觉未继承Fine的实测速差补偿；主yaw保持在本轮效果不足。
没有实际PWM/内部控向目标，不能唯一断定PI、静摩擦或卡料；不能全局提高PWM后称已定位根因。

此前的分区几何与Fine补偿候选不并入本次；当前只审上方yaw优先/完成语义范围。
不要求新增测试程序/遥测/ISR/握手/时间门/停车。
不要直接回退已批准的STAGE容差或改ENTER，接收方可提出缩小/替代/拒绝，重要方案由人决定。
完整原始目录、569帧回放索引、统计、曲线和选帧：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/safe_zone_terminal_20261112/README.md
```

新扫障“推土机”仅登记TODO，旧C有限取障/成对寄存候选不采用：放货、左爪最大、统一朝右推，
爪越内壁但轮不入区；外缘到对称轴23/15cm。8cm名义几何窗口不是已批准导航距离或下位实施任务。

## 已有交付：MATCH-RETURN-20261112 / MATCH-TERMINAL-DRIVE-20261112

已读取F405文档回复提交bbb719c4d3d9035cc1905efbb782b2aace4180e2，较0d52ef8仅改三份文档；
没有末端驱动固件改动。固定入口blob 19cdd37c72b3dbf2e6009874282b26adfa9cda00，完整答复blob
66a229d14bde5ef6ca6112c40e76168dd8f9fddb。再次核对实际运行源码仍为0d52ef8对应内容：Task.c blob
dff0959ae3dcb1fe4cb7c2ab1715674ecf5853d1、app_config.h blob
6a7ee6c6502e5ac28cd3f43a31ac1aca570b5774、motor.c blob
d0717d7ad017f64adae20fa332c7c75664dd564b。此次未打开UART读actual；源码不是烧录证明。
用户本轮已批准上位区域到位方案。下方为本轮实现与接口回执，不要求F405新增功能或修改驱动。

```text
完整下位答复（固定提交）：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/bbb719c4d3d9035cc1905efbb782b2aace4180e2/docs/F405_TERMINAL_DRIVE_REPLY_20261112.md
```

```text
MATCH-RETURN-20261112：上位已实现，告知接口行为，不要求下位新功能。

只在每次正常投送完成后，改原RETURN_CENTER的目标：
红方(0,+0.30)m、蓝方(0,-0.30)m，均为中心沿己方安全区法向偏30cm。
距目标≤0.15m就锁定发送D=0；到位后的定位波动不恢复非零细调。
保留0x08名称/布局/flags、实时H/D、原0x1D task/action、mode16本地退出和mode17→SEARCH。
F405现有返程消费H/D，未自行写死中心；收到D=0沿原完成即可，不新增专门返程协议。
开局藏点释放、抓取放弃和搜索失败等原中心返程未更换目标/8cm容差。

MATCH-TERMINAL-DRIVE-20261112：已按用户批准在上位实现；下位驱动候选未实施。

认可的源码结论：
- 普通RETURN末端120—200mm/s用MoveAngle，STAGE为110—180mm/s用MoveAngleFine。
  STAGE已有0.5mm/s轮零带及最多150‰实测速差补偿，不能把“换成Fine”当STAGE修复。
- 6mm/s死区不能独自吞掉整个120mm/s非零平移向量；静摩擦、反复换向或上游输出归零尚未唯一定位。
  API接受、motors_active及软件PWM命令不证明真实轮运动；本次没有补采现场输出。
- 原返程30cm偏置/15cm锁D=0已可兼容，不需再放宽下位全局80mm。

上位已实现：
1. 仅给STAGE预备点独立横向到位范围：X±100mm，纵向仍Y±50mm。
   使用独立safe_zone_staging_lateral_tolerance_m，不修改通用fence_lateral_tolerance_m，不波及藏点等路径。
2. 原几何距离≤180mm时只修超出上述范围的轴，已在范围内的轴分量为0；粗段保持原H/D。
   180mm与当前F405 APP_NAV_STAGE_FINE_ENTRY_MM一致，控制的是空间阶段，不是新增时间/年龄检查。
   末端D为仍需修正的分量合成长度、H为其场地方向；车头航向仍由F405原gyro保持。
   同task/action继续实时H/D，只有两轴都到位才锁D=0，复用原完成/载荷复审交接。
   例如横差90/纵差40mm可直接到位；横差90/纵差70mm只沿纵向修正，不同时横移90mm。
   正反方向及红蓝两侧均按同一规则；数例已由现有状态机检查覆盖，不是实测或现场根因证明。
3. 后续仍由主gyro对正法向，视觉仅大偏差才横移，再走原ENTER。
   纵向暂不放宽：F405从ENTER本地原点执行固定670mm，视觉横移不补起推距离；不能把所有到位范围一起放大。

改动减少没有业务必要的横向追点，不提速、不新增等待/握手；代价是转入视觉时横向误差可能更大，
需要原视觉横移完成，仍可能把部分定位微调时间转为视觉调整。100mm是工程初值，未实车标定。
它不保证消除纵向容差外的电机停滞；该部分仍需按同段已有TARGET/PWM/ACTUAL/RAW32及H/D、动作身份区分。

下位只改RETURN/FINE为现有MoveAngleFine的建议保留为第二步候选：若15cm到位外仍停，再决定是否采用。
它不解决已用Fine的STAGE，也不取消换向流程；可能增大瞬态PWM及短距过冲，不在此窗口自动要求实施。
不要求下位新增测试代码/程序、遥测或ISR；本轮不新增新鲜度/资格等待、反复握手、延时或保护停车。
```

本轮仅改competition_rescue/state_machine.py、原tests/test_state_machine.py及本端当前状态/待办/固定入口。
上位原0x03 STAGE、0x1D布局/flags和动作配对不变，下位收到原D=0沿原完成即可，无需新增协议或固件改动。
下位普通RETURN改Fine仍未选用，不把这次上位交付当成该固件候选已获授权。

必要检查：旧代码先复现区域到位和单轴指向两项不符，修后8项直接相关检查通过；
覆盖红蓝/物资及伤员、正负X/Y、只修超限轴/双轴向量、18cm外粗段、D=0交接及抖动保持、藏点/原返程、
视觉大偏差及载荷平台复审。补充藏点横差90mm仍须导航的原检查通过；语法/diff通过。
本次未开串口/设备、启动车轮、重启正式程序或清日志；真实等待时间、起动及投送效果仍待原入口实车验证。
此前6edf4a0交付时确认旧test_return_stash_requires_confirmed_cargo_and_search_handoff在c907be0及当次修改后均因
NO_CANDIDATE而非stash_return_start_gate失败，已记录，本轮不重跑或借STAGE改动顺手改其藏点搜索业务。

## 前次请求：MATCH-DELIVERY-OPENING-20261112（下位已有源码回执，部署待确认）

用户已批准按最后125830比赛分析修复，并追加“安全区视觉只有偏差较大才修正”。
上位基线ad4a108；只读核对F405 main为19ec9f718d0cdb9431748c89d18b229d9251063b，
Task.c blob 3a7d26b19a5d64eae4aa83b26b2c7b07c4d1993d；固定入口blob 543c39c3cc7841805f93e5b007c720b680a8caaa。
本场actual同提交/dirty1、NormalRun/TASK、921600；dirty字节不以干净源码替代。

```text
请求ID：MATCH-DELIVERY-OPENING-20261112

为什么改：
1. 开局主等待在START_CENTER_APPROACH：距中心估计114mm仍追80mm，10—27.5秒几乎无轮运动。
   原140°静止预检已经真正运行，不重套MATCH-OPENING-CAPTURE-20261111旧补丁。
2. 绿+伤员确实同时在爪内，运输复审有效帧恰好漏绿；120°重新看到后没有回完整载荷审核。
3. 上位发送+201px视觉偏差，但当前视觉ALIGN只ACK不执行，随后按固定场地方向推。

最小配套：
A. task_process_start完成原1.10m快速前行本地轮程后，直接原task_finish_start释放中心观察。
   取消融合位置提前转START_CENTER_APPROACH及距中心80mm的抓取前提；粗段后偏差交给现有视觉。
   保原后退、180°、开爪、速度/PID与既有超时，不改其他NAV容差。上位原140°预检、非空合爪/空爪接近接手。

B. 实现原0x04视觉ALIGN的真实横移，保持车头垂直入口：
   先用主gyro对场地法向（红90°、蓝270°），随后保持该航向，视觉只控制左右横移。
   arg_a是有符号像素偏差：分区点减640，正值需车体右移，负值左移；arg_b仍0选120°/11500选115°，aux仍0。
   初始|偏差|≤64px不横移；>64px才开始，回≤32px结束；完成后64px内不反复微调。
   110—180mm/s复用原末端速度范围，1mm/s/px为工程初值；接收方核对实际MoveAngle方向并复用斜坡/主gyro控向。
   不把像素当毫米，不改gyro正向，不加目标转向或在ENTER中用融合XY纠偏。
   同一task/action实时更新arg_a与原0x1D源frame，不按SEQ重启。ALIGN需要走动态输入；定位ALIGN重发不重启。
   横移实际执行中用原mode10，真正对准后mode11；原completed须反映视觉动作完成，不能刚ACK或仅粗转正就置完成，
   后续新大偏修正不能继承旧completed。上位也结合实时偏差判断，不再把ACK当完成。
   原ENTER继续同一场地法向＋本地里程计直推，原0.67m动作、开爪/后退及扫障恢复保持；无需重写ENTER。

C. 推进前如果承载平台重新看见异类，回原140°运输复审：
   对闭爪TASK_ALIGN_SAFE_ZONE，允许现有0x12/CARGO_AUDIT占位请求进入原TASK_TRANSPORT_AUDIT/mode48。
   请求source frame=0、左右类别/计数/总数0、非STABLE，只保留本次目的地位；这是视角请求，不能计作空爪样本。
   停止横移、保持载荷，原相机准备后收真实两帧审核，非法走原分离、合法才重新ALIGN。
   同动作重发不能重启相机或清审核；不扩到任意NAV mode10、ENTER或已开爪/放货后的状态。
   当前mode10若实际是TASK_ALIGN_SAFE_ZONE横移中仍属此入口；上位结合已接受的ALIGN上下文识别。

上位已做：
- 闭爪期间衔接原140°ROI内明确看到的异类证据，历史帧单独记录，不向原始检测添加假框；释放/分离/放货清除。
- 平台混装与地面障碍分开，推进前回原140°复审；运伤员的地面走廊不再无条件豁免绿/黑。
- 实时视觉横移、64/32px规则、原0x1D源帧与同动作更新；缺实际框用原PAUSE，恢复后原动作继续，不增加计时等待。
- ENTER的定位XY没有新增纠偏；原mode51、首件单绿、首次后最多两件绿/黑、单伤员等保持。

不要求新增测试代码或独立测试程序。只复用现有构建/原入口做必要检查；
不得新增新鲜度、反复握手、资格等待、延时、保护停车/FAULT、永久锁存或额外ISR/遥测。
接收方可接受、缩小、替代或拒绝，先向人说明其他重要变更。代码及本端回执commit/push，
回传实际文件/行为、必要结果、未完成项、构建与烧录情况；expected不能冒充actual。
```

完整实现说明及必要检查：
```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_DELIVERY_OPENING_FIX_20261112.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_delivery_20261112/interface.json
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_delivery_20261112/checks.json
```

## 本端结果与部署边界

上位必要检查和本场同帧复算完成；3123/3127帧不再以单橙放行，3139帧识别平台额外绿238。
这是离线结果，不是实车横移或精度验收。本窗口未开设备、发运动命令、重启正式程序或清原日志。
前次交付时F405尚未配套；本轮已读到0d52ef8源码及回执实现三项，构建/夹具通过据该端报告，
对方明确未烧录。此刻实板未读，实车横移/开局效果待确认；不再重复下发已实现的三项。
大融合、ROI/模型、C/F/G等其他待办均未恢复。T265跳变与弱雷达约束未根治，不宣称由动作修复治愈定位。

## 已核对的前次回执

19ec9f7已实现原开局静止预检与4秒空爪出口，本场预检→GRAB已有实跑证据；
A的mode39末端纠偏、E的最多两件材料及原载荷保持也已在当前源码，不重复下旧任务。
原下位入口已保留人的知情/拒绝/选择权，规则无需重发；本次没有将历史任务整包带回。
