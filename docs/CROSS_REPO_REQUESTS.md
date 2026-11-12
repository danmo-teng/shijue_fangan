# 两端最新修改要求

更新：2026-11-12。只维护当前相关请求与回执，历史按Git取证，不自动恢复。
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

## 当前证据：MATCH-SAFE-VISUAL-20261112（分析，不是固件实施任务）

续查补充：RDK用原帧9731/9758/9781复现围栏外橙色802被safe bbox标成区内，目标选择返回None；
建议真实围栏前沿/区域共用于分区与区内外，几何候选尚未实现，不把改为115°或平滑bbox当完整修复。
新增MATCH-PILE-ALIGN-20261112分析：首绿7次进入mode52、6次回20；聚堆锚点虽锁定，发送点却是动态邻居联合框中心，
测距又改用锚点。建议上位统一对准/测距锚点；下位普通目标近中心减旋已经存在，不能再发一遍旧实施要求，
聚堆目前!cluster_target_active条件排除该优化、仍40px/最低60mm/s，可讨论沿用原近中策略。
本轮只提修法，尚未批准代码或固件实施；不新增时间门、等待、握手、测试程序/遥测或ISR。
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

后续候选是修视觉分区几何/控向耦合，并由下位审查已有横移锁角与Fine补偿可否复用；
当前用户只要求先分析，尚未批准新代码或固件改动，不要求新增测试程序/遥测/ISR/握手/时间门/停车。
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
