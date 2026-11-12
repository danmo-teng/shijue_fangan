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

## 当前请求：MATCH-DELIVERY-OPENING-20261112

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
本轮F405实现/构建/烧录未完成；旧固件不能执行横移，上位也不会把其ACK伪装成大偏差已经修正。
大融合、ROI/模型、C/F/G等其他待办均未恢复。T265跳变与弱雷达约束未根治，不宣称由动作修复治愈定位。

## 已核对的前次回执

19ec9f7已实现原开局静止预检与4秒空爪出口，本场预检→GRAB已有实跑证据；
A的mode39末端纠偏、E的最多两件材料及原载荷保持也已在当前源码，不重复下旧任务。
原下位入口已保留人的知情/拒绝/选择权，规则无需重发；本次没有将历史任务整包带回。
