# 两端最新修改要求

更新：2026-11-11。只维护当前相关请求和回执；旧请求按Git历史取证，不自动恢复任务。
发布、读取、镜像要求均不等于审核通过或实施授权。接收方先审实际代码，向人说明影响，
可以接受、缩小、替代或拒绝。人保留知情、拒绝和选择权；已明确批准范围不重复审批。
协作约束见本仓库AGENTS.md和docs/CHANGE_WORKFLOW.md，不增加时间门、握手或故障锁存。

## 固定位置及启用状态

```text
RDK仓库：danmo-teng/shijue_fangan，分支codex/gamepad-teleop
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md

F405仓库：kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot，分支main
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md
```

两端固定入口均已建立。F405的AGENTS及同名文件在41e46994420b825394b728220e5d672ce19c4828核对，
已记录人的知情/拒绝/选择权、请求不等于授权以及直接读取对端GitHub的规则。
当前GitHub连接对F405仍只有读权限；双方AI更新自己仓库相关要求/回执，不再逐次请用户转交。
这不是常驻执行服务；每次相关跨端工作先核对源文件当前提交和请求ID，避免拿缓存或旧版本执行。

## 最新请求：MATCH-ABE-20261111

用户本轮只批准A、B、E，其他先不做。上位B/E已实现，以下是给下位负责人的完整配套说明。发布不等于对端已经审核/烧录；接收方先看实际代码，简短向人说明接受、缩小、替代或拒绝的理由，超范围仍由人决定。RDK窗口不写F405。

```text
请求ID：MATCH-ABE-20261111
范围：只做A的mode39返程修正、E的最多两件材料审核；B复用现有接口，无需下位新功能。

当前只读源码：F405 main的Main/Src/Task.c内容blob
c98f949b1b1bbac6e8d6b01727b03c88a0cee74a
固定入口blob：fa52e939c599c144dcb99a39a23ae47956eeb7b7
两轮历史actual=e85c738/dirty1、NormalRun/TASK、921600，不代表此刻实机身份或干净烧录字节。

A：绿＋危险蓝长停在mode39，不能按允许绿蓝混运解决。
只改task_sweep_reverse_trace的SWEEP_REVERSE_DRIVE请求：
1. 沿锁定段计算场地向量 -remaining*u-cross*n，越过终点后纵向项为0，只修超限横差，
   不再用持续倒退＋偏航来消横差，避免越走越远。
2. 转成当前车体系后复用Motor_MoveAngleHeading维持原锁定航向，不再外加一套yaw输出。
3. 非零纠偏沿已有APP_NAV_FINE_MIN_SPEED_MM_S（当前110）到原120mm/s上限。
   保原完成容差、段游标、转向及后续相机流程。
影响：末端小纠偏比旧20mm/s请求更快，贴物横移可能更急；不是复用完整STAGE fine助推。
日志无子阶段/PWM，不能断言这是现场唯一根因。先审实际diff及速度取舍，不扩为新电机接口。
参考补丁只针对当前函数，可审后摘取；不整包套旧补丁。
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_abe_20261111/A_MODE39.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_abe_20261111/F405_MODE39_ENDPOINT.patch

B：上位已修“中心投影在色框内但脚点在围栏前”的错误排除，以及39转车后42继续锁旧像素/ID。
保留真正区内、走廊外、原载荷排除与44/45原货找回身份；灰框随原业务筛选结果变化。
F405继续现有像素APPROACH/GRAB接口，不新增TYPE/opcode或回执步骤。
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_abe_20261111/B_TARGET_SELECTION.md

E：只改task_audit_semantic的材料合法性，首次后上限2，绝不是3。
1. first_delivery_done之前仍必须且只能单绿；开局临时藏点的原非空语义保持。
2. 之后允许两绿、两黑或一绿一黑，同侧或分侧，都沿原合爪/复审/运输/投送。
   伤员仍单件；危险蓝、未知、伤员混合、三件及计数/类别不一致仍非法。
3. 用现有mixed_material=6表示同侧绿黑，只有该侧count=2才合法。
   复用原0x18/CARGO_AUDIT、两侧计数、总数、角色与STABLE；保现有两帧审核/任务动作去重。
4. 不改ENTER/EXIT、抬爪、退距或导航；不加入内部固定双推，也不做F的补推。
上位默认已改2；当前下位仍单件，必须配套后再联调，不能只拿上位双件代码对旧固件运行。
找回数量按新实际审核，原2件只剩1件时不为凑数等待。
首件阶段沿原双方共享标志，不把TASK_COMPLETE/DONE当成真实得分证明，不单边改阶段语义。
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_abe_20261111/E_TWO_MATERIALS.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/match_abe_20261111/F405_TWO_MATERIAL_AUDIT.patch

本轮不做：C贴墙反复扫障出口、D开局/预备点切角、F补推、G侧绕、ROI、模型、定位实验、
三件材料、主动单橙优先、固定双推。旧MATCH-D-PITCH-20261111仍暂停。
不新增新鲜度/附加时间门、反复握手、延时等待、保护/FAULT或永久锁存。
不要求下位新增测试代码或测试程序；只做必要构建/已有直接检查，commit/push，
回传实际修改文件/行为、必要结果和烧录身份，未烧录就明确写未烧录。
```

## 本端实际结果与未完成

A的16组几何检查通过。B的3项改前失败改后通过、另2项相关回归通过；E的5项直接回归与原mode51/运输分离检查通过，52组上位实际报文字段与配套实际C语义一致，3组原始raw_audit复算符合两件/首件规则。最后联合11项相关检查、4文件语法、27项保护指纹通过。两份补丁可在当前源码副本组合应用。

上述C检查复用已有审核夹具，不是完整固件构建。没有写/烧录下位、开设备/电机或实车验收；原始日志未删，用户现场ROI/模型/正式定位产物未动。当前E不兼容下位旧单件合同，等待对端配套回执。

## 其他请求

其他任务与旧请求本轮均不实施；历史要求/回执按Git历史或对应报告取证，不自动恢复。D仍未确定根因，ROI及局部黑目标归属交人工，F仍只考虑一绿一黑的有条件补推且未实施。
