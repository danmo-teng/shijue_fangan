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

## 最新请求：MATCH-OPENING-CAPTURE-20261111

用户本轮只批准开局两项：先静止检查夹内；空爪补进复用原4秒出口。
上位代码已实现并通过必要检查；下位仅准备Task.c参考补丁，尚未实施/烧录。
接收方先审当前代码，向人说明接受、调整或拒绝的理由；不把发布当作已审核或额外改动授权。

```text
请求ID：MATCH-OPENING-CAPTURE-20261111
目的：解决中心已有货仍先对准物堆、空爪补进无进展长等。
范围：只改Task.c，复用原协议、相机、审核、GRAB与恢复流程；不扩电机PID/全局速度。

已核对当前main：Task.c内容blob cfc01337eb7e350382c00c61f132afaa45d669ff，
与7653af9b5f44ff18a1aa268eaf70d125ff0ee389的源文件一致。
两轮实际自报7653af9/dirty1、NormalRun/TASK、921600；dirty设置不能用干净Git替代。

1. 开局先看夹内。
上位在下位释放中心观察后发原0x18/CARGO_AUDIT：opcode=0x12、VALID，
audit flags仅INITIAL_STASH，左右类别/计数/总数全0、无STABLE，0x1D源frame=0。
这是请求视角，不是真实空爪结果。仅开局SEARCH、张爪且非运输/复审阶段接受，
走原相机20→21、140°流程，静止，不启动200mm补进。
同任务动作重复请求只确认、不重启相机/不记样本；沿用原task/action配对。
到21以后收原真实两帧审核，真实稳定非空等原GRAB、后复审、藏点NAV，不自行合爪。
真实稳定空爪回SEARCH，当前动作completed且非cancelled；重复完成包不重新进预检。
上位只发起这次开局预检，确认空爪后用新任务转原接近，不反复看空爪。

2. 普通/cluster初次空爪补进不能被每个空帧重置。
保原200mm额度，复用capture_wait_active/started_ms与原
APP_CAPTURE_DECISION_WAIT_MS=4000，持续空爪超出走原task_begin_capture_escape。
这是原补进决策出口，不是新鲜度门、附加计时/等待或FAULT。
代价：卡住/没货时更早退出重找，不再依赖轮程增加或长等待。

尚未确定：18:57轮23.0—29.9秒690次三轮增量全0，gyro也不动，但模式21报执行中。
源码请求200mm/s不等于PWM/实际运动。请结合已有TaskDiag目标/实测/PWM与输出归属
定位具体子因；需要其他控制修改时先向人说明原因/影响，不盲调全局参数，不扩新遥测。

完整说明、接口及检查：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/OPENING_CAPTURE_FIX_20261111.md
最小参考补丁（先审再摘取，不整包应用旧候选）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/reports/opening_capture_20261111/F405_OPENING_CAPTURE.patch

本轮不改ROI/模型/定位/其他比赛动作；不新增新鲜度、握手、延时等待、保护/FAULT或永久锁存。
不要求新增测试代码/程序。必要构建及原入口直接检查后，commit/push并回传
实际文件/行为、必要结果、未完成项及实际烧录身份；未烧录明确写未烧录。
人保留对其他重要改动的知情、拒绝和选择权。
```

## 本端结果与部署边界

上位11项相关检查通过，含真实非空23→22→藏点NAV、空爪回接近、重复请求/延迟回执、原定距/运输/mode51。
Task.c候选在已有C主机夹具的6项场景及原两帧/STABLE、渐进分离检查通过，补丁在当前干净源副本可应用。
这不是完整F405构建或实车验收；旧固件不执行SEARCH里的新观察入口，必须配套后联调。
本窗口未写/烧录下位机，未打开设备、启动电机或清日志；原ROI/模型/定位改动保留。
具体回执见docs/reports/opening_capture_20261111/checks.json。

## 已核对的上一轮回执

F405固定入口当前blob ff38e836db9bcf2d60a140be2dda4852026247e4，已记录从ad1cc2d实施A/E；
当前Task.c也包含mode39末端向量修正和首次后最多两件材料审核。B复用现接口。
不再把“下位仍单件/未实现A”当当前状态，也不重新下发旧MATCH-ABE任务。
18:51/18:57日志有7653af9/dirty1实际身份，但未因此宣称A/E实车效果或干净烧录版本已验收。

## 其他请求

D“合爪抬相机后不出发”仍未定位确切故障段，与本次合爪前卡顿分开；
C/F/G、ROI/局部黑块人工单、大融合等均未恢复。历史按对应报告/Git取证，不自动实施。
