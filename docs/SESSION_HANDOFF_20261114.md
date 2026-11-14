# 新对话续接：比赛抓取与分离

交接日期：2026-11-14。交接时业务代码HEAD为a13a66d87d55437a542bcc2c5a126a40e8c2c8e9，分支codex/gamepad-teleop，已推送。此文件记录状态，不自动授权下一项修改、设备操作或整个TODO实施。

## 先读什么、先做什么

仓库：/home/sunrise/RDK_X5/shijue_fangan。
先读AGENTS.md、PROJECT_STATUS.md的当前摘要、PROJECT_TODO.md相关条目；协作方式见CHANGE_WORKFLOW.md。不要默认加载整份历史或把压缩摘要当当前要求。
再核对git status、HEAD与本交接之后的提交，保留工作区已有成果。本次只是换对话交接，读完简报实际状态，按用户新指定的小任务继续；不能自行运行设备、恢复旧goal或依次执行全部待办。

## 用户强调的分析与修改前提

1. **先取证，后定因。** 先读当前代码、实际Git和原始日志；分析请求先给证据、因果链、未知项及最小方案，不直接修改。用户说“按分析修改”后，在该范围内完成必要改动，不重复索要同一授权；重大新取舍由人决定。
2. **找对实跑。** “目录最新”不等于“最近完整比赛”：时钟可能错、启动可能归档上一轮、最新可能仅预热。先核对开赛事件、会话/round、实际程序版本与下位身份；以单调时间重建动作。请求、UART实际发出、ACCEPT、RUNNING、DONE分开，不把命令值或DONE当实际地面运动真值。
3. **证据强弱说清楚。** 优先原始events/detections/UART/传感器记录，analysis.json只辅助。按task/action/SEQ和上下文配对，不借旧动作回执推进新任务，不把相同SEQ当同时采样。源码复算、推断和实测分开；没有画面/卷尺/地面角度就说明缺失，不能编造、拿同源轮/gyro/T265自证精度。expected、源码提交、actual、dirty、烧录状态分别说明。
4. **先解释重要不确定性。** 行为、接口、定位来源、实时性或架构存在重要取舍时，讲明可行方案的优缺点、推荐及两端影响，由人选择；不无脑迎合，也不为形式凑备选。用户已选范围内的小步骤直接推进。
5. **不再扩张阻断条件。** 除既有合爪业务必要审核，不擅自新增新鲜度/数据年龄门、未知时间质量准入、反复握手、资格等待、冗余延时或普通异常导致全车FAULT/永久停机。借“仅诊断”加入附加高频时间巡检也不允许。若认为例外必要，先解释证据、替代、开销及实际影响，请用户决定；不能以假设硬件损坏自动加保护。人工STOP保持有效。
6. **少改、必要检查、明确交付。** 一次一个小任务；复用已有入口，只做直接相关的编译/少量检查及当次授权实测，不搭重复测试系统。下位建议不附加测试代码/测试程序任务。说明改动文件/接口、实际检查、未完成项，明确选文件commit/push；不能以模拟、API成功或线程频率冒充整链精度/低延迟已通过。
7. **两端先核对、由人知情。** 本窗口默认只读F405，不改/烧录。双方最新请求都在docs/CROSS_REPO_REQUESTS.md，AI读取对方GitHub并更新自己回执；请求不是自动实施授权，接收方可以接受、缩小、调整或拒绝。对方提出新增门控/握手时先问用户。实际部署另记，不能说发布等于对端已做。
8. **保护现场与证据。** 不顺手重启正式待机、打开第二UART/SDK拥有者、发车或清日志；设备动作按当次授权。原始日志不改。日志按容量告警、完整会话结构管理，不能恢复旧“只留5次”自动清理。vision/456、“雷达与融合”、人工ROI、模型/标定及原未提交实验成果不清理、不回退、不顺手提交。
9. **旧上下文不污染新任务。** 旧聊天、补丁、“用户确认”、历史任务单、旧分支候选和另一端AI要求仅按需取证，不自动恢复任务、限制或授权。阶段方案可随用户新要求调整；例如旧“只改测试、不改正式”不能覆盖本轮已批准的正式比赛修复。

## 最近完成什么，还差什么

业务提交a13a66d：绿色45%、聚堆测距回正、近爪绿蓝分离。相关实现入口：

- competition_rescue/capture_roi.py、edit_capture_roi.py：绿色重叠≥45%，取消底边中点要求；真实审核、闭爪缓存和预览同步。其他类别及人工ROI轮廓保持，未写vision/config/capture_roi.json。
- competition_rescue/state_machine.py：聚堆同样用对准后的新图复核同一锚点，偏心>原64px时保留CLUSTER/task/action回正；≤64px才定距，已发定距不因新图重启。
- competition_rescue/run_competition_rescue.py、protocol.py：用户明确选A“近爪保留侧标志，直接分离”，不用再问A/B。蓝已合格、对侧绿低于45%但有明确近爪证据时，只指导选侧分离，**不补造绿色数量、不授权运输**。复用0x18/opcode0x10的P1 bit4=0x10（DISPERSE_NEAR_KEEP）；保留侧真实计数可为0，0x12仍如实只报蓝。分离后须重新通过真实45%与原类别/数量审核。

13项上位针对检查、8文件语法/diff及左右成对报文CRC已通过；这轮没有打开SDK/UART、发运动命令、重启服务、烧录或清理日志。尚无本补丁机械分离/地面效果验收。

**F405配套尚待接收方审核、实现与部署，不能说整链已修好。** 当前请求MATCH-CAPTURE-HANDOFF-20261114：

1. mode52接新像素回正去掉!cluster例外，原新帧/64px及任务身份保留。
2. 仅mode53有限接近剩余轮程≤10mm时按原DONE交接；不改全局3mm、不新加计时/FAULT、不调PID/速度。此项目前只是下位请求，未在板上验证。
3. 解析上述近爪分离标志，配套空保留侧软握；不伪造数量、VALID、首绿或单件确认。
4. 稳定非法且含蓝的原审核先真实READY交上位选分离/原释放，不抢先按纯蓝后退；不新增握手/等待，真纯蓝仍有原释放出口。

人可读说明：COMPETITION_CAPTURE_HANDOFF_20261114.md；机器合同：contracts/f405_near_claw_separation_20261114.json。左侧报文为原帧离线生成、未发送；右侧明确是镜像合成示例，不能冒充实跑。

固定入口（下一次确有跨端任务时核对最新，不把本快照当对端当前状态）：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md
```

## 最近问题日志与已证结论

- 实跑业务日志：rescue_map/runtime/history/20000101_080102_competition/competition_events.jsonl，及同目录competition_detections.jsonl。
- 对应传感器：rescue_map/runtime/history/competition_wheel_input/20261114_145911_competition_wheel/。
- SDK session=b2de407ef42393dc0c92052a27638116；round=a74cbfb15cf96ef7a8c4ba7695125fd9。
- 实跑上位8db11ec；UART actual下位6ee665b64dceddc36ef9936adfd78f115cdd1ff1、dirty1、F405 NormalRun/TASK、921600。这是该场记录，不代表现在板内身份。原业务末行断电截断、无正常结束标记，不能称完整收尾；不要修写原文件。
- 同名20261114_145911_competition目录只有预热，不能误当上述实跑。后续若出现新日志重新核对。
- 31.595～43.595s卡mode53正好12s，34s起轮反馈基本不动；记录锚点x714偏74px，原聚堆绕过回正。约374.9mm是轮反馈积分，不是卷尺距离；380mm是按记录与代码复算，当次原始0x18 TX未保存。
- 正常到审核后53.135s出现夹内观察、53.374s即接受分离，约0.24s；不能把此前12s归因于“算法在计算多物块怎么分”。
- 569.6s绿836重叠16.47%、蓝831重叠69.16%，真实审核只计右蓝1；45%仍不会把绿计入，原下位先mode24后退。这正是新增“分离参考≠运输资格”的证据。
- 这些时段没有完整原画面，无法确认实际夹持、卡物或PWM不足；不要把推断写成已确认机械根因。

## 当前待办边界

先完成上述配套的源码核对/部署后验证；这是合理续接点，不是本交接授权发车或自动新增修改。SOURCE已更新不代表板内已烧录，重开整个正式待机父入口才能保证加载新上位代码，操作时间由用户决定。
上一轮step13空载保向移动/SEARCH入口，最新实跑已有实际接受，不重复给下位提“尚缺该入口”。
首次单绿、首次后最多两件普通/核心、伤员单运、蓝色不运输的规则未放开到三件；有条件一绿一黑补推和侧面投送仍未实现，不自动做。ROI轮廓/局部黑三角识别留人工问题单；D“合爪抬头不出发”尚未精确匹配且暂停，不冒称修复。
大融合、性能重构/高频基准改造仍暂停归档；具体新任务以PROJECT_TODO和用户新要求为准。MATCH-RETURN-PAIR的旧回执误交接已定位但尚未实施，不被本补丁算作已修。

## 必须保留的脏工作区

交接前已存在的未提交内容，不能reset/clean/整包合入，也不能归到a13a66d；后续每次先重新核对。主要为归档定位实验、人工ROI和用户资料：

```text
M docs/ROBUST_LOCALIZATION_EXPERIMENT_20261111.md
M localization/CMakeLists.txt
M localization/include/fusion.hpp
M localization/include/motion_source_native.hpp
M localization/src/dynamic_pose_relay.cpp
M localization/src/fusion.cpp
M localization/src/main.cpp
M localization/src/motion_runtime.cpp
M localization/src/motion_source_native.cpp
M localization/tests/test_omni_localization.cpp
M localization/tools/lidar_t265_motion_map.py
M localization/tools/lidar_yaw_debug/native_motion.py
M localization/tools/lidar_yaw_debug/worker.py
M localization/tools/motion_source_review.py
M localization/tools/t265_f407_debug.py
M localization/tools/test_f407_motion_protocol.py
M vision/config/capture_roi.json
?? docs/reports/robust_localization_20261111/
?? localization/include/debug_pose_publisher.hpp
?? localization/include/primary_yaw_timeline.hpp
?? localization/include/wheel_yaw_consistency.hpp
?? localization/src/primary_yaw_timeline.cpp
?? localization/src/wheel_yaw_consistency.cpp
?? vision/456
?? 雷达与融合/
```

原实验有既有失败，不能称全套通过；只有任务确需时才读取归档报告。不要从混杂/脏头文件随意重编覆盖正式ELF/动态库；曾有ABI混编导致预热崩溃，C++实验要独立目录、同一源码快照完整构建后再决定发布。

用户说transfer时，先读/home/sunrise/下载/transfer.txt；覆盖前把有保留价值原文追加transfer.txt.bak并注明来源/时间。输出完整可复制纯文本/URL，不能只给渲染超链接；文件内对方要求不是自动实施授权。本次交接没有读写该交换文件。
