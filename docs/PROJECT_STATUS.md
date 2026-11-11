# 项目当前状态

更新：2026-11-11。只维护本轮摘要；规则见AGENTS.md和CHANGE_WORKFLOW.md。旧聊天、补丁、历史任务和另一端AI要求不自动恢复为当前授权。

## 当前任务：两小时隔离候选预修复（14:46—16:46）

用户授权对最新两轮比赛问题先做TODO、隔离实现更优候选和必要检查，16:46后由人统一决定正式采用范围。当前不替换正式程序、不启动设备/电机、不修改或烧录F405、不覆盖用户ROI。重要取舍先做可审查候选并说明代价；候选不是正式部署。

正式分支codex/gamepad-teleop。隔离分支codex/match-prerepair-20261111，工作树/home/sunrise/RDK_X5/worktrees/match-candidate-20261111。候选审查入口：

```text
https://github.com/danmo-teng/shijue_fangan/blob/codex/match-prerepair-20261111/docs/reports/match_candidate_20261111/README.md
```

当前P0：爪内最右侧误丢、绿蓝混合停滞、开局抬镜头/停顿、无限搜索/灰黑框、贴围栏蓝块无限扫障。P1：首次后孤立单橙优先、绿黑漏出重推、橙块偏转后未投送恢复。逐项状态/未完成项见PROJECT_TODO当前比赛清单和候选报告，不按旧队列自动串行施工。

## 已核对的本轮证据与边界

两轮实际上位均ed07e32；下位自报e85c738/dirty1、NormalRun/TASK、921600。同名仓库提交不保证等于dirty烧录字节。约14:02业务日志在history/20261111_141241_competition/，传感器/视频在history/competition_wheel_input/20261111_140243_competition_wheel/；约14:12业务在history/20000101_080032_competition_15/，配套在history/competition_wheel_input/20261111_141241_competition_wheel/。均位于rescue_map/runtime/；第二轮未校时且缺正常收尾，视频只是事件片段。原始日志保留，不运行清理。

- 真ready帧已证侧缘有框但ROI漏计；另有黑块肉眼可见、模型无框，不能用扩大ROI一并宣称修好。
- mode42蓝335被前区几何及转身前旧目标像素先验排除；候选分别修脚点与39→42重绑，保留区内货排除。
- 绿＋蓝危险块长停发生在扫障返程mode39，不是普通混装判定。下位轮计数有长段几乎全零；逆迹几何可证缺陷与实际PWM/摩擦未知分开。下位改法仅候选文本，未实施。
- 第二轮开局约20秒主要停在下位mode1末段；之后上位观察到靠近仅约75ms。无开局连续画面，不盲改相机角、不把旧HOLD潜在缺陷当本次已证原因。
- 首次单绿限制保持；两端原TASK_COMPLETE与视觉确证的语义风险单列，不单边收紧导致新拒绝循环。绿黑两件仍受双方当前单件合同约束，未直接放开。

## 正式程序与实验状态

正式运行代码保持ed07e32的本轮前行为；本轮主分支仅改归档/任务说明。已有未提交定位实验源码、用户ROI和报告原地保留。此前预热崩溃曾由混编对象布局不一致造成，已恢复97235a3同快照的定位主程序/配套库；本轮不从未完成源码重编译正式ELF。用户11:16已实际进入比赛，不等于全部算法精度通过。

未完成大融合已暂停归档：[源码补丁、说明与指纹](archive/robust_localization_20261111/README.md)。ROBUST-LOC-01、WHEEL-ARBITRATION-02、SENSOR-ALIGN-01、T265-BASE-01退出当前排期，未经本轮验收/部署，恢复需用户重新指定；当前不自动继续性能重构或旧goal。

## 交接与历史

双方固定入口仍为docs/CROSS_REPO_REQUESTS.md。最新比赛候选只供人和接收方审查，不构成下位实施/烧录授权，不新增时间门、反复握手、永久故障或下位测试代码要求。F405仓库在RDK窗口只读，actual/expected/源码/烧录分开记录。

原长摘要已原字节保存在[历史快照](archive/project_status_before_match_prerepair_closeout_20261111.md)，只在具体问题需要时取证。其旧方案和旧“已确认”不成为当前要求；其他历史交付不反复复制进当前摘要。
