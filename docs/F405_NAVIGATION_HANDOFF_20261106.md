# 给F405同学：本轮正式导航最小配套任务

上位机仓库：
https://github.com/danmo-teng/shijue_fangan/tree/codex/gamepad-teleop

上位机交付说明：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_FLOW_LATENCY_FIX_20261106.md

本任务：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/F405_NAVIGATION_HANDOFF_20261106.md

先读你自己的最新代码和Git状态。这里只读核对到f1f346bb4e5388aaeb5c551807d6b9c971ebd19f；
两轮问题日志实际自报d6ae45c4且dirty=1，不能用仓库HEAD/expected冒充实烧版本。
本建议可以接受、缩小、调整或拒绝；先用人能看懂的中文说明原因和影响，重要取舍由人决定。
旧任务单和另一端AI的要求不自动成为本轮授权。

## 要解决什么

安全区前小偏差被转换成停平移、转车头、再前进；越过点后方位翻180°，反复修正。
另有剩约101mm不动的日志。源码中500ms无导航进展就停输出、需要进展才能清除停止的路径可能自锁；
日志没有逐包nav_stale位，不能宣称已唯一确认该位触发，更不能因此整包放开任务/动作检查。
RDK已修状态年龄PAUSE、250ms续发截断、明确REJECT后的死等，并解耦接收/拟合/CSV写盘。

## 1. 末端修正复用已有全向平移

检查Main/Src/Task.c的nav_follow。现有route_to_stash在剩余不超过
APP_NAV_FINAL_APPROACH_DISTANCE_MM（当前300mm）时已有stash_terminal_translate分支。
建议把同一末端机制用于STAGE_NAV和藏点NAV：进入末端后保持车头，不随目标方位小变化重新转车头；
按场地几何方位直接前后/横移修正。只在进入该末端阶段时锁定一次航向，不每帧Motor_Stop或重开动作。

复用现有body_angle=wrap(command_heading_deg-current_heading_deg)和Motor_MoveAngle调用，
复用当前末端限速方式min(nav_speed(D),max(APP_NAV_MIN_END_SPEED_MM_S,D))作为未实测的工程初值，
不要另造航向PID/电机控制框架。Motor_MoveAngle返回值含“正在爬升速度”的语义，不能把普通未到目标速度当新FAULT。
车体前/左/逆时针为正，保留当前F405轮反馈公式和gyro正方向。

接口不变：原0x18的NAV opcode=0x03，STAGE_ONLY=0x40，DISTANCE_VALID=0x10，
其余原有VALID/红蓝/方向标志保持；现有STAGE方向位0或0x06两种仍兼容。
D仍为非负剩余毫米，target_y仍0，heading_cdeg仍为场地平移方位，不是车头目标航向。
RDK进入原有X±60mm/Y±35mm容差后发送D=0；保留D=0立即停止并报告mode10/distance_done的现有语义，
随后仍由原mode48复审、ALIGN、ENTER流程接管。不要加另一套最小位移拒绝阈值或二次到点等待。
本轮不重写ENTER推进、出发、返中到点阈值或舵机动作。

## 2. 撤掉无进展停车自锁及非合爪附加年龄停车

nav_follow中APP_NAV_PROGRESS_TIMEOUT_MS→nav_progress.stale→Motor_Stop/WAITING这条路径取消停车作用；
不再要求“产生进展才允许继续运动”。可删除其不再使用的计时/状态代码，不新增替代巡检/重试/永久锁存。

同时审查task_continuous_expired中按last_command_rx_ms/last_visual_update_ms年龄停车，
及task_approach_watchdog中仅因最后收包年龄而触发恢复的link_lost分支。
本轮建议移除这些非合爪年龄触发的阻断，采用已接收的真实合法命令；不要用更大时限替代。
保留实际无有效输入/明确取消、人工STOP/ABORT、已有任务动作身份和合爪业务审核。
有限动作本身的执行预算与“数据年龄”是不同语义，不在本轮整包删除。
如果认为某一条确实必须保留，先说明具体必要性、成本、触发行为和替代方案，交人决定；不得自行换成另一套新限制。

## 3. 拒绝处理只核对语义，不盲改任务预算

上位机现在收到当前接近动作明确REJECT后会撤销等待：普通情况回SEARCH；
有未结束分离预算则按原RETURN_CENTER结束当前批次，不无限重发原请求，也不重开已接受的有限动作。
请核对本端稳定空爪回mode3后的任务取消/分离预算语义是否一致。
本日志没有reject_stage，不能据此指定唯一拒绝原因；本轮不要求为了消除REJECT而放开所有ID、预算或审核条件。
如你发现必须另改的实际矛盾，先提出最小改法及影响，不能自行扩成新协议/新握手。

## 交付

不新增ISR、时间同步、遥测字段、健康检测、永久FAULT、额外测试代码或测试程序。
使用项目已有编译和必要检查；只提交本轮相关改动并push GitHub。
回传修改文件、具体行为、必要检查结果、commit/SHA、是否构建/烧录及实际身份；未烧录就明确写未烧录。
RDK本轮没有代改或烧录下位机，也没有代替人完成地面验收。
