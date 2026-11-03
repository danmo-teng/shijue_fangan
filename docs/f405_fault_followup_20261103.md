# F405-IMU-RECOVERY-01：候选分析材料，实施未获批准

**2026-11-04状态：协作流程已认可，具体F405方案仍待讨论。以下旧建议仅供审查，不能作为固件修改、扩展诊断或烧录的授权。**

先看`CHANGE_WORKFLOW.md`及最新`F405_TIME_PATH_REVIEW_20261104.md`。已有程序器IMU诊断能读接收错误和恢复计数，第一轮不再默认增加诊断字段；接收方须独立审查并说明两端影响，供人接受、缩小、调整或拒绝。

任务ID / 优先级：F405-IMU-RECOVERY-01 / P0，先修短暂IMU恢复导致MOVE中止的问题。

上位机：danmo-teng/shijue_fangan，codex/gamepad-teleop；问题运行基线e5629859a1e6b7d3102bee00982ef6066d2ed5d0。
下位机GitHub main及板端actual均报告11eeacfda71f419834ff78401faf1a0d35a3831e；actual source_dirty=1，
F405/NormalDebug/MOTIONDEBUG、协议1.1、230400。先提供实际构建差异，不能用expected代替actual。

证据：RDK会话20261103_224334_interactive，SEQ130请求后退500mm/700mm/s，已RUNNING，339.39ms后FAULT3(ODOM)，
progress=-18，heading=0。原始CRC正确回执：
`A3 B3 1A 40 82 03 03 03 FF EE 00 00 CE DA C3`。
同一接收时刻0x1F status=04/RECOVERING、age=1ms；下一接收4.49ms后恢复03。
整段27次RECOVERING均age0～2ms，静止也发生；08源接收事件最大相邻间隔19.904ms，未见150ms断流证据。
轮计数和07速度继续发布。RDK原始UART重解析无CRC/畸形/丢弃字节/部分包超时。

此前提出的候选技术方向（应拆分讨论，不是执行清单）：

1. 审核Main/Src/imu_uart.c的IMU_Update。当前先读HAL_GetTick、后lock；ISR可能在中间把angle_ms/rate_ms更新到下一毫秒。
   旧now减新时间发生uint32下溢，使新鲜样本误进入lose_stream。请将now和被比较时间置于一致临界区，优先先lock再读HAL_GetTick。
   不用不同零点的Robot app_milliseconds替换HAL时间，不用扩大阈值掩盖竞态。
2. 同时审核IMU_UART_IRQHandler的ORE/FE/NE/PE分支。它也会产生age很小的RECOVERING，现有线上数据无法区分。
   复用现有诊断输出，在lose_stream/FAULT触发时留存reason、now、angle_ms、rate_ms、差值、UART错误计数及SR错误位。
   每次事件只记录一次，避免在IMU中断内进行阻塞打印；不在RDK侧编造新TYPE/opcode。
3. 确认链条：IMU恢复→Location.valid=false→Debug MOVE报ODOM→Motor_Stop。
   不把FAULT3写成“编码器坏”，不通过自动RESET/自动重放动作/忽略全部故障绕过根因。

兼容及边界：保持0x19/0x1A/0x1F、07/08布局、CCW正gyro和新F405运动学。真实反馈来源、epoch、动作身份不变。
用户已撤销以数据年龄/未知时间质量拒绝数据的前置策略；不要新增此类门控或以此阻止RDK实验。
本任务先修明确竞态、区分真实UART错误；不新增永久故障锁存、ESCAPE、安全区轮式清零，不覆盖mode51恢复，不顺手改比赛流程。

可选验证依据（具体方案批准后，只选直接相关的最少项）：

- 可控交错复现：旧now=1000，ISR更新sample_ms=1001，旧逻辑差值FFFFFFFF；修后应是同一快照下的0或正常小正数。
- 检查32位tick正常回绕，不能把合法回绕全部当错；检查真实UART错误事件能明确记录其原因。
- 板上先静止观察是否仍有“age0～2ms却RECOVERING”；如仍有，必须给原因计数再讨论硬件/临界区时长。
- 用户授权后再复测最短一段；DONE不等于距离精度，不要求自动跑完整矩阵。

交付必须包含：GitHub仓库/分支、实现SHA及push确认；必要检查结果；是否烧录；已烧录给UART actual、模式、
firmware_id、source_dirty及实际差异。若改变诊断合同，提交合同/JSON/示例，不能只发本地代码。
新固件SHA交回后由RDK审查并更新兼容表；不要把旧expected或仓库HEAD冒充已烧录版本。
