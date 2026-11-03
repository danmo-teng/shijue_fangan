# 2026-11-03实车FAULT分析与下一次初始化

上位机日志/代码基线为 `e5629859a1e6b7d3102bee00982ef6066d2ed5d0`，分支codex/gamepad-teleop。
最新完整会话：`rescue_map/runtime/history/t265_f407_motion_map/20261103_224334_interactive/`。
前一223037会话无运动命令、无完整标记，状态文件末尾不完整，不能混作一次运动。
本轮只离线读日志和源码，没有打开设备、发运动或清理日志；两会话已加入模块保护清单，原始内容未改。

## 已确认的动作

| 请求SEQ / 实验 | 实际发送 | 回执及结果 |
|---|---|---|
| 127 / T1 | 后退500mm，700mm/s | 约1594.84ms后DONE，fault0，标量进度−500；不是独立实测500mm |
| 128 / T2 | 后退500mm，700mm/s | 约199.11ms后因再次点击实验按钮被STOP129替代，随后STOPPED/fault0；不是FAULT |
| 130 / T3 | 后退500mm，700mm/s | 约339.39ms后FAULT3，标量进度−18mm、Location航向字段0 |
| T4 | 未发出 | 已有FAULT状态，客户端要求先明确STOP；不能统计为第四次实车运动 |
| 131 | 退出STOP | 匹配STOPPED/fault0，证明协议响应恢复，不证明物理停稳或问题已修复 |

原始UART独立重解析得到首个FAULT帧，CRC正确：

```text
RX monotonic ns = 754103139517
A3 B3 1A 40 82 03 03 03 FF EE 00 00 CE DA C3
                 │  │  │  └MOVE_BODY
                 │  │  └FAULT=3 ODOM
                 │  └STATE=3 FAULT
                 └请求SEQ=0x82=130
```

508657字节原始RX重解析：6240旧帧、4385扩展帧，CRC/畸形/丢弃字节/部分包超时均0。
上位机没有因雷达缺墙或定位质量发送这个故障；0x1A是F405主动报告。

## 故障链条

实际板端身份为11eeacf、source_dirty=1、F405/NormalDebug/MOTIONDEBUG、协议1.1、230400。
GitHub main已核对仍为完整SHA `11eeacfda71f419834ff78401faf1a0d35a3831e`。
dirty=1意味着不能把GitHub内容当作完整烧录镜像，需下位机负责人给出构建差异。

在同一FAULT接收时刻：0x1F从正常03变为04（RECOVERING、READY=false），**age仅1ms**；
4.49ms后的下一次接收已恢复03。08帧也从835(0x343)变为836(0x344)，随后恢复，
其中不包含STALE位。轮反馈07仍有真实计数增量，flags129表示有效值/滑移状态未知，不是编码器故障位。
46.31ms后0x1A航向恢复非零，但动作结果仍保持FAULT，未自动续跑。

该会话0x1F共出现27个RECOVERING样本，age全部0～2ms，静止时也发生。
08中23个RECOVERING发布，发布时yaw接收事件年龄170～1914微秒；相邻发布的yaw/rate接收事件时间最大间隔均19.904ms。
这些是MCU接收事件时刻，非已标定物理采样时刻；但现有证据不支持“真的断了150ms”的解释。

对应已推送源码：

- [Debug.c:478](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/Main/Src/Debug.c#L478)：MOVE看到`Location_GetPose().valid=false`即报ODOM。
- [Location.c:172](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/Main/Src/Location.c#L172)：IMU不ready或heading_uncertain使Location无效。
- [Debug.c:183](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/Main/Src/Debug.c#L183)：`debug_finish`调用Motor_Stop并结束动作。

因此FAULT3是**下位机定位有效性被短暂IMU恢复状态打断**，不能直接解释为三轮编码器损坏、
雷达看不到围栏或RDK新鲜度门拦截。

## 根因候选：可确认的源码缺陷与缺失证据

[imu_uart.c:352](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/Main/Src/imu_uart.c#L352)
的IMU_Update先`now=HAL_GetTick()`，后`lock()`。两者之间若SysTick和UART中断更新了angle_ms/rate_ms，
临界区内可能出现旧now=1000、新sample_ms=1001，uint32差值成为4294967295，误触发lose_stream。
新鲜数据被暂时标成RECOVERING，三次有效角度后恢复，**与本次表象高度吻合**。
把读now放入同一临界区即可修复这个确定存在的快照竞态；不能改用不同零点的Robot运行时间代替HAL时间。

另一个入口是[imu_uart.c:150](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/Main/Src/imu_uart.c#L150)：
UART5出现ORE/FE/NE/PE也会立即lose_stream，并可能在age很小时恢复。
本次线上没有lose_stream原因、UART5错误寄存器和错误计数，所以**不能排除这个分支，也不能宣称现场唯一根因已证实**。
需要修明确竞态并补充最小诊断，不应通过上位机忽略FAULT、自动RESET或自动重放运动掩盖。
配套任务见 `f405_fault_followup_20261103.md`；RDK窗口未修改/烧录下位机。

## 雷达与运动记录的边界

219次匹配被当前几何规则接受：212次双轴、7次单轴；拟合RMS中位16.90mm、最大21.61mm。
用户反馈减速带使雷达看不到围栏，因此不能把这些接受数或小残差当作实际围栏识别正确的证明。
T265推算车体倾斜角中位1.27°、最大4.84°，提示平面扫描假设需要留意；它不是独立姿态真值，
也不足以在本轮拍脑袋设一个自动越障阈值。

用动作前[-250,-50]ms、结束后[150,350]ms的T265中心坐标中位估计端点：
SEQ127约0.432m、SEQ128约0.0086m、SEQ130约0.0345m。后两段被停止/故障截断。
没有ground_truth或人工停稳点，不能据此拟合比例、声称真实距离误差或修改安装外参。

## 本轮上位机改动

用户已确认：场地中心为原点，按初始车头最接近+X/0°选择90°等价分支，保留相对墙真实夹角，暂不分红蓝。
下一次推荐放场地中心无遮挡处，但不会强制把车的实测位置设成(0,0)，也不要求固定1号/135°。
复用补丁的全局矩形初始化，要求可观测两轴、非对称歧义排除、三帧一致，固定分支后用主yaw传播，
不在每次转弯重新旋转坐标。单墙或不足的墙证据不会伪造完整XY。

新增“越障暂停雷达/恢复雷达修正”：暂停时只传播原有SDK Pose与主yaw，保持已有偏差，不更新雷达XY，
原始扫描照录。恢复后重新检查墙约束；不归零、不自动换分支、不发底盘STOP或加永久锁存。
这是人工可用性标记，**不是自动遮挡/倾斜识别已经完成**。本次未改正式比赛、安装候选或比例。
