# F405 四键操作声音反馈配套任务

更新：2026-11-06。本页只交接本轮新增蜂鸣器功能，不恢复历史任务。

## 当前事实与最小改动

用户已同意四键操作：待机选择出发区、安全区，短按启动，按下停止、长按退出。
F405 板上蜂鸣器已由用户确认存在且曾经响过；当前资源表列出 PA10，默认保持低电平。
希望用现有蜂鸣器确认选择，避免无屏时盲按；不新增灯、屏幕或其他 IO。

上位机已实现发送端和唯一 UART 所有者适配，源码工作已完成；F405 只读核对基线为
`51f1d0709059566c41b818309711986d22f34982`，该版本**没有这里定义的 0x20 蜂鸣指令**。
上位机本地发送/写串口成功不等于蜂鸣器已响，本次未烧录或实测声音。

请接收方先核对自己的最新代码、PA10 初始化及蜂鸣器驱动极性，再实现下面这一个小功能。
可以接受、缩小、调整或拒绝建议，但请用简短中文说明具体冲突、影响和替代方案，让人决定；
不要默默扩成新启动协议、遥测或故障管理框架。

## 已有功能继续复用

- 启动继续使用原 `0x11 CONFIG`，只在用户启动本轮后发送。待机选择和声音反馈不发送 CONFIG。
- 人工停止继续使用原 `0x18 / ABORT(0x07)`，不新增 STOP/启动 opcode。
- 当前 `Task.c:535` 的 REMOTE_STOP 已调用 `Vision_RearmConfig()`；`Task.c:6815` 在收到新配置后沿
  `task_initialize()` 重建本轮任务。保持该行为及原收爪流程，不重复改造重启接口。
- 上位机待机由同一个 C++ 可执行程序的 `--operator-uart-only` 持有 UART，不创建 T265 SDK；
  比赛时由正式 C++ 进程持有 UART/SDK。所有者顺序交接，不同时打开两个 UART 控制进程。
- 正式 C++ 收到本轮人工 ABORT 后停止该进程内旧任务/CONFIG/心跳续发，重开本轮进程才恢复；
  这是用户手动停止语义，不新增传感器故障锁存。

## 新增报文：固定 15 字节 TYPE 0x20

**不能使用 0x1B**：当前 `vision.h` 在非 MotionDebug/Gamepad 条件下已为运动命令预留 `0x1B/0x1C`。
本轮只读核对的消息定义未使用 `0x20`，因此蜂鸣器独立使用 `0x20`；若接收方更新代码已占用，
先反馈冲突、协商字段，不私自换 TYPE 后宣称两端已对齐。

| 字节索引 | 内容 |
| --- | --- |
| 0..1 | `A3 B3` |
| 2 | TYPE = `20` |
| 3 | SEQ，沿用 8 位传输编号；不作为动作编号或声音去重条件 |
| 4 | 本消息版本 = `01` |
| 5 | 响声次数，`01..04` |
| 6 | 响声种类，`00` 短声 / `01` 长声 |
| 7..11 | 保留，必须为 `00` |
| 12..13 | CRC16/MODBUS，计算字节 2..11，初值 `FFFF`，低字节在前 |
| 14 | `C3` |

建议固定声音节奏：短声 100ms、长声 350ms、相邻声间隔 150ms。该计时只用于声音播放，
不参与运动、启动、传感器新鲜度或资格判断。次数表示完整的响声数量，最后一声后保持关闭。
新合法蜂鸣请求替换正在播放的声音，不维护会不断积压的声音队列；同一 SEQ 再收到也按一次新请求处理。
消息只表达播音参数，不改变出发区、红蓝方或比赛状态。

SEQ=0 的完整例子，已与上位机 `beep_frame()` 核对：

```text
1短：a3 b3 20 00 01 01 00 00 00 00 00 00 a0 61 c3
4短：a3 b3 20 00 01 04 00 00 00 00 00 00 f5 61 c3
1长：a3 b3 20 00 01 01 01 00 00 00 00 00 a1 b0 c3
2长：a3 b3 20 00 01 02 01 00 00 00 00 00 92 b0 c3
```

机器可读字段和相同例子见 `localization/tools/contracts/f405_operator_beep_v1.json`。
这是新消息自己的版本 1，不要求新增能力查询、升级旧遥测报文版本或等待版本握手。

## F405 实现边界

1. 在已有固定帧解析器 `vision_frame_valid()` 中允许 0x20，沿用原帧头/长度/CRC校验。
2. **在 `vision_save_frame()` 消费或清除 `context_pending` 之前，独立处理 0x20 后返回。**
   该消息不得改写任务上下文、任务/动作编号、`latest_data.mission`、动作 ACK、配置确认次数或 `config_ready`。
   原代码会在进入业务分支前无条件清 `context_pending`，直接在末尾追加 `else if` 会破坏配对，不能这样接。
3. 合法消息仅提交当前播音请求；非法版本/次数/种类/保留字段仅忽略这条声音消息，不清任务状态、不进 FAULT。
4. 在现有低频主循环/已有任务轮询中推进 PA10 播音状态，用现有系统 tick 表达声音时长。
   不用 `HAL_Delay`，不在 UART 接收/中断上下文等待响完，不新增 ISR、定时器中断、长临界区或高频轮询。
   蜂鸣器不应占用电机/编码器/IMU的实时路径。确认现有板级关闭电平后，启动默认关闭，播放结束回到关闭电平。
5. **不返回 ACK、不新增握手或周期查询、不检查数据年龄、不新增控制许可等待、不开新遥测流。**
   声音失败不能阻止启动/停车，也不改变原动作配对、mode51、合爪审核或现有比赛策略。

## 交付与验证

只做必要编译和现有相关调用路径检查，不要求新增测试代码、测试程序或新的诊断框架。
核对 PA10 没有复用冲突，以及蜂鸣消息插入时不消费 `0x1D + 0x18` 任务配对。
真实响声可以在获准的不动车条件下直接确认；编译/串口写成功不能替代实物响声结果。
不要为了蜂鸣验证启动电机或重放比赛动作。

完成后提交并推送 GitHub，回传：提交 SHA、实际修改文件、必要检查结果、未完成项。
另行说明是否烧录、设备实际自报 SHA/`source_dirty`；源码提交、构建 expected 和实际烧录身份分开。
本页没有授权修改下位机其他功能；既有源码若不同，请先反馈实际差异。

## 完整参考链接

```text
上位机仓库：
https://github.com/danmo-teng/shijue_fangan
本轮交接（分支内容会更新）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/F405_BUTTON_BEEPER_HANDOFF_20261106.md
机器可读合同（分支内容会更新）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/localization/tools/contracts/f405_operator_beep_v1.json
上位机消息构造（分支内容会更新）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/competition_rescue/operator_protocol.py
已核对下位机基线：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/commit/51f1d0709059566c41b818309711986d22f34982
消息TYPE与字段：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/51f1d0709059566c41b818309711986d22f34982/Main/Inc/vision.h
帧分派及context_pending：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/51f1d0709059566c41b818309711986d22f34982/Main/Src/vision.c#L477
人工ABORT后重新配置：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/51f1d0709059566c41b818309711986d22f34982/Main/Src/Task.c#L535
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/51f1d0709059566c41b818309711986d22f34982/Main/Src/Task.c#L6815
PA10资源：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/51f1d0709059566c41b818309711986d22f34982/docs/F405_RESOURCE_MAP.md
```
