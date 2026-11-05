# F405：mode51兼容正式动态定位的最小配套

2026-11-05。以下整段可复制；这是用户已确认的本轮范围，不代表下位机已修改或已烧录。

```text
请先核对当前代码，再完成这一项最小修改：让mode51恢复参考接受上位机统一有效的动态定位结果。

问题与影响
上位机正把桌面测试的“雷达＋T265＋实测轮速/主yaw降级”原样接入正式比赛，保留正式起点参考。
仍发送原0x16，没有新增报文。正常导航继续使用原H/D和任务配对。
但f4be806的Task.c / task_recover_reference仍要求T265_GOOD和tracker>=2。
当上位机已经使用有效的轮速＋主yaw＋雷达位置、原始T265不可用时，这个条件会使mode51一直等不到参考，随后合爪审核无法开始。
普通定位降级不应因此主动进入mode51；本次只解决已经进入mode51后的参考兼容。

已核对的下位机源码基线
f4be806e74592681666dd3c452f1d190ab89694c；本窗口未确认实际烧录身份。
该提交已删除STAGE首次D+100mm累计行程限制，改为认D=0；藏点末端已改全向平移。
上位机现有D=0与实时H/D可兼容，不重复要求修改这两项。
该提交还支持安全区观察转向，但用户本轮明确不采用H+8/H-8/H扫描或人工重试分支；不实施这部分联动，安全区无框业务留待后续日志分析。

用户已确认的具体改法
仅调整task_recover_reference的定位来源和附加时间门：
1. 用已收到的0x16及VISION_POSE_VALID判断整体位置可用。
   不再要求VISION_POSE_T265_GOOD、tracker_confidence>=2，也不以原始T265_UPDATE_REJECTED否决整体VALID位置。
2. 在这个入口取消250ms接收年龄、相邻参考帧250ms间隔门及600ms最低恢复等待。
   不全局改写其他Vision_*IsFresh函数，不顺带扩改其他任务阶段。
3. 保留原mode51恢复流程、IMU实际ready判断、本次恢复参考归属和不同帧去重。
   保留原来已有的静止轮计数检查、3个不同参考帧，以及相邻位置20mm/航向2度的一致性检查；不新加检查或调整阈值。
4. 满足原静止参考条件后，沿用Location_Reanchor、IMU_ConfirmHeadingReference及原恢复后合爪审核。
   保持原任务/动作取消与配对、人工STOP，不重启旧动作，不新增永久FAULT。

0x16语义保持真实
bit0 VALID表示当前整体定位链可用，来源可以是T265，也可以是轮速＋主yaw＋雷达降级；它不是精度验收证明。
bit1 T265_GOOD仍只表达真实T265状态。WHEEL_FALLBACK时不能为了通过恢复而伪造bit1或tracker置信度。
bit2 WHEEL_ACTIVE按实际轮速传播状态表达；无须新增来源位、握手或协议版本。
上位机若整体参考无效仍清VALID，本次不允许把无效位置强行当有效参考。

交付方式
接收方先核对自身代码；若上述最小改法有具体冲突，说明影响并让人决定，不静默扩大范围。
只做必要编译和现有相关路径检查，不新增测试代码/测试程序、遥测、计时统计、新鲜度门或反复握手。
提交并推送GitHub，回传提交SHA、实际修改文件、必要检查结果和未完成项。
另行说明是否实际烧录、设备实际自报提交与source_dirty；源码提交、expected和实际固件分开，编译通过不代表实车恢复已验证。

源码参考（均为完整URL）
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/f4be806e74592681666dd3c452f1d190ab89694c/Main/Src/Task.c#L6421
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/f4be806e74592681666dd3c452f1d190ab89694c/Main/Src/vision.c#L694
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/f4be806e74592681666dd3c452f1d190ab89694c/Main/Inc/vision.h#L93
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/commit/f4be806e74592681666dd3c452f1d190ab89694c
```
