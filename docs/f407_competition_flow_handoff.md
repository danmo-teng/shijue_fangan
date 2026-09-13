# 完整智能救援流程的 F407 配合说明

本文是上位机新分支的接口说明，不是F407补丁。F407仓库不得由上位机仓库自动修改或生成补丁。

上位机仍通过定位进程的原子`uart_command.bin`发送固定15字节`TYPE=0x18`帧：

> **当前配合版本说明（2026-09-13，以上位机`codex/gamepad-teleop`最新提交为准）**
>
> 本节是当前F407配合合同，优先级高于本文后面的历史设计文字。F407仓库仍由下位机负责人
> 自行修改；上位机仓库不生成或应用F407补丁。命令编号、15字节帧、CRC、序号和现有正常
> 抓取/导航动作均保持不变。实现只允许完成下列明确事项，不要自行增加新的超时、重试、
> 自动重启或坐标修正逻辑。

## 当前上位机流程与F407最小配合要求

### 1. 开局分流

上位机在F407进入`mode=3 SEARCH`后先判断是否有稳定且可单独取得的绿色物资：

```text
单独绿色物资 → 直接APPROACH/GRAB/NAV，不进入临时藏堆
无单独绿色且存在聚集物资堆 → initial_stash临时搬堆
无可靠目标 → 继续SEARCH
```

上位机只在`initial_stash=True`的夹内审核中放宽类别和数量限制。F407需要同步调整
`task_validate_audit()`：

- `initial_stash=True`时只要求审核帧稳定且`total_count>0`，不按绿色、核心、伤员、危险、未知类别或总数拒绝；左右数量字段只是15字节协议中的饱和显示值，不作为开局藏堆的容量联锁；
- `initial_stash=False`时继续执行正式投送硬联锁：首件恰好1件绿色，物资批次不超过3件，
  伤员只能单独1件，危险/未知/伤员混装不能进入正式投送。

### 2. 临时藏堆后的返中

开局藏堆释放完成后，上位机只发送指向场地中心`(0,0)`的`RETURN_CENTER`，直到收到F407新鲜`mode=3`。
返中期间不处理视觉目标、不启动危险换道、不进入SEARCH。

F407在`mode=17 FACE_FIELD_CENTER`中必须先快速转向中心航向，再正向驶向中心；不要采用
“不转向、直接倒车、边倒车边修正”的返中方式。中心附近进入独立终止窗口后停车并报告`mode=3`。
不要用普通HOLD提前把返中转换为SEARCH；HOLD只能在已经完成RETURN且F407处于SEARCH时作为搜索心跳。

上位机在收到`mode=3`后还会让F407继续完成当前SEARCH的90°和120°两轮扫描；中心搜索累计约
720°且没有连续3帧稳定目标时，才允许前往藏点复查。

### 3. 正式抓取、混装和打散

```text
SEARCH → APPROACH_TARGET → mode=20
→ mode=21夹内观察 → 稳定CARGO_AUDIT → GRAB_CONFIRMED
→ mode=22/GRIPPER_CLOSED → NAVIGATE
```

不同投送区物资混在一起、危险物与其他物资混在一起时，F407只按以下顺序执行：

```text
夹内审核非法 → RELEASE_BOTH → mode=34
→ YIELD_BACKOFF → mode=30
→ 确认空爪 → SEARCH → DISPERSE_PILE → mode=35
```

空爪、没有`cargo_recheck_pending`时才接受`DISPERSE_PILE`。打散过程中不要因为视觉暂时
漏帧而停止已经接受的DISPERSE动作；F407继续执行自身15秒动作保护。

### 4. 正式安全区投送

上位机可能在地图停车点以前，因为视觉已经确认当前物资完成“安全区外→安全区内”而发送
`ENTER_SAFE_ZONE`。F407必须保留现有行为：

- `TASK_NAVIGATE`收到合法ENTER时，若末端补推尚未完成，继续完成末端慢推；
- 补推完成后打开双爪，摄像头转到检查角，进入`mode=15 RAM_VERIFY`；
- 不要求上位机再次发送ALIGN，也不因为上位机提前ENTER而进入旧对正流程。

上位机随后在`mode=15`下确认视觉投送，发送`TASK_COMPLETE`。如果有限观察窗口内无法看到被推物，
但F407已经报告新鲜`mode=15`且没有明确的区外残留证据，上位机按超时依据发送`TASK_COMPLETE`，不无限等待。
F407完成张爪后进入
`mode=17 FACE_FIELD_CENTER`，等待RETURN_CENTER。

稳定`CARGO_AUDIT`中的类别已经表达普通+核心混合（左右分别为普通/核心，或一侧为
`MIXED_MATERIAL`）。F407需要锁存该标志：普通+核心混合时在收到`ENTER_SAFE_ZONE`后自行执行
“短距离前推→短距离后退→再次前推”；不需要抬大舵机，上位机不发送这两个子动作。

### 5. 安全区卡住脱困

上位机只在完成投送、F407处于`mode=17`、RETURN_CENTER已经被接受，并且T265平移、航向和
编码器在较长观察窗口内都无进展时，发送一次`ESCAPE_MANEUVER`。短暂停顿、摄像头转动、
原地对正和普通动作ACK延迟不能触发该命令。

F407在`TASK_FACE_FIELD_CENTER/mode=17`接受ESCAPE时需要：

1. 保持双爪打开；
2. 先调用`Lift_SetTravelPosition()`抬起前方机构；
3. 按现有ESCAPE参数完成转向和横移，使车体离开安全区围栏；
4. 动作完成后报告新鲜`mode=31 ESCAPE_DONE`，并更新ACK。

上位机收到本次`mode=31+ACK变化`后恢复RETURN_CENTER。F407不要因为该动作执行时间较长
要求上位机重复发起，也不要把未完成动作报告为完成。

### 6. 场内长期无进展

上位机只在明确的APPROACH/NAV/RETURN平移阶段，且F407报告相应运动状态、命令已经被接受时
联合观察：

```text
T265平移长期无变化
+ T265航向长期无变化
+ 编码器有效进展长期无变化
```

正常短暂停顿不触发处理。第一次处理发送`YIELD_BACKOFF`并等待新鲜`mode=30+ACK变化`；
再次确认仍无进展才发送`ESCAPE_MANEUVER`并等待`mode=31+ACK变化`。F407执行自身7秒/15秒
动作保护；上位机不因动作慢、位移暂小或ACK延迟发送ABORT。

### 7. 故障和通信状态

F407需要区分可恢复告警和真正锁存故障：

- 可恢复的短时无进展、临时定位等待、目标帧超时、动作等待不能置永久fault；
- 真正的电机方向/堵转、IMU失效、编码器硬故障仍按F407本地保护停车；
- 上位机不把每个`fault_code`再次转换成ABORT，也不在通信短暂恢复前反复发ABORT；
- F407自身命令看门狗、CRC和序号检查保持不变。

上位机只在用户主动急停/退出或确认车辆越界时发送ABORT。F407若已经进入锁存故障，等待
人工复位，不要依靠上位机发送普通任务帧尝试“自动清故障”。

### 8. 不得修改的内容

- 不修改15字节协议、命令编号、CRC和SEQ语义；
- 不恢复旧的`ALIGN_SAFE_ZONE`对正流程；
- 不新增上位机或F407重复的几百毫秒级目标/动作超时；
- 不修改场地坐标、安全区坐标、出发区坐标和定位轴参数；
- 不把视觉漏帧、短暂停顿或单次ACK延迟当作电机故障；
- 不自动重启比赛、不自动清除锁存故障。

### 9. 安全区末段的万向轮对位

上位机发送的NAV航向始终是当前位置到正确物资区/伤员区目标点的真实几何方向，不在剩余300 mm
时替换为安全区法向。F407进入末段后应把这个方向作为平移向量，同时把车头方向单独收敛到
红方90°或蓝方270°，允许使用万向轮横移到目标点前方；只有横向位置正确后才接受最终ENTER推送。

上位机不会使用`CHANGE_LANE`代替该横向对位，F407也不要把距离小于100 mm作为理由直接锁角度并
取消横移。

```text
A3 B3 18 SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

`P0`为命令，`P1`为`VALID/RED_SIDE/DRIVE_STRAIGHT/USE_FINAL_HEADING/DISTANCE_VALID`等标志；普通导航继续使用`P2/P3=剩余距离mm`、`P6/P7=绝对航向0.01°`。

## 新命令

| 命令 | 值 | `P2/P3` | `P4/P5` | `P6/P7` |
|---|---:|---|---|---|
| `PAUSE` | `0x01` | 0 | 0 | 0 |
| `APPROACH_TARGET` | `0x09` | 目标图像X | 目标图像Y | 0 |
| `HOLD` | `0x0A` | 0 | 0 | 0 |
| `YIELD_BACKOFF` | `0x0B` | 有符号后退距离mm | 0 | 0 |
| `ESCAPE_MANEUVER` | `0x0C` | 有符号旋转角度deg | 有符号横移距离mm | 0 |
| `RELEASE_LEFT` | `0x0D` | 0 | 0 | 0 |
| `RELEASE_RIGHT` | `0x0E` | 0 | 0 | 0 |
| `RELEASE_BOTH` | `0x0F` | 0 | 0 | 0 |
| `DISPERSE_PILE` | `0x10` | 0 | 0 | 0 |
| `CHANGE_LANE` | `0x11` | 有符号横移距离mm | 有符号前进距离mm | 0 |
| `CARGO_AUDIT` | `0x12` | 左爪类别码 | 右爪类别码 | 审核信息 |

`CARGO_AUDIT`类别码：`0=空`、`1=普通`、`2=核心`、`3=伤员`、`4=危险`、`5=未知`、`6=普通+核心混合`。

审核帧中，`CARGO_AUDIT`使用字节级紧凑格式：`P2=左爪类别码`、`P3=右爪类别码`、`P4=左爪数量bits0～1/右爪数量bits2～3`、`P5=审核标志`、`P6=audit_id`、`P7=总数量`。

审核标志为：

- `P5 bit0`临时藏堆，bit1危险存在，bit2未知存在，bit3伤员混装，bit4审核稳定，bit5目的地为伤员区；
- `P2/P3`表示当前左右爪的主要类别，混合普通+核心使用类别码6；
- 下位机只有收到稳定且合法的审核后，才允许接受后续导航或投送命令。

现有命令`GRAB_CONFIRMED=0x02`、`NAVIGATE_WAYPOINT=0x03`、`ENTER_SAFE_ZONE=0x05`、`TASK_COMPLETE=0x06`、`ABORT=0x07`、`RETURN_CENTER=0x08`继续保留。

## HOLD、PAUSE、STOP和ABORT语义

`PAUSE=0x01`沿用固定15字节帧：`P0=0x01`、`P1=CMD_VALID`，其余载荷全部为0，SEQ正常递增。
例如`SEQ=0x20`时完整帧为：

```text
A3 B3 18 20 01 01 00 00 00 00 00 00 B8 B5 C3
```

- `HOLD=0x0A`是正常流程心跳。F407处于SEARCH时收到HOLD，继续本地90°/120°循环扫描；上位机等待目标或连续帧筛选时仍发送HOLD。
- `PAUSE=0x01`明确冻结当前阶段并锁存停车，用于操作员暂停、定位重建、摄像头恢复或需要保留当前动作阶段的情况；PAUSE帧过期不能自行恢复。
- `STOP=0x00`和`ABORT=0x07`不能用于普通等待、目标筛选或短暂视觉丢失。

解除PAUSE必须收到当前阶段能够接受、字段合法且SEQ更新的命令：

- SEARCH：`HOLD`；
- APPROACH：带合法X/Y的`APPROACH_TARGET`；
- NAV：带完整H/D/FLAGS的`NAVIGATE_WAYPOINT`；
- RETURN：带完整H/D/FLAGS的`RETURN_CENTER`；
- DISPERSE、YIELD、ESCAPE、CHANGE_LANE或释放动作：重发原动作命令。

阶段不匹配、字段非法或重复SEQ的命令不得解除PAUSE。DISPERSE执行中持续约1秒收到HOLD时，
F407按现有约定取消未完成打散并返回SEARCH；收到PAUSE只能冻结打散，解除时由上位机重发
`DISPERSE_PILE`继续本次动作。

F407侧需要按以下位置实现，以上位机本节语义为准：

1. `Main/Inc/vision.h`：在`VisionMissionCode`加入`VISION_CMD_PAUSE=1`；
2. `Main/Src/vision.c`：允许解析该命令，但必须校验`P1=CMD_VALID`且`P2～P7`全部为0；
3. `Main/Src/Task.c`：增加独立的暂停锁存标志，不改变原`TaskState`和远程动作类型；收到合法新SEQ
   PAUSE后ACK并`Motor_Stop()`，暂停锁存期间每个任务周期都保持停车，不能因250 ms命令过期恢复；
4. 收到非PAUSE命令时，先按暂停前的当前阶段验证命令、FLAGS和数据；只有验证通过并准备实际接受
   该命令时才清除暂停锁存。不能先解除再验证；
5. 暂停远程动作时保存暂停开始时间，解除后补偿`remote_action.started_ms`及当前阶段计时，避免PAUSE
   时间被7秒/15秒动作保护误算；
6. SEARCH解除PAUSE的HOLD只恢复本地扫描，不切换TaskState；DISPERSE解除PAUSE必须是同一动作类型
   的新SEQ `DISPERSE_PILE`，而HOLD仍按持续约1秒取消打散的现有语义执行。

## 任务硬联锁

F407应在执行层再次拒绝下列情况：

- 首件普通物资不是恰好1件绿色；
- 正式普通/核心投送批次超过3件（`initial_stash=True`临时藏堆不适用此限制）；
- 伤员与其他物资混装，或伤员数量不是1件；
- 危险或未知目标进入安全区投送动作；
- `RELEASE_LEFT/RIGHT`指定的爪子没有对应物资或动作条件不满足；
- 命令帧失联、序号/CRC非法、电机故障、IMU/编码器故障。

下位机只负责实时执行和硬联锁，不需要保存全场目标列表。上位机已经在`competition_detections.jsonl`保存所有识别结果，并通过稳定的track ID决定当前批次。

上位机的`DISPERSE_PILE`不是F407到达中心后的固定动作，而是首件搜索阶段在新鲜视觉帧中确认“看到绿色但无法单独取得”后才发出的请求。普通无目标、视觉超时、夹内复审等待和夹爪闭合时，上位机不得新发该命令；F407也必须按空爪、无复审等待再次拦截。

非法审核的推荐状态握手为：

```text
CAPTURE_AUDIT
  → RELEASE_LEFT/RIGHT（释放异常侧）或 RELEASE_BOTH
  → 新鲜mode=32/33/34
  → YIELD_BACKOFF(-250 mm)
  → 新鲜mode=30
  → 单侧释放：保留selected_batch，重新CAPTURE_AUDIT
  → 复审合法：GRAB_CONFIRMED，等待GRIPPER_CLOSED=1
  → 复审非法：RELEASE_BOTH，mode=34后再次YIELD_BACKOFF，清空批次回SEARCH
```

`RELEASE_LEFT/RELEASE_RIGHT`表示打开并把对应侧物资留在原地；不能理解为“保留左/右侧”。`mode=32/33/34`和`mode=30`必须新鲜，上位机不能用本地超时猜测动作已经完成。

## 停滞退让与脱困

当前上位机只在已确认处于真实APPROACH/NAV/RETURN平移阶段，且连续约3秒同时没有
T265平移、T265航向和有效编码器进展时，才请求一次`YIELD_BACKOFF`。短暂停顿、原地对正、
摄像头观察和动作ACK延迟不能触发退让。

退让完成后，上位机只有再次确认同样的长期无进展，才请求`ESCAPE_MANEUVER`。两类动作都必须
等待新鲜完成mode和ACK变化；动作执行较慢不能触发ABORT。只有确认越界、用户急停或明确的
不可恢复故障才允许ABORT。

下位机需要在`TYPE=0x17`状态的`mode`字段报告动作完成，建议增加以下模式号：

```text
30 YIELD_DONE
31 ESCAPE_DONE
32 RELEASE_LEFT_DONE
33 RELEASE_RIGHT_DONE
34 RELEASE_BOTH_DONE
35 DISPERSE_DONE
36 LANE_DONE
```

退让和脱困期间必须有运动看门狗；如果电流、轮速、碰撞或机构故障已经明确，直接停车，不应继续旋转。

## 验收顺序

1. 架空验证新命令解析、CRC、重复SEQ幂等和`HOLD/ABORT`区别；
2. 空载低速验证`YIELD_BACKOFF`、正负旋转和横移方向；
3. 单物资验证左右爪释放；
4. 普通+核心两件、伤员单件、危险混入四种审核组合验证硬联锁；
5. 最后再做中心拥挤物资、对手避让和完整比赛流程。

返中验收必须确认上位机持续发送指向固定中心点`(0,0)`的`RETURN_CENTER`，直到F407真正上报
`SEARCH`；不能用中心圆、600 mm圆周或`HOLD`提前切换搜索。
