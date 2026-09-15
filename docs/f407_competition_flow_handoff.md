# 完整智能救援流程的 F407 配合说明

本文是上位机新分支的接口说明，不是F407补丁。F407仓库不得由上位机仓库自动修改或生成补丁。

上位机仍通过定位进程的原子`uart_command.bin`发送固定15字节`TYPE=0x18`帧：

> **当前配合版本说明（2026-09-15，以上位机`codex/gamepad-teleop`最新提交为准）**
>
> 本节是当前F407配合合同，优先级高于本文后面的历史设计文字。F407仓库仍由下位机负责人
> 自行修改；上位机仓库不生成或应用F407补丁。命令编号、15字节帧、CRC、序号和现有正常
> 抓取/导航动作均保持不变。实现只允许完成下列明确事项，不要自行增加新的超时、重试、
> 自动重启或坐标修正逻辑。

## 当前上位机流程与F407最小配合要求

### 1. 开局统一藏物资

上位机在F407进入`mode=3 SEARCH`后，只要识别到稳定物资，就统一建立`initial_stash=True`批次：

```text
单独绿色、其它单件或聚集物资 → 全部执行initial_stash临时搬堆
无可靠目标 → 继续SEARCH
```

开局阶段不再存在“单独绿色直接正式投送”分支。F407按`initial_stash`标志完成抓取、藏点NAV和
双开释放，之后等待上位机`RETURN_CENTER`；开局藏物资不使用正式搜索阶段的mode37打散握手。
开局没有稳定目标时，上位机持续发送HOLD并保持`INITIAL_OBSERVE`，让F407继续本地90°/120°扫描；
不得通过本地观察超时设置`initial_stash_done`或提前进入正式SEARCH。只有收到本次藏点
`RELEASE_BOTH`的新鲜mode34和ACK变化后，上位机才把开局藏物资标记为完成并开始返中。

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

普通SEARCH候选通过现有类别、置信度、坐标范围和tracker hits检查后只需1个新帧，不再额外等待3帧。
APPROACH目标短暂消失时，上位机最多150 ms重发最后坐标，之后明确发送HOLD让F407停车；F407不得再
依赖250 ms任务帧或视觉帧超时。目标重新出现后，上位机继续发送APPROACH_TARGET。HOLD只停车并保留
当前靠近上下文，不得自行当作彻底取消；若以后需要放弃目标，双方另行增加明确CANCEL语义。

发现需要打散的聚集目标时，不允许从SEARCH直接发送`DISPERSE_PILE`，必须按以下握手执行：

```text
SEARCH
→ APPROACH_TARGET(flags=VALID|CLUSTER_TARGET，X/Y=聚集区域中心)
→ 持续发送并等待F407新鲜mode=37且ACK已变化
→ 上位机用3个新视觉帧确认原目标位于聚集区域左侧、右侧或无法判断
→ 左/右明确：DISPERSE_PILE置bit6 SIDE_VALID，右侧再置bit7 TARGET_RIGHT
→ 选择性分离完成后新鲜mode=35且ACK已变化
→ 上位机进入CAPTURE_AUDIT，不发送HOLD，复审后GRAB或继续释放/YIELD
→ 左右无法判断：DISPERSE_PILE不置bit6/bit7
→ 整堆撞分完成后新鲜mode=34且ACK已变化
→ 上位机进入DISPERSE_RESELECT，原地重新关联原目标，不立即发送HOLD
```

`CLUSTER_TARGET=P1 bit5(0x20)`只能出现在`APPROACH_TARGET`，P2/P3为聚集区域中心X，P4/P5为
聚集区域中心Y。F407只有接受该命令并完成聚集靠近后才上报mode37；mode37之前收到DISPERSE必须
拒绝。打散过程中不要因为视觉暂时漏帧而停止已经接受的动作；F407继续执行自身15秒动作保护。

`DISPERSE_PILE`的bit6/bit7语义为：bit6=`SIDE_VALID`，bit7=`TARGET_RIGHT`。bit7只能用于
DISPERSE且必须和bit6同时置位。bit6置位时F407只分离并保留指定侧目标，完成报告mode35；bit6未
置位时执行整堆撞分，完成报告mode34。所有完成都必须对应本次DISPERSE的ACK。

mode35后F407保持夹内复审状态，上位机持续发送CARGO_AUDIT。mode34后F407保持原地等待重选；上位机
找到已经独立的原目标时直接发送普通APPROACH，仍聚集且当前聚集目标撞分少于2次时重新发送
CLUSTER_TARGET。约1.25秒且取得至少3个新帧仍无法关联时，上位机才发送HOLD回普通SEARCH。

正式夹内审核第一次无法判断物资左右归属时，使用`separate_then_search`：

```text
第一次不明侧审核
→ RELEASE_BOTH(separate_then_search)
→ F407内部完成双开、后退0.40m、Touch闭爪、700mm/s前撞0.40m、450mm/s后退0.40m
→ 双爪重新完全打开并清除下位机复审标志
→ 新鲜mode=34且ACK已变化
→ 上位机清除selected_batch、cargo_recheck_pending和全部旧审核累计
→ 上位机进入SEARCH并持续发送HOLD，不发YIELD/CARGO_AUDIT/GRAB/APPROACH
→ 等待F407新鲜mode=3
→ mode=3后从新的视觉帧重新选择撞散目标
```

明确可判断单侧异常的`RELEASE_LEFT/RIGHT`仍保留原YIELD后夹内复审流程；临时藏堆到点后的
`RELEASE_BOTH`仍使用藏堆释放和返中流程，不属于撞分。

撞分分支在mode34后不会进入`CAPTURE_AUDIT`，因此空爪时`capture_audit=None`不能再触发清零并
永久停留WATCH；`CAPTURE_AUDIT`中的无观测等待只保留给真实夹内审核和单侧释放后的复审。

### 4. 正式安全区投送（2026-09-15新流程）

本节替代旧的“直接NAV到围栏并自动最终补推”流程。上位机先导航到对应半区围栏前400 mm：

```text
红方物资(-150,+740) mm，红方伤员(+150,+740) mm
蓝方物资(+150,-740) mm，蓝方伤员(-150,-740) mm
```

`NAVIGATE_WAYPOINT`的`P1 bit6=STAGE_ONLY`。上位机进入预备点地图容差后会锁存并持续发送同一帧
`D=0`，只有确认该D=0经过relay发送、ACK变化、新鲜mode10、`DISTANCE_DONE=1`且
`GRIPPER_CLOSED=1`后才发送ALIGN。F407到`D=0`后只停车、置`DISTANCE_DONE`并保持mode10，
不得启动旧的安全区最后补推。随后上位机分两次使用`ALIGN_SAFE_ZONE=0x04`：

1. `P1=VALID|USE_FINAL_HEADING|RED_SIDE`，`P6/P7=红方9000或蓝方27000`。F407按定位/IMU航向
   原地对正；接收时可立即ACK，但转向期间继续报告mode10，真正完成后报告mode11。
2. 对正后，上位机才开始统计本方安全区。连续3个不同视觉帧识别成功后，取三个框坐标中位数并
   冻结；无镜像画面下物资区使用框宽1/3处，伤员区使用2/3处。第二个`ALIGN_SAFE_ZONE`设置
   `P1 bit6=VISUAL_CORRECTION_VALID`，`P2/P3=目标点X-640`的有符号像素误差。F407只在首次收到
   该上下文时把像素误差换算为相对转角，重复的新SEQ帧只ACK、不得重复累加转角；执行期间报告
   mode10，转向完成后重新报告mode11并锁存最终推进航向。

如果第一次定位对正完成后的5秒内没有形成连续3帧安全区，上位机跳过第二次视觉修正，直接使用
红方90°或蓝方270°的定位正方向推进；这是唯一新增超时，不得再增加视觉漏帧或对正超时。

`ENTER_SAFE_ZONE=0x05`改为：

```text
P1      VALID|DRIVE_STRAIGHT|DISTANCE_VALID|RED_SIDE；视觉修正成功置bit6，否则置USE_FINAL_HEADING
P2/P3   车体旋转中心到y=±1140 mm围栏直线的法向距离mm
P4/P5   0
P6/P7   bit6置位时为0；定位降级时为红方9000或蓝方27000
```

bit6置位时，F407只能保持第二次ALIGN锁存的航向；定位距离只调节前进速度，不能修改方向。bit6未
置位时，F407使用P6/P7的定位正方向降级推进。到机构理论位置后，F407锁存最后补推，忽略后续定位
距离变化，以250 mm/s使用编码器继续前进50 mm，完成后停车并报告mode15。推进速度、接近减速以及
普通+核心混合物资的二次推进仍全部由F407本地完成。

冻结框只用于本次一次性视觉转角，不能在靠近过程中更新，也不能替代现有“物资区外→区内”投送
确认所用的实时安全区检测。每次新抓取完成、重新进入正式NAV前，上位机会清空上一次冻结框和计数。

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
- `ALIGN_SAFE_ZONE`只能按第4节的新两段对正语义实现，不得恢复旧的围栏边强制锁角推进；
- 不新增上位机或F407重复的几百毫秒级目标/动作超时；
- 不修改场地坐标、安全区坐标、出发区坐标和定位轴参数；
- 不把视觉漏帧、短暂停顿或单次ACK延迟当作电机故障；
- 不自动重启比赛、不自动清除锁存故障。

### 9. 安全区末段方向与距离分工

二维定位导航只运行到围栏前400 mm预备点。第一次ALIGN用定位对正安全区正方向，第二次ALIGN只
执行冻结框算出的单次像素转角。进入ENTER以后，上位机不再发送到半区中心点的二维距离或动态
航向，只发送到围栏所在直线的法向距离。F407保持锁存航向直推，不能用位置坐标重新修正方向。

```text
A3 B3 18 SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

`P0`为命令，`P1`为`VALID/RED_SIDE/DRIVE_STRAIGHT/USE_FINAL_HEADING/DISTANCE_VALID/CLUSTER_TARGET`等标志；普通导航继续使用`P2/P3=剩余距离mm`、`P6/P7=绝对航向0.01°`。`CLUSTER_TARGET=bit5`只允许用于`APPROACH_TARGET`。`bit6`按命令区分：NAV表示`STAGE_ONLY`，ALIGN/ENTER表示`VISUAL_CORRECTION_VALID`，DISPERSE表示`SIDE_VALID`；`bit7=TARGET_RIGHT`只能用于DISPERSE且必须同时置bit6。

## 新命令

| 命令 | 值 | `P2/P3` | `P4/P5` | `P6/P7` |
|---|---:|---|---|---|
| `PAUSE` | `0x01` | 0 | 0 | 0 |
| `ALIGN_SAFE_ZONE` | `0x04` | 定位对正时0；视觉修正时有符号像素误差 | 0 | 定位对正航向或0 |
| `APPROACH_TARGET` | `0x09` | 目标或聚集区域中心X | 目标或聚集区域中心Y | 0 |
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

上位机的`DISPERSE_PILE`不是F407到达中心后的固定动作。只有聚集目标已连续稳定、上位机持续发送
带`CLUSTER_TARGET`的APPROACH，而且F407用新鲜mode37和本次ACK确认靠近完成后才允许发送。
普通无目标、视觉超时、夹内复审等待和夹爪闭合时，上位机不得新发该命令。

非法审核的推荐状态握手为：

```text
CAPTURE_AUDIT
  → 能判断异常侧：RELEASE_LEFT/RIGHT → mode=32/33 → YIELD_BACKOFF → mode=30 → 复审
  → 第一次不能判断左右：RELEASE_BOTH(separate_then_search) → mode=34
  → 清空selected_batch和审核状态 → HOLD等待mode=3 → 重新搜索撞散后的目标
  → 该路径禁止YIELD、CARGO_AUDIT、GRAB_CONFIRMED和提前APPROACH_TARGET
```

`RELEASE_LEFT/RELEASE_RIGHT`表示打开并把对应侧物资留在原地；不能理解为“保留左/右侧”。所有完成
跳转必须同时满足STM新鲜、mode正确、ACK相对动作开始前已变化；仅看到relay发送成功不能当作F407接受。
LCD显示`CMD:APP REJ`时，上位机必须保留当前等待/复审阶段，不能跳到APPROACH。

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
37 CLUSTER_READY
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
