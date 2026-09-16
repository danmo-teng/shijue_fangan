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

上位机在本次RETURN曾被relay发送并ACK后，收到`mode=3`只进入SEARCH准备门控：先持续发送HOLD，
等待F407摄像头到`9000/90°`，并丢弃返中完成时及相机移动期间的旧帧。只有新鲜mode3、相机90°、
视觉新鲜且frame_sequence大于返中完成帧下限时，才允许发送新的APPROACH_TARGET。中心搜索累计约
720°且没有稳定目标时，才允许前往藏点复查。

RETURN、CLUSTER_APPROACH、DISPERSE、RELEASE/YIELD、ESCAPE、两次ALIGN和ENTER等长动作统一使用
接受锁存：动作开始保存初始ACK与relay计数基线；只要匹配命令在基线后经过relay且ACK曾变化，
`command_accepted`永久置位；最终以新鲜完成mode和该锁存判断完成，不在结束时重新比较8位ACK。

### 3. 正式抓取、混装和打散

```text
SEARCH → APPROACH_TARGET → mode=20
→ mode=21夹内观察 → 稳定CARGO_AUDIT → GRAB_CONFIRMED
→ mode=22/GRIPPER_CLOSED → NAVIGATE
```

普通SEARCH候选通过现有类别、置信度、坐标范围和tracker hits检查后只需1个新帧，不再额外等待3帧。
APPROACH目标短暂消失时，上位机在`max(0.30秒, 2.5×平均视觉帧周期)`窗口内重发最后坐标，之后明确发送HOLD让F407停车；F407不得再
依赖250 ms任务帧或视觉帧超时。目标重新出现后，上位机继续发送APPROACH_TARGET。HOLD只停车并保留
当前靠近上下文，不得自行当作彻底取消；若以后需要放弃目标，双方另行增加明确CANCEL语义。

发现需要打散的聚集目标时，不允许从SEARCH直接发送`DISPERSE_PILE`，必须按以下握手执行：

```text
SEARCH
→ APPROACH_TARGET(flags=VALID|CLUSTER_TARGET，X/Y=聚集区域中心)
→ F407摄像头130°完成水平对正，再转到140°并上报mode=38
→ 上位机立即进入夹爪ROI审核；无物资时持续发送非STABLE全零CARGO_AUDIT
→ 连续稳定非空CARGO_AUDIT被ACK后，F407停车并上报mode=37
→ 审核合法：上位机持续发送GRAB_CONFIRMED
→ 需要分离且锁定目标明确在左/右爪：DISPERSE置bit6 SIDE_VALID，右侧再置bit7 TARGET_RIGHT
→ 选择性分离完成后新鲜mode=35且ACK已变化
→ 上位机进入CAPTURE_AUDIT，不发送HOLD，复审后GRAB或继续释放/YIELD
→ 锁定目标左右不明：DISPERSE_PILE不置bit6/bit7
→ F407原地转12°、相机保持140°、稳定300 ms并上报mode35
→ 上位机保留批次和track线索，只使用mode35后的新ROI帧重新审核
→ 最多执行两次无侧观察转向；仍无法分侧才发送最终RELEASE_BOTH
→ 最终RELEASE_BOTH收到本次mode34和接受证据后清空批次并回SEARCH
```

`CLUSTER_TARGET=P1 bit5(0x20)`只能出现在`APPROACH_TARGET`，P2/P3为聚集区域中心X，P4/P5为
聚集区域中心Y。F407只有完成聚集靠近并进入140°夹内观察后才上报mode38；收到稳定非空审核后
停车并上报mode37。mode37之前收到GRAB或DISPERSE必须拒绝。确认前若进入mode3或mode24，上位机
清除旧审核、待发送GRAB/DISPERSE和锁定批次，重新进入目标选择。打散过程中不要因为视觉暂时漏帧
而停止已经接受的动作；F407继续执行自身15秒动作保护。

`DISPERSE_PILE`的bit6/bit7语义为：bit6=`SIDE_VALID`，bit7=`TARGET_RIGHT`。bit7只能用于
DISPERSE且必须和bit6同时置位。bit6置位时F407曲线分离并保留指定侧；bit6未置位时执行原地12°
观察转向。两种DISPERSE都完成于mode35。

mode35后F407保持夹内复审状态，上位机持续发送CARGO_AUDIT，并用frame floor拒绝动作前审核帧。
无侧观察完成次数只在新鲜mode35且本次DISPERSE已经被接受后累计，最多两次。两次后仍无法分侧，
上位机持续发送最终RELEASE_BOTH，等待新鲜mode34和该命令接受证据后清空批次回SEARCH。

最终抓取和运输审核由1个新的`frame_sequence`直接形成显式STABLE，同一帧不得重复累计。聚集筛选
单独按本轮selected target判断类别和期望数量，默认只剩1件目标才允许GRAB，不复用允许1～3件运输
的宽松规则。曲线分离选侧使用frame floor之后第1个新鲜、非空且保留侧数量大于0的审核帧，不再
等待3帧2票。单侧绿色优先；两侧都有绿色或均无绿色时保留数量较少侧，平局依次比较selected_count、
跟踪稳定度、距离和track_id。只有非空侧无法确认，或危险/未知物资归属完全不明时，才发送无侧
DISPERSE执行12°观察。协议字段不变。

聚集APPROACH期间F407保持双爪完全打开，上位机持续发送`APPROACH_TARGET|CLUSTER_TARGET`直到新鲜
mode38，不等待GRIPPER_CLOSED。第一次带SIDE_VALID的曲线分离由F407执行15°保持，后续带侧分离执行
25°保持；上位机不增加协议位，只保留cluster_id和selected_batch，每次mode35后使用动作完成后的
第1个新帧复审，仍不合法就继续发送带侧DISPERSE。

复审得到稳定空爪时，上位机持续发送显式STABLE全零CARGO_AUDIT，直到F407新鲜mode3；随后清除
selected_batch、cluster上下文、侧向结果、旧track和frame floor，再进入SEARCH。无侧DISPERSE仍
只表示12°观察转向，最多两次，不得恢复撞击流程。

正式夹内审核第一次无法判断物资左右归属时，兼容命令仍使用`separate_then_search`上下文，但它是观察动作而不是最终释放：

```text
第一次不明侧审核
→ RELEASE_BOTH(separate_then_search)
→ F407执行兼容性的12°观察转向并上报mode=35
→ 上位机确认该RELEASE_BOTH经过relay发送且ACK曾变化，永久锁存本次接受证据
→ 保留selected_batch，累计一次观察次数并进入CAPTURE_AUDIT
→ 只使用mode35后的新夹爪ROI帧重新发送CARGO_AUDIT
→ 可以分侧时发送DISPERSE_PILE|SIDE_VALID，保留右侧时再置TARGET_RIGHT
→ 仍不能分侧且累计次数少于2时发送无侧DISPERSE_PILE再次观察
→ 累计2次后仍不能分侧才发送最终RELEASE_BOTH
→ 最终双开等待新鲜mode=34和本次命令接受证据，随后清空批次并回SEARCH
```

明确可判断单侧异常的`RELEASE_LEFT/RIGHT`仍保留原YIELD后夹内复审流程；临时藏堆到点后的
`RELEASE_BOTH`仍使用藏堆释放和返中流程，不属于撞分。

兼容观察与两种DISPERSE都在mode35后进入`CAPTURE_AUDIT`；最终RELEASE_BOTH只有在两次观察后仍
不能分侧时才执行，并在mode34后直接清空批次回SEARCH。

### 4. 正式安全区投送（2026-09-15新流程）

本节替代旧的“直接NAV到围栏并自动最终补推”流程。上位机先导航到对应半区围栏前600 mm：

```text
红方物资(-150,+540) mm，红方伤员(+150,+540) mm
蓝方物资(+150,-540) mm，蓝方伤员(-150,-540) mm
```

`NAVIGATE_WAYPOINT`的`P1 bit6=STAGE_ONLY`。进入预备点容差后，上位机持续发送
`P1=VALID|DISTANCE_VALID|STAGE_ONLY|RED_SIDE(按阵营)`、`D=0`，H仍为当前位置到预备点航向，
不置DRIVE_STRAIGHT或USE_FINAL_HEADING。上位机永久锁存本阶段D=0经过relay发送且ACK曾变化的证据；
只有锁存成立、新鲜mode10、`DISTANCE_DONE=1`且`GRIPPER_CLOSED=1`后才发送ALIGN，不在最终时刻重新
比较8位ACK。F407到`D=0`后只停车、置`DISTANCE_DONE`、把摄像头命令到120°并保持mode10，
不得启动旧的安全区最后补推。随后上位机分两次使用`ALIGN_SAFE_ZONE=0x04`：

正式投送从WAITNAV驶往预备点的`D>0`阶段也保持同一精简flags：蓝方`0x51`、红方`0x59`；普通NAV、
藏堆NAV和RETURN继续使用完整方向flags，不得一起改成精简格式。

1. `P1=VALID|USE_FINAL_HEADING|RED_SIDE`，`P6/P7=红方9000或蓝方27000`。F407按定位/IMU航向
   固定执行红方90°、蓝方270°原地对正；上位机不发送角度补偿。接收时可立即ACK，但转向期间
   继续报告mode10，真正完成后报告mode11。
2. 对正后，上位机才开始统计本方安全区。连续3个不同视觉帧识别成功后，取三个框坐标中位数并
   冻结；无镜像画面下物资区使用框宽1/3处，伤员区使用2/3处。第二个`ALIGN_SAFE_ZONE`设置
   `P1 bit6=VISUAL_CORRECTION_VALID`，`P2/P3=目标点X-640`的有符号像素误差。F407只在首次收到
   该上下文时把像素误差换算为相对转角，重复的新SEQ帧只ACK、不得重复累加转角；执行期间报告
   mode10，转向完成后重新报告mode11并锁存最终推进航向。

如果第一次定位对正完成后的5秒内没有形成连续3帧安全区，上位机跳过第二次视觉修正，直接使用
红方90°或蓝方270°的定位正方向推进；这是唯一新增超时，不得再增加视觉漏帧或对正超时。

`ENTER_SAFE_ZONE=0x05`改为：

```text
P1      VALID|DRIVE_STRAIGHT|RED_SIDE；视觉修正成功置bit6，否则置USE_FINAL_HEADING
        不置DISTANCE_VALID
P2/P3   0
P4/P5   0
P6/P7   bit6置位时为0；定位降级时为红方9000或蓝方27000
```

bit6置位时，F407保持第二次ALIGN锁存的航向；bit6未置位时使用P6/P7的红方90°或蓝方270°定位
正方向。ENTER之后上位机定位不再参与速度和终点判断，重复ENTER帧也始终保持P2/P3=0。

F407修改建议保持简单：

1. `vision.c`和`delivery_enter_command_ok()`接受上述无`DISTANCE_VALID`、D=0的ENTER格式。
2. 第一次在ALIGN完成状态接受ENTER时，只锁存一次编码器`path_mm`起点和最终航向；后续重复ENTER
   只ACK，不能重置起点。
3. 用本地编码器累计完成整段推进。按现有参数等效初值可设为`600-113+200=687 mm`，速度和减速
   继续使用现有本地参数，后续只通过实车调整这一总距离。
4. 编码器到达设定距离后立即停车，沿用现有张爪、摄像头120°和300 ms稳定流程，随后上报
   `mode=15/TASK_RAM_VERIFY`。
5. mode15中重复ENTER只ACK并保持停车；收到上位机视觉确认后的TASK_COMPLETE再执行退出安全区。

不新增任务状态、上位机推进超时或额外定位门槛。普通+核心混合物资的二次推进仍由F407本地完成。

冻结框只用于本次一次性视觉转角，不能在靠近过程中更新，也不能替代现有“物资区外→区内”投送
确认所用的实时安全区检测。上位机在NAV/ENTER阶段只保留“曾在区外”证据，收到mode15后清空区内
窗口并从新的视觉帧开始确认。每次新抓取完成、重新进入正式NAV前都会清空上一次冻结框和计数。

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
- F407的CRC和序号检查保持不变；正常Task不再依赖任务帧龄自动停车。除启动自主阶段外，
  上位机发现STM状态失联时持续发送PAUSE，状态恢复后重发当前阶段合法命令解除PAUSE。

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

二维定位导航只运行到围栏前600 mm预备点。第一次ALIGN用定位对正安全区正方向，第二次ALIGN只
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
- 序号/CRC非法、电机故障、IMU/编码器故障。

下位机只负责实时执行和硬联锁，不需要保存全场目标列表。上位机已经在`competition_detections.jsonl`保存所有识别结果，并通过稳定的track ID决定当前批次。

上位机的`DISPERSE_PILE`不是F407到达中心后的固定动作。只有聚集目标已连续稳定、上位机持续发送
带`CLUSTER_TARGET`的APPROACH；F407先上报mode38接受夹内审核，稳定非空审核后上报mode37，
此后才允许上位机发送GRAB或DISPERSE。
普通无目标、视觉超时、夹内复审等待和夹爪闭合时，上位机不得新发该命令。

非法审核的推荐状态握手为：

```text
CAPTURE_AUDIT
  → 能判断异常侧：RELEASE_LEFT/RIGHT → mode=32/33 → YIELD_BACKOFF → mode=30 → 复审
  → 第一次不能判断左右：RELEASE_BOTH(separate_then_search) → 12°观察 → mode=35
  → 保留selected_batch，只使用动作后的新ROI帧发送CARGO_AUDIT复审
  → 可以分侧：DISPERSE_PILE|SIDE_VALID → mode=35 → 再复审
  → 仍不能分侧且累计少于2次：无侧DISPERSE_PILE → mode=35 → 再复审
  → 累计2次仍不能分侧：最终RELEASE_BOTH → mode=34 → 清空批次并回SEARCH
```

`RELEASE_LEFT/RELEASE_RIGHT`表示打开并把对应侧物资留在原地；不能理解为“保留左/右侧”。所有完成
跳转必须同时满足STM新鲜、mode正确、ACK相对动作开始前已变化；仅看到relay发送成功不能当作F407接受。
LCD显示`CMD:APP REJ`时，上位机必须保留当前等待/复审阶段，不能跳到APPROACH。

左右语义必须严格保持：`RELEASE_LEFT`打开左侧、保留右侧；`RELEASE_RIGHT`打开右侧、保留左侧；
`DISPERSE+SIDE_VALID`且`TARGET_RIGHT=0`保留左侧，`TARGET_RIGHT=1`保留右侧。F407保留左侧时左爪
65°夹紧、右爪72°打开；保留右侧时左爪108°打开、右爪115°夹紧。上位机只有确认对应保留侧
count大于0才置SIDE_VALID；否则不置bit6/bit7，执行12°观察转向。

两侧都属于当前任务合规物资但仍需拆开时，依次优先：唯一包含green_supply的一侧、物体数量较少
的一侧、锁定目标数量更多的一侧、轨迹更稳定的一侧、距离更近的一侧、稳定track ID较小的一侧；
仍无法可靠判断才执行无侧标志12°观察转向；最多两次，之后最终RELEASE_BOTH。

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
38 CLUSTER_CAPTURE_AUDIT
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
