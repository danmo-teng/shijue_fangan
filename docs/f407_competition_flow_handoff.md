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
→ F407原地转20°、相机保持140°、稳定300 ms并上报mode35
→ 上位机保留批次和track线索，只使用mode35后的新ROI帧重新审核
→ 最多执行两次无侧观察转向；仍无法分侧才发送最终RELEASE_BOTH
→ 最终RELEASE_BOTH收到本次mode34和接受证据后清空批次并回SEARCH
```

`CLUSTER_TARGET=P1 bit5(0x20)`只能出现在`APPROACH_TARGET`，P2/P3为聚集区域中心X，P4/P5为
聚集区域中心Y。F407只有完成聚集靠近并进入140°夹内观察后才上报mode38；收到稳定非空审核后
停车并上报mode37。mode37之前收到GRAB或DISPERSE必须拒绝。确认前若进入mode3或mode24，上位机
清除旧审核、待发送GRAB/DISPERSE、最后APPROACH坐标和锁定批次，记录恢复frame floor并持续HOLD，
只从mode3之后的新视觉帧重新进入目标选择。普通APPROACH目标超过动态缺帧窗口后也持续发送HOLD；
目标在mode24前恢复时发送新的APPROACH_TARGET可取消恢复。打散过程中不要因为视觉暂时漏帧
而停止已经接受的动作；F407继续执行自身15秒动作保护。

`DISPERSE_PILE`的bit6/bit7语义为：bit6=`SIDE_VALID`，bit7=`TARGET_RIGHT`。bit7只能用于
DISPERSE且必须和bit6同时置位。bit6置位时F407曲线分离并保留指定侧；bit6未置位且bit5未置位时
执行原地20°观察转向。这两种普通DISPERSE完成于mode35；bit5首件轻撞完成后直接mode3。

mode35后F407保持夹内复审状态，上位机持续发送CARGO_AUDIT，并用frame floor拒绝动作前审核帧。
无侧观察完成次数只在新鲜mode35且本次DISPERSE已经被接受后累计，最多两次。两次后仍无法分侧，
上位机持续发送最终RELEASE_BOTH，等待新鲜mode34和该命令接受证据后清空批次回SEARCH。

普通mode21以及聚集/分离复审后最终允许GRAB，都要求2个不同新帧内容一致；同一帧不得重复累计。
首件正式绿色完成前仍要求恰好1件绿色；首件完成后的material允许夹内实际1～3件普通、核心或
MIXED_MATERIAL，伤员仍须单独1件，不再要求与APPROACH前selected_batch完全一致。无效审核的曲线
分离选侧仍使用frame floor之后第1个新鲜、非空审核帧。仅左侧有绿色保留左，仅右侧有绿色保留右；
两侧都有绿色时保留绿色数量较少侧，相同保留左；两侧都无绿色时保留左，只有左空才保留右。普通
混装、数量相同、track接近不再触发无侧观察。只有overall ROI内完全没有可强制归侧候选时，才发送
无侧DISPERSE执行20°观察。协议字段不变。

聚集APPROACH期间F407保持双爪完全打开，上位机持续发送`APPROACH_TARGET|CLUSTER_TARGET`直到新鲜
mode38，不等待GRIPPER_CLOSED。所有带SIDE_VALID的DISPERSE曲线都使用固定15°保持；25°只属于普通
RELEASE_LEFT/RIGHT单侧释放。每次mode35后使用动作完成后的新帧复审。

新增`DISPERSE_PILE P1 bit5=FIRST_GREEN_BUMP`，仅用于首件正式绿色尚未完成的中心混堆轻撞，且不能
与SIDE_VALID/TARGET_RIGHT同时置位。F407接受后ACK并进入mode25，固定执行：后退0.10 m→Touch闭爪
→前进0.20 m→后退0.10 m→双开；完成后直接进入mode3，不上报mode35。重复新SEQ同命令只ACK且
不重启动作。上位机确认本次接受证据和新鲜mode3后清除旧目标并用新帧重搜。首件绿色完成后拒绝bit5。

普通单目标按新版流程：F407在125°水平对正后直接转到140°，稳定后进入mode21并置CLAW_VISIBLE，
不再等待原track重新出现；随后以180 mm/s最多慢爬500 mm。上位机收到新鲜mode21+CLAW_VISIBLE后
停止APPROACH并记录frame floor。普通抓取要求frame floor之后2个不同frame_sequence内容一致：第一
帧发非STABLE审核，第二帧发STABLE审核；每个新视觉帧使用新audit_id，同一帧重复发送保持audit_id。
聚集mode38/mode37和分离复审中的合法GRAB同样使用2帧；无效审核的分侧决策仍保持1帧。

最终GRAB两帧的一致性需要忽略单件目标位于左爪还是右爪，只比较总数、实际类别集合以及危险/未知/
混装标志。F407的`task_latch_audit()`也应按该归一化语义累计普通两帧；显式STABLE第二帧不得因为左右
抖动被本地重新清零。上位机出现第一张合法候选后会忽略一张瞬时无效帧，连续2张无效才重新分离；
稳定空爪仍按原流程回SEARCH。

SEARCH顺序固定为120°一圈再90°一圈。F407累计完成约720°且仍无目标后保持mode3并进入
SEARCH_WAIT_RETURN；收到合法RETURN_CENTER后ACK并进入mode17。上位机持续发送实时H/D到D=0，
F407重新mode3后从120°开始新一轮搜索。任何阶段收到合法APPROACH都应立即退出等待并接管目标。

上位机只在STM新鲜、CLAW_VISIBLE=1且`camera_pitch_cdeg==14000`时建立夹内审核；全局cargo仅服务
SEARCH/APPROACH。每次进入mode21、mode38或收到本次新鲜mode35后都会清除动作前审核和选侧缓存，
设置新frame floor，只使用`frame_sequence > frame_floor`的140°ROI帧。F407的mode35必须保持
CLAW_VISIBLE=1和相机140°，直到接受新的CARGO_AUDIT。

左右绿色数量是上位机本地字段，不进入UART。选侧固定为：仅左绿保留左，仅右绿保留右；两侧有绿
保留绿色数量较少侧，相同保留左；两侧无绿保留左，只有左空才保留右。只要overall ROI内存在任何
可归侧候选就发送SIDE_VALID，右侧再置TARGET_RIGHT；普通物资混装、两侧数量相同或track接近不再
触发无侧观察。无侧DISPERSE只保留给overall ROI内没有可强制归侧候选的异常情况。

大ROI负责确认“物体存在”：满足底部中心和重叠条件的所有物体都进入total_count。左右爪ROI只负责
分离方向；无法分侧的物体设置unknown_present并保留在total_count，使审核无法GRAB并走无侧20°观察。
首件正式绿色阶段只有总数为1且为绿色时才能GRAB；首件完成后的material可按夹内实际1～3件普通/
核心组合GRAB，伤员仍须单独1件。最终合法审核会把实际类别、数量和material/injury目的地写回
selected_batch；非法审核继续分离。

复审得到稳定空爪时，上位机持续发送显式STABLE全零CARGO_AUDIT，直到F407新鲜mode3；随后清除
selected_batch、cluster上下文、侧向结果、旧track和frame floor，再进入SEARCH。无侧DISPERSE仍
只表示20°观察转向，最多两次，不得恢复撞击流程。

正式夹内审核第一次无法判断物资左右归属时，`separate_then_search`上下文使用无侧DISPERSE观察，不能复用最终双开的RELEASE_BOTH：

```text
第一次不明侧审核
→ DISPERSE_PILE（不置SIDE_VALID/TARGET_RIGHT）
→ F407执行兼容性的20°观察转向并上报mode=35
→ 上位机确认该DISPERSE经过relay发送且ACK曾变化，永久锁存本次接受证据
→ 保留selected_batch，累计一次观察次数并进入CAPTURE_AUDIT
→ 只使用mode35后的新夹爪ROI帧重新发送CARGO_AUDIT
→ 可以分侧时发送DISPERSE_PILE|SIDE_VALID，保留右侧时再置TARGET_RIGHT
→ 仍不能分侧且累计次数少于2时发送无侧DISPERSE_PILE再次观察
→ 累计2次后仍不能分侧才发送最终RELEASE_BOTH
→ 最终双开等待新鲜mode=34和本次命令接受证据，随后清空批次并回SEARCH
```

明确可判断单侧异常的`RELEASE_LEFT/RIGHT`仍保留原YIELD后夹内复审流程；临时藏堆到点后的
`RELEASE_BOTH`仍使用藏堆释放和返中流程，不属于撞分。

无侧观察与带侧DISPERSE都在mode35后进入`CAPTURE_AUDIT`；最终RELEASE_BOTH只有在两次观察后仍
不能分侧时才执行，并在mode34后直接清空批次回SEARCH。

### 4. 正式安全区投送（2026-09-15新流程）

本节替代旧的“直接NAV到围栏并自动最终补推”流程。上位机先导航到对应半区围栏前600 mm：

```text
红方物资(-150,+540) mm，红方伤员(+150,+540) mm
蓝方物资(+150,-540) mm，蓝方伤员(-150,-540) mm
```

`NAVIGATE_WAYPOINT`的`P1 bit6=STAGE_ONLY`。进入预备点容差后，上位机持续发送
`P1=VALID|DISTANCE_VALID|STAGE_ONLY|RED_SIDE(按阵营)`、`D=0`，H仍为当前位置到预备点航向，
不置DRIVE_STRAIGHT或USE_FINAL_HEADING。新鲜mode10、`DISTANCE_DONE=1`且`GRIPPER_CLOSED=1`
即可锁存预备点完成并发送ALIGN，不再把当前8位ACK与阶段初始ACK比较作为永久门槛。F407到`D=0`
后只停车、置`DISTANCE_DONE`、把摄像头命令到120°并保持mode10，
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

第二次视觉ALIGN完成后，上位机重新记录frame floor，只使用转向完成后新视觉帧中的新鲜safe bbox
建立车头到对应安全半区入口之间的推进走廊；新框到达前继续等待，不能退回使用视觉修正前冻结框或
短时缓存框。走廊排除安全区框内目标、本轮`delivery_items`和当前携带/锁定批次track ID。底部20%
夹爪近场目标只有在完整类别/数量与carried manifest一致，或与最近确认携带框纵向重叠至少60%时
才能排除；与携带清单不同类别的危险物或伤员仍必须作为走廊障碍。
首件正式绿色投送时走廊内任意物体触发`CLEAR_SAFE_ZONE=0x13`；之后只在危险物或伤员挡路时触发。
`P2/P3`为视觉纵向距离减去`safe_sweep_capture_offset_mm`后裁剪到80～600 mm的实际前进距离；该配置
包含相机/定位参考点到夹爪入口的机械距离和入爪余量，默认150 mm，需按实车标定。`P4/P5`为暂放
横移（物资/核心`+150`，伤员`-150`），`P6/P7=0`。上位机确认本次CLEAR已ACK后，只要看到新鲜
`mode23+GRIPPER_CLOSED+CLAW_VISIBLE`就进入扫障后复审；是否曾采到mode39只用于诊断，不是门槛。
F407重新完成3个不同audit_id审核，合法后进入mode40。mode40不能直接ENTER，必须重新执行定位ALIGN
和视觉ALIGN。每趟最多扫障2次。

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
3. 用F407当前已有的本地编码器流程完成整段推进；上位机不发送实时围栏距离，也不参与本地推进
   距离、速度或终点判断。
4. 编码器到达设定距离后立即停车，沿用现有张爪、摄像头120°和300 ms稳定流程，随后上报
   `mode=15/TASK_RAM_VERIFY`。
5. mode15中重复ENTER只ACK并保持停车；收到上位机视觉确认后的TASK_COMPLETE再执行退出安全区。

不新增任务状态、上位机推进超时或额外定位门槛。普通+核心混合物资的二次推进仍由F407本地完成。

投送完成后F407先本地后退0.30 m并上报mode16，再进入mode17。上位机看到新鲜mode16或mode17后
立即持续发送原有RETURN_CENTER H/D；F407在mode16继续完成本地后退，在mode17开始接受RETURN，
不得把提前到达的RETURN当作HOLD或用它中断本地退出。

为避免mode16期间曾收到PAUSE后无法恢复，F407在TASK_EXIT_SAFE_ZONE收到字段合法的新SEQ
RETURN_CENTER时应ACK并保存最新H/D，但继续执行本地0.30 m后退，不提前切换状态。ACK用于解除PAUSE；
后退完成进入mode17后直接使用已保存的最新RETURN。重复RETURN只更新H/D，不重置后退距离。

当前F407还应补齐两项会影响连续启动和维护的一致性：

- `TASK_STOPPED + TASK_FAULT_REMOTE_STOP`收到一组新的合法赛前配置时，视为操作员明确启动新一轮任务，
  清除REMOTE_STOP并重新执行任务初始化/自主出发。电机、IMU、非法状态等其他fault仍要求人工复位，
  不能被配置帧清除。
- 同步更新`tools/vision_protocol.py`、`tools/test_vision_protocol.py`和`MISSION_PROTOCOL.md`中的旧ENTER
  距离格式、旧聚集mode说明及普通单目标单帧STABLE说明，使参考工具与实际C固件一致。无侧DISPERSE
  的注释统一为20°观察转向，不再写“整堆撞击”。

冻结框只用于本次一次性视觉转角，不能在靠近过程中更新，也不能用于最终推进走廊或替代现有“物资
区外→区内”投送确认所用的实时安全区检测。上位机在NAV/ENTER阶段只保留“曾在区外”证据，收到mode15后清空区内
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
| `CLEAR_SAFE_ZONE` | `0x13` | 前进距离80～600 mm | 暂放横移±150 mm | 0 |

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
  → 第一次不能判断左右：无侧DISPERSE_PILE → 20°观察 → mode=35
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
count大于0才置SIDE_VALID；否则不置bit6/bit7，执行20°观察转向。

两侧都属于当前任务合规物资但仍需拆开时，依次优先：唯一包含green_supply的一侧、物体数量较少
的一侧、锁定目标数量更多的一侧、轨迹更稳定的一侧、距离更近的一侧、稳定track ID较小的一侧；
仍无法可靠判断才执行无侧标志20°观察转向；最多两次，之后最终RELEASE_BOTH。

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
39 SAFE_SWEEP
40 SAFE_SWEEP_DONE
41 BOUNDARY_RECOVER
```

mode41由F407按普通阶段距地图边300 mm、ENTER推进距边50 mm的本地阈值触发。上位机收到后清除
当前批次、carried manifest、track、审核、NAV/ALIGN/ENTER和扫障上下文，只发送HOLD而不重发旧动作；
F407张爪、转向场地中心并继续驶入距边至少400 mm后才回mode3。上位机必须等新鲜mode3，并只从该
动作之后的新视觉帧重新SEARCH。

退让和脱困期间必须有运动看门狗；如果电流、轮速、碰撞或机构故障已经明确，直接停车，不应继续旋转。

## 验收顺序

1. 架空验证新命令解析、CRC、重复SEQ幂等和`HOLD/ABORT`区别；
2. 空载低速验证`YIELD_BACKOFF`、正负旋转和横移方向；
3. 单物资验证左右爪释放；
4. 普通+核心两件、伤员单件、危险混入四种审核组合验证硬联锁；
5. 最后再做中心拥挤物资、对手避让和完整比赛流程。

返中验收必须确认上位机持续发送指向固定中心点`(0,0)`的`RETURN_CENTER`，直到F407真正上报
`SEARCH`；不能用中心圆、600 mm圆周或`HOLD`提前切换搜索。

## 2026-09-17：合爪前后双重审核（最新要求，覆盖旧抓取审核描述）

上位机现在要求最新版F407提供`mode=23/POST_GRAB_AUDIT`和TYPE `0x17` flags bit4
`AUDIT_VALID`。未适配这两个字段的旧固件不会被上位机误判为审核成功，也不会进入NAV。

请严格按以下流程修改F407，不改变15字节帧格式和现有命令编号：

1. `Main/Inc/vision.h`增加`VISION_STM_AUDIT_VALID = 0x10U`；`Task.c`发布TYPE `0x17`
   时，只有当前已接收审核且`audit_valid=true`才置bit4。ACK只表示收到命令，不能代替该位。
2. `Main/Inc/Task.h`增加`TASK_POST_GRAB_AUDIT=23`。协议mode23表示已经合爪、摄像头保持140°、
   `GRIPPER_CLOSED=1`、`CLAW_VISIBLE=1`、底盘停车并等待合爪后的新审核；mode22继续只表示最终审核
   已通过、允许接收NAV。
3. 合爪前普通mode21、聚集mode38/mode37、分离复审和mode23合爪后复审全部要求3个不同
   `audit_id`且语义签名一致。同一审核帧以不同任务SEQ重复发送时不能累计。首件合法审核归一化为
   `FIRST_GREEN`，后续合法物资归一化为`MATERIAL_LEGAL+total_count`，单伤员归一化为
   `INJURY_SINGLE`；非法审核归一化为`INVALID+total_count+DANGER_PRESENT+UNKNOWN_PRESENT+
   INJURY_MIXED+DESTINATION_INJURY`，不比较左右位置以及绿色、核心、mixed的精确组成。任何语义
   不一致审核立即以当前帧重新从1计数。上位机累计3帧后若仍未看到`AUDIT_VALID`，会继续消费真实
   140°夹内新帧并生成新的`audit_id`，不会永久重复最后一个ID。
4. 合法条件：首件正式绿色未完成时必须恰好1个绿色；首件完成后，恰好1个伤员合法，或1～3件
   普通/核心/普通核心混合合法；危险、未知、超过3件、伤员混装均非法。`DESTINATION_INJURY`
   必须与本次实际审核一致：单伤员置1，合法物资置0。不能沿用APPROACH前的目标类别判断审核。
5. F407收到已通过合爪前审核的`GRAB_CONFIRMED`后，如果审核中包含核心物资，先锁存当前航向，
   以近距离慢速和编码器累计向前50 mm，再停车执行`Claw_Touch()`；不含核心时直接合爪。
   重复`GRAB_CONFIRMED`只ACK并保持动作幂等，不能重复启动50 mm。
6. `Claw_Touch()`完成后不要直接进入mode22。清除合爪前审核缓存，进入mode23，保持相机140°和
   双爪当前角度，等待上位机从新的frame floor发送合爪后3帧审核。
7. mode23收到合法STABLE审核并确认本地`audit_valid=true`后，ACK该审核并进入mode22；收到稳定
   空爪审核时双爪打开并回mode3；收到稳定非法审核时保持mode23并允许现有
   `RELEASE_LEFT/RIGHT/BOTH`或`DISPERSE_PILE`处理。单侧释放完成后仍沿用YIELD和140°复审流程。
8. `GRAB_CONFIRMED`只允许在合爪前审核已满足3帧且`AUDIT_VALID=1`的mode21或mode37接受。
   审核非法时保持原状态并显示`GRAB REJ`，但上位机按新流程不会发送这种命令。
9. mode23允许执行一次有限视觉恢复：先在140°等待审核，未形成稳定结果时依次短暂到138°、回140°、
   再到142°、回140°。138°/142°期间上位机会持续发送同一audit_id的非STABLE全零审核，不能把它
   当成取消、HOLD或PAUSE。每次相机离开或重新到达140°，F407和上位机都必须清除旧连续计数；
   只能使用本次回到140°之后的新audit_id重新累计3帧。
10. 有限恢复最终失败时F407双开并直接回mode3。上位机在mode23或待确认审核阶段看到新鲜mode3，
    会清除selected_batch、carried manifest、track、审核和frame floor，并等待动作后的新帧重新SEARCH。
11. mode23形成连续3帧非法审核后保持停车并等待释放/分离命令4秒；上位机在非法审核ACK后会立即
    发送动作。窗口结束仍未收到合法动作时，F407双开并回mode3，不能进入mode22或沿用旧审核。

完整握手：

```text
mode21或mode38
→ 合爪前连续3帧CARGO_AUDIT
→ F407置AUDIT_VALID
→ GRAB_CONFIRMED
→ 若含核心则编码器前进50 mm
→ Claw_Touch
→ mode23（清除旧审核）
→ 合爪后连续3帧CARGO_AUDIT
→ 合法：AUDIT_VALID=1并进入mode22
→ 上位机按合爪后实际清单发送NAV
```

## 2026-09-18：对照F407 16c4b77的流程适配

本次只读核对下位机 `16c4b77` 的 `Main/Src/Task.c` 和
`Main/Inc/app_config.h`，仅修改上位机。

- 投送进入新鲜mode15并确认ENTER已接受后，视觉观察最多1秒。
  提前确认则立即发送TASK_COMPLETE；超时同样进入TASK_COMPLETE，
  不再第二轮观察，完成依据记录为 `delivery_visual_timeout`，不能记为视觉确认成功。
  F407的 `APP_DELIVERY_VERIFY_WAIT_MS=1200`：上位机1秒切换后持续发送完成命令，
  F407到自身1200 ms门槛才实际进入mode16。因此上位机修改不能把实际退出压到1秒以内。
- F407聚集mode38只有收齐3个不同audit_id才进入mode37。上位机待确认期间继续
  消费真实新帧，包括非法聚集审核；稳定空爪待回SEARCH同样续ID。
  重复同一视觉帧不生成新ID，避免丢包后双方永久等待。
- F407在释放/分离后的 `cargo_recheck_pending` 阶段只接受RELEASE_BOTH。
  此阶段危险复审直接双侧释放；首次危险审核仍按明确左右侧释放并YIELD复审，
  不改为FIRST_GREEN_BUMP或DISPERSE。

STAGE、两次ALIGN、转向后新框走廊检查、0x13及mode39/23/40、mode41
继续按现有协议运行。缺标定和走廊不可用的门禁保持现有行为。
