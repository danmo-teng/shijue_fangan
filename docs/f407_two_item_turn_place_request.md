# 可直接复制给F407：两件上限、mode41修复、转向放置与限距找回

本次只读核对下位机GitHub HEAD `c71efd6`（feat: add vision-guided safe-zone obstacle pickup）。
其mode42/43、扫障审核bit6已经实现，但仍采用150 mm横移暂放、普通/核心最多3件，
HOLD接收分支仍会打断mode41转向。本文件覆盖旧扫障说明中的对应条款。
上位机已按下面的报文契约修改；F407尚需由下位机负责人实施。不要直接照旧协议烧录联调。

## 一、正式运输最多两件，必须上下位机同步

1. 开局INITIAL_STASH=1仍优先按STASH_NONEMPTY处理，任意非空均可藏堆，
   不受正式两件上限影响，且藏堆完成不置first_delivery_done。
2. 首次正式投送仍只能恰好1件GREEN，不允许CORE替代、不允许两件。
3. 首次完成后，GREEN/CORE/两者混合合计只能1～2件；伤员仍只能单独1件。
4. 修改Task.c的task_audit_semantic：开局分支之后的total>3改为total>2；
   同步检查所有GRAB、mode23、NAV放行和扫障取回审核，禁止硬编码3件合法。
   3个不同audit_id的审核帧数不变，左右两位计数字段不变，不能把实际3/4件截为2。
5. 超量审核必须置AUDIT_VALID=0并留在可接受分离/释放的审核状态。
   不得提前mode22，否则上位机等分离、F407等NAV会卡死。
   mode43是专用障碍清理，仍非空即可；mode45找回原物资必须恢复正式规则。

## 二、mode41的HOLD只ACK，不能中断自主恢复

c71efd6的Task_Process虽已豁免mode41的普通HOLD拦截，task_accept_mission中
HOLD仍落入Motor_Stop/nav_ready=false分支。motor.c的Motor_Stop会将
angle_turn.status重置为IDLE，导致每帧HOLD都取消转向，下一轮重新起转。

请在task_accept_mission的HOLD分支对TASK_BOUNDARY_RECOVER单独处理：
- 更新ACK和心跳，但不调用Motor_Stop、不清nav_ready、不清boundary_turn_done，
  不重设转向原点、编码器原点、step_started_ms或已完成张爪标志。
- 保留现有公共ACK解除PAUSE处理，避免新增早return反而让PAUSE无法解除。
- mode41只在进入时初始化一次，完成张爪→转向中心→驶入→mode3。
- 上位机仍持续HOLD，不通过停止心跳绕过问题。
- 增加诊断：下位机Location x/y/yaw、触发边距/阈值、恢复目标航向、
  boundary_turn_done、累计位移。上位机T265坐标不等于F407内部编码器坐标。

## 三、0x13新放置语义：转向后直行200 mm，不再横移

视觉CLEAR报文：TYPE=0x18，P0=0x13，P1=VALID|RED_SIDE，P2/P3=0，P6/P7=0。
P4/P5：伤员=-200（原物资暂放在左侧），普通/核心=+200（暂放在右侧）。
绝对值200表示转向后前进的放置距离，不是底盘横移量。

同时修改vision.c的CLEAR解析校验和Task.c的task_safe_sweep_command_valid：
视觉请求arg_a=0时要求abs(arg_b)=200；旧距离式请求可另保留原±150兼容。
不能只修改Task.c，否则vision.c会提前丢包。重复CLEAR只ACK、不重置任何动作。

令S为扫障开始的预备点，H为正对围栏的锁存航向。
左侧绝对航向L=wrap(H+90°)，右侧R=wrap(H-90°)。按物理左右核对IMU符号。
伤员：原物资放L、障碍放R；普通/核心完全镜像。

伤员完整流程：
1. 保存原清单/目的地、S、H，左转到L。
2. 编码器前进200 mm，停车，Claw_Open完成，保持张爪后退200 mm回S。
   后退完成前不允许转入找障碍；张爪完成不等于物资已退出夹爪。
3. 右转回H，进入现有mode42/43，像素靠近障碍并抓取。
4. 回mode39，按实际靠近轨迹倒退到S并恢复H。
   当前c71efd6的task_safe_sweep_return_to_center会掉头前进回点，不满足“后退回预备点”；
   应保存运动段和航向，反向回放必要的转向/后退，不能仅用累计path_mm当直线距离。
5. 右转到R，前进200 mm，停车张爪完成，保持张爪后退200 mm回S。
6. 转到L找回原物资。此时从R到L是180°，不是只左转90°；
   也可先左转90°回H，再左转90°到L。必须以绝对暂放航向为准。
7. 按下面mode44/45找回；成功后后退实际找回距离到S，转回H。
8. mode23正常合爪复审→mode40→重新定位/视觉ALIGN→走廊检查→无障碍ENTER。

普通/核心的所有左右镜像；前进/后退200 mm顺序完全一致。
不复用旧SAFE_SWEEP_PARK_LOAD/MOVE_TO_LOAD的横移代码冒充此流程。

## 四、新增找回与失败状态，与上位机一致

保留mode39机械动作、mode42障碍靠近、mode43障碍审核。
新增：
- mode44 SAFE_SWEEP_RETRIEVE：原物资视觉寻找/靠近。
- mode45 SAFE_SWEEP_RETRIEVE_AUDIT：原物资140°夹内审核。
- mode46 SAFE_SWEEP_RETRIEVE_FAILED：找回失败，已张爪、后退到S并结束局部动作，等待RETURN_CENTER。

mode44：上位机切回原物资类别/track，发0x09像素APP；没有新目标发HOLD。
必须允许原物资track重建，但APP/HOLD/audit_id变化不得重置下面的200 mm预算。
无目标时HOLD在44中表示没有像素目标，不是取消找回：沿已锁定的原暂放航向低速向前寻找，
有APP时按像素修正靠近，所有前进均计入同一个200 mm预算。
不能进入无限360°普通SEARCH，也不能朝围栏漫游；即使一直没有APP，也必须在200 mm处结束寻找。

mode45：使用普通0x12 CARGO_AUDIT，P5 bit6=0（不是扫障目标审核），INITIAL_STASH=0，
DESTINATION_INJURY保持原物资目的地。上位机统计整个夹爪ROI，不只统计选中track。
F407按原目的地和正式数量规则累计3个不同ID，合法后置AUDIT_VALID，等待上位机GRAB。
不能像普通mode23一样一合法就直接mode22；必须留在45接收GRAB，合爪后回mode39退回S。
非法/空审核不得置AUDIT_VALID，不得拿危险物替代原伤员/物资；失败按下一节退回。
mode45/44切换、140°视角变化应重置审核计数，但不重置位移预算。

## 五、找回累计前进最多200 mm，失败完整退回再返中

进入找回阶段时一次性锁存位移基准，使用编码器累计实际向前运动。
APP更新、track变化、44↔45切换、HOLD、重试都不能重新获得200 mm额度；
短暂后退也不能冲销已消耗的累计前进额度。
例如已前进150 mm，换目标后剩余最多50 mm。到200 mm必须停止继续前进。
在范围内目标进入夹爪，可原地完成现有3帧审核；无目标/未形成有效取回，
按现有有限观察恢复结束本次找回，不能无限原地搜索，也不增加前进行程。

失败动作：张爪，按实际运动退回S，结束局部转向/后退后才上报46。
mode46保持到上位机0x08 RETURN_CENTER到达；接受其H/D并进入现有mode17返中。
46里的重复CLEAR/APP/GRAB不得重启扫障或前进；RETURN重复ACK且不重置返回动作。
上位机收到46清selected_batch、原携带清单、track、审核、扫障和投送上下文，
持续RETURN_CENTER，收到mode3和新帧后重新SEARCH；不发TASK_COMPLETE，不加delivery_count，
不修改first_delivery_done/first_common_delivered。

## 六、其他保持与最小联调

mode43仍使用P5 bit6=0x40专用非空审核，允许蓝色障碍；mode45及mode23不允许该豁免。
模式39/41/44的HOLD不能重置其自主机械阶段/找回预算；44中HOLD按上述限距向前寻找处理。
扫障原任务与临时障碍审核缓存必须隔离；mode41按原规则清全部扫障上下文。
不改变开局藏堆、首件单绿、20°观察、普通mode24恢复、无标定像素走廊、
无障碍直接ENTER、投送视觉最多1秒和原导航坐标。

联调重点：3/4件拒绝正式NAV、2件通过、首件两绿拒绝、开局4件仍可藏；
伤员左右动作与物资镜像；张爪后必须后退；44只追原物资不追蓝色；
找回预算不被新命令重置；找回失败46→17→3；mode41连续HOLD下转向能完整完成。

注意：视觉漏检的物体不会自动出现在审核数量里。本次两件上限能拦住日志中识别出的3件，
不能声称能消除遮挡导致的核心漏计；后续需结合夹内图像检查识别与ROI，不能用截断计数规避。
