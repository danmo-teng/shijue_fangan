# 可直接转交F407：停用自动地图边界恢复＋预备点140°复审

已核对下位机b618701。上位机本次实际增加TRANSPORT_AUDIT状态并使用新mode48；
下位机仍由下位机负责人修改，本文件不是F407补丁。
不要求LCD、调试输出或新增测试；只修改以下业务逻辑。

## 一、关闭基于地图坐标的自动mode41触发

在Task_Process中停用task_check_boundary_guard()自动调用。
可以用默认关闭的编译开关包住该调用，不是把50 mm再次改成别的触发距离。
不要删除mode41枚举或重排后续mode编号；上位机保留收到mode41后的兼容处理。

只关闭这条自动地图坐标触发路径，不改变：
- 夹内慢爬300 mm预算。
- 扫障原物资找回200 mm累计预算及mode46。
- ENTER总650 mm、末段330 mm/s和1500 ms接触保护。
- 各动作自身编码器行程、停止条件、通信/视觉帧龄和上下文保护。
- 原场地坐标系、T265变换、导航目标、进攻/防守策略。

## 二、新增mode48，明确区分于mode23

新增常量：TASK_TRANSPORT_AUDIT / wire mode48。
它表示正常物资已到600 mm预备点，停车保持夹爪闭合，正在进行运输后的完整夹内复审。
不得直接复用mode23的“合法后转mode22”路径，否则会重启NAV形成循环。

原STAGE到达交接保持：新鲜mode10＋DISTANCE_DONE＋GRIPPER_CLOSED。
上位机不再立即发ALIGN，而是以同task的新action发送现有CARGO_AUDIT：
首包total=0、非STABLE、vision_frame=0、INITIAL_STASH=0、SWEEP_PICKUP=0。
此首包是进入复审请求，不是确认空爪。

F407仅在TASK_NAVIGATE、delivery_stage_only、distance_done和gripper_closed成立时，
允许CARGO_AUDIT触发mode48：停车，保持夹爪闭合，相机转140°，清旧READY/VALID和审核缓存。
清除CLAW_VISIBLE，实际140°稳定后才重新置CLAW_VISIBLE。
重复同action审核只ACK/更新审核，不重复转相机或重置整个阶段。

相机未就绪时上位机持续非STABLE全零占位包，不发HOLD阻止相机完成。
上位机看到mode48＋140°＋CLAW_VISIBLE＋GRIPPER_CLOSED后建立新frame floor，
只计之后的真实新帧。视角离开/回到140°重新累计，不跨角度拼帧。

## 三、mode48审核语义

- 继续使用0x12和现有审核字段，不新增命令或报文类型。
- 纳入task_audit_state及上下文校验，只有匹配task/action的真实新vision_frame可累计。
- 收齐3个不同实际视觉结果才READY，合法才VALID；SEQ、ACK、STABLE均不能代替。
- mode48不执行夹内前进慢爬，也不把合法审核自动改成GRAB/NAV/ENTER。
- READY=0继续原地审核；不能设置超时自动放行ALIGN/ENTER。
- 普通/核心正式合计最多2件，伤员只能单独1件，首件仍只能单绿色。
- 伤员+普通/核心、危险、未知、超过数量限制，均非法。

上位机本阶段读取整个夹爪ROI，不按原track或携带清单过滤，不把数量截断为2。
仅在mode48，将普通物资计入ROI的重叠要求从80%降到60%，与伤员/核心一致，
用于减少伤员旁边普通物资漏计；不扩大到全屏，不放宽其他阶段审核或投送成功规则。
这不能替代检测器对遮挡物的识别，完全漏检的物体不会凭空进入计数。

## 四、合法后的两个出口

1. 实际目的地未变：
   上位机按新清单锁存类别/数量，直接发送定位ALIGN。
   F407增加从mode48接受合法定位ALIGN的分支，恢复相机120°，
   待相机稳定、定位对正完成再上报mode11；继续原二次视觉ALIGN→新框走廊→CLEAR/ENTER。
   不额外合爪，不回mode22，不重新走同一个NAV。

2. 实际目的地改变，例如运输伤员后发现只剩普通物资：
   上位机发同task、新action的STAGE NAV实时H/D，目标改为实际物资对应半区预备点。
   F407在mode48＋READY＋VALID＋夹爪闭合时接受该NAV，沿用正常STAGE流程。
   不能沿用旧伤员区目的地；抵达新预备点后再次复审运输结果。

注意mode48允许根据真实类别改变审核DESTINATION_INJURY，不能套用扫障找回阶段
固定原目的地的审核拦截，否则会拒绝这次重新分类。

## 五、非法与空爪出口

- READY=1、VALID=0时，允许按现有RELEASE_LEFT/RIGHT/BOTH处理。
- 伤员与普通分处左右时，上位机按任务选需要释放的一侧，完成后接原YIELD和新帧复审。
- 混在同一侧、未知或无法有效保留时，按普通放弃路径双开恢复。
- 特别处理mode48的稳定空爪：允许READY后RELEASE_BOTH执行普通放弃，
  不要被现有task_release_command_valid中total_count==0的拒绝条件卡住。
  进入mode48时的vision_frame=0占位包不能触发这个空爪放弃。
- 不调用投送完成逻辑，不加delivery_count，不改首件或藏堆完成标志。
- 不随便新建task重置分离/限推额度；三次分离、mode24普通恢复、mode47返中规则保留。

上位机mode48期间接收到真实mode24/3/41/47会取消旧pending审核并按对应路径恢复。
READY/VALID和CARGO_AUDIT完成回执必须属于当前action，不能沿用运输前mode23的结果。

## 六、保持1秒投送策略

预备点复审与安全区内mode15投送观察是两个阶段。
mode48没有超时放行；mode15仍最多观察1秒，未确认仍TASK_COMPLETE，
记录delivery_visual_timeout。不得把这两个规则混为一谈。

上位机需要配套此mode48固件后运行。旧b618701不会接受STAGE停车后的审核请求，
新版上位机会等待该请求被接受，不再绕过复审直接ALIGN。
