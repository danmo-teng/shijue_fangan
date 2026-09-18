# 上位机配套cd5a072：限推审核、三次分离与恢复分流

已只读核对F407 cd5a072；本次实际修改上位机任务状态机、上下文发布/恢复门和C++帧解析。
不增加UART命令或字段。0x1D/0x18、0x1E/0x17和READY/VALID原协议保持。

## 300 mm限推审核

mode21/38/37停车不作为上位机卡死条件，继续发送完整夹爪ROI的新视觉帧和audit_id。
READY=0继续审核；READY=1且合法才GRAB，非法才安排释放/分离。
上位机不发送重新开始APP来恢复推进行程，同一批次的审核和物理分离保持task_id。
下位机保留同目标300 mm预算、停车审核及既有4秒决策窗口；不是限制整条远距离APP。

## 三次递进分离

- 默认上限3；20°无侧观察单独计数，不计为带侧分离。
- 只有本次DISPERSE带侧命令或单侧RELEASE被接受，才累计一次；重发/等待不重复累计。
- 原物理分离请求保留同一action_id。下一次真实分离请求有独立请求轮次，分配新action_id。
- 一旦确定保留侧，后续重试保持同侧；该侧已无可保留物时执行普通放弃，
  不借换侧重新拿三次额度。松角仍完全由F407控制：0°、10°、20°相对第一次基准。
- mode35完成后建立新frame floor，完整夹爪ROI重新三帧审核。
  单侧RELEASE→YIELD→复审同样可以接下一次带侧分离，不再固定一次复审就双开。
- 第三次READY=0时不下失败结论；第三次合法GRAB，READY=1仍非法发送最终双开，
  不发送第四次带侧请求。F407如已经自主mode24/47，上位机直接按真实恢复状态处理。
- 首件单GREEN、后续普通/核心合计最多2件、伤员单件、开局任意非空藏堆保持。

## 恢复路径分开处理

| 下位机证据 | 上位机处理 |
|---|---|
| 普通放弃进入mode24 | 同已接受上下文HOLD，等待本地张爪/退300/左转90；不提前RETURN |
| mode24→mode3 | 清旧任务，建立新帧下限，按原搜索策略继续；藏堆未完成则INITIAL_OBSERVE |
| 第三次失败mode24→mode47 | 47才表示已开爪且退完300 mm，清旧目标/审核/携带物，启动T265 H/D RETURN |
| mode47→mode17→mode3 | 同task的新RETURN action；更新H/D不换action，到mode3建立新frame floor |
| 开局藏点RELEASE_BOTH | 原mode34→RETURN，不改为普通失败恢复 |
| 聚集360°恢复 | 原停车/HOLD握手，不追加300 mm失败返中 |
| 扫障找回失败mode46 | 原扫障失败返中，保持独立于mode47 |
| mode41 | 原边界恢复，全上下文清理和HOLD |

mode47返中不发TASK_COMPLETE、不增加delivery_count、不改first_common_delivered或initial_stash_done，
不设置safe_zone_exit_pending。首件未完成时返中后仍只允许正式单GREEN。
上位机不假设mode24只能持续500 ms，可以等待下位机数秒的机械恢复。

## 已请求与已接受上下文分开

上位机分别保存requested task/action和F407配对状态中的accepted task/action。
新鲜mode24、已执行任务后的mode3、mode41、46、47先于context_pending和相机暂停门处理，
取消过时的APP/GRAB/分离pending请求，不能为等待被拒绝请求而阻止恢复。

HOLD/PAUSE发往F407已接受上下文，不使用未被接受的请求编号。
mode47启动RETURN沿用已接受task_id，分配新的action_id；本地分配游标不回退复用已申请编号。
重复mode47等待RETURN ACK时只重发同一RETURN，不每周期重新分配动作号。
RETURN的vision_frame固定0，仍使用完整配对帧。

保留旧mode3竞争修复：新CLUSTER请求刚发出、尚未看到执行时的旧mode3，不被误判为恢复。
RETURN的mode17/3仍核对对应上下文及接受证据，ACK不能代替到中心。
F407无需伪造接受已拒绝的请求，回报其真实accepted上下文即可。

## ALIGN与S1退区

上位机没有独立1.5°ALIGN前置门，继续以当前ALIGN归属的新鲜mode11交接；二次视觉修正保留。
下位机3°进入、6°保持容差不改普通/聚集像素对正阈值。

mode16期间可以缓存RETURN，上位机不会在S1升55°等待1秒、后退300 mm、降85°等待1秒时
判定RETURN卡死；运动进展观察仅在实际mode17返中时启用。
上位机不发S1/S3舵机指令，S3仍120°，不把相机角度当作S1升降反馈。

## C++接收器

丢弃噪声字节、异常或重复帧头时清除待配对0x1E上下文。
只有紧邻同SEQ的1E→17才成立，原CRC错误/非状态插帧清理保持。
此次已重新编译localization转发程序；换设备部署时也必须更新二进制。

## 下位机仍需保持的配合

cd5a072已有对应机械路径，本次不另要求LCD、调试输出或测试修改。
保持：300 mm同目标限推、三次同侧松角、普通失败24→3、第三次失败24→47、
mode47等待RETURN、S1完整退区、真实accepted上下文回报，不能回退为所有失败都返中。
650 mm推进、末段330 mm/s/1500 ms接触保护、无障碍ENTER、进攻/防守坐标、
1秒投送超时继续且记录delivery_visual_timeout均不变。
