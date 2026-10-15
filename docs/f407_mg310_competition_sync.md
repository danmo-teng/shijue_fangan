# F407 MG310 与完整比赛上位机同步

## 2026-10-15：4071c71 增量适配

下位机执行代码基线为`4071c71f0eab3a6a360b8e3d00815bdd6ad25236`；核对时
远端HEAD `93f7550`只比它增加交接文档。上位机从`79d7dc1`增量适配，不覆盖main，
不修改下位机仓库。源码版本不代表实机已经烧录，FIFO精度也仍需实测。

### 轮式完整变换与旧日志差异

1. `0x15`原始16位累计计数按回绕求增量；每轮符号-1只乘一次。
2. 乘`pi * 0.070 / 1061.268`得到轮位移m1/m2/m3，轮序为右前/左前/后。
3. 定位器与建图工具共用`three_wheel_translation`：
   `forward=-(m1-m2)/sqrt(3)`，`left=(m1+m2-2*m3)/3`。
4. 定位器附加旋转`encoder_to_robot_yaw_deg`默认0；建图参考使用上述原生车体轴。
5. 轮式参考平移按T265航向投影到场地；正式四区航向与起点不变。

`79d7dc1`的旧实现为正号forward再旋转180°，最终forward与新公式一致，left相反。
旧文档的“实测180°”对应当时未确认烧录版本的测试，不能证明适用于4071c71。
本次移除这项符号补丁，不修改T265的+x/-y、后20mm/右70mm或位置跳变设置；
历史日志及其中参数不改写，需用已确认的4071c71分别复核前进和左移。
若未来启用轮式yaw，公式为`-(m1+m2+m3)/(3R)`；当前`R=0`继续禁用，
130mm只用于MotionDebug转速协议换算，不能用它自动开启轮式yaw。

### MotionDebug、分离与故障

- MOVE_BODY发送物理前进/左移，左移改为正arg2；逆时针TURN仍取负号。
  轮缘mm/s仅在协议边界转换一次为0.1度/秒。界面移动范围50～700mm/s；
  TURN理想执行时间达到10秒时提示可能超时，不改变下位机时限。
- DONE须同时匹配命令SEQ、opcode、本次发送后时间及1秒内的新鲜状态；
  FAULT、STOPPED或超过1秒无匹配状态取消自动返航后续段，迟到DONE不能恢复。
  失联仅表示机械状态未知，不声称下位机已停。正式H/D和Gamepad不套此调试符号。
  NormalRun与MotionDebug基础极性相同，但转角速度规划不同，不能互相替代精度验收。
- 分离候选与运输合法性分开：mixed_material须有完整计数及绿色/核心ROI证据、
  可靠侧别且符合目标角色才可尝试分离；运输仍只允许现有合法单件。
  这些局部组成只用于分离决策，不改变线上MIXED_MATERIAL类别或审核签名。
- 保留侧独立使用三个不同图像帧，采集间隔至少100ms；侧别或数量改变重新计数。
  等ACK时即使全局签名不变，左右分布改变也重新确认。动作接受后保持已选侧。
- 动作接受才计数；同一堆重新分配任务不清零累计次数。第三轮后仍先审核，
  合法则运输，非法才执行原双开/后退/mode47返中路径。无新增补退或侧移。
- 收到真实F407故障后保留任务现场并锁住发送，故障位清零、失联、人工暂停或
  context_pending都不能自动重发旧动作。人工排查、重新初始化下位机后，按R
  重新配置并等待新状态/上下文。故障码5不能唯一说明FIFO溢出。

夹紧角度仍由下位机决定：能确认保留侧单件合格为20度档，其余为10度档；
同侧同档每轮减5度，换侧/换档不清总次数。上位机不推断实际夹角，不改写数量或类别。
下位机弧线分离按12/16/20度目标、100/130/160mm前段上限、300mm总路径和
60mm横向预算（预留5mm制动）执行，允许提前完成；上位机只等待mode35/mode30与新图像。

### 已核对保留的流程与协议缺口

伤员为injured_orange：23→49（每爪放松5度，150mm/s有效后退2秒）→23→50→22；
mode49撤销旧证据并保留伤员角色，mode50保持第二轮pending，22/40缺pending的17秒
看门狗仍有效。CORE闭爪前50mm推进、扫障独立路径、mode48、先STABLE后READY/VALID、
线上非法签名、占位证据、危险处理、首件、投送超时和计数均不重写。

FIFO算法与细分故障不在0x1A中；未虚构新二进制字段。当前上位机没有程序器文本IMU
解析器，不新增另一路串口控制。外部诊断按字段名查看`ALG=FIFO_TS_TRAPZ`及
IMU_FIFO的PEAK/OVERRUN/FAULTS/MAX_POLL_MS/TEMP_MC；DT_US是硬件样本间隔，
MAX_POLL_MS是主循环读取间隔。CIRCLES/CAL为程序器文本命令，不是0x19指令。
T265场地航向与F407 Location航向分属不同参考原点；0x1A不是原始IMU积分角。
此前提到的动作独立超时、断流重同步、导航净进展判断和NormalRun停车后角速度确认
仍是建议，本次不等待不存在的新回执或状态。

### 部署记录与最少实车验收

上位机重新编译`localization/build`的`t265_omni_localizer`和`t265_map/build`的
`t265_map_builder`，退出旧入口再重启，以生成新runtime配置。两种入口可用
`--actual-lower-firmware <已烧录提交> --actual-lower-mode NormalRun|MotionDebug|Gamepad`
记录操作员确认的信息，默认unconfirmed；正式session和调试metadata记录上位机提交，
这些字段不是UART烧录证明。正式比赛用NormalRun，桌面运动实验用MotionDebug。

1. 前进、左移各一小段，确认实车与轮式参考方向一致；逆时针90度检查负TURN编码及匹配DONE。
2. 单个橙色伤员检查23→49→23→50→22，后退只发生一次，第二轮用新帧，闭爪期间不重审。
3. 分离动作重发不加次数；第三轮合法可运输，非法经mode47返中，中心新mode3/新视觉才再抓取。
4. 从已有故障记录或受控维护流程验证提示和停发；故障位清零仍不续跑，重新初始化并R复位后重新握手。
   不为验收人为制造FIFO溢出或修改运行保护。

仅使用必要编译、语法和现有针对性检查，不新增测试程序，不跑无关整套回归。

### 本次实际修改文件

- 运动解算：`localization/src/fusion.cpp`、`localization/include/fusion.hpp`、
  `localization/include/config.hpp`、`t265_map/src/t265_map_builder.cpp`。
- 当前配置：`localization/config/localization.example.conf`、
  `localization/config/t265_debug_offset_candidate.conf`、
  `localization/config/t265_debug_offset_scale_candidate.conf`。
- 调试控制：`localization/tools/f407_motion_protocol.py`、`localization/tools/t265_f407_motion_map.py`。
- 比赛增量：`competition_rescue/state_machine.py`、`competition_rescue/run_competition_rescue.py`。
- 会话记录：`rescue_map/field_model.py`、`rescue_map/map_app.py`。
- 复用既有检查文件：`localization/tests/test_omni_localization.cpp`、
  `localization/tools/test_f407_motion_protocol.py`、`competition_rescue/tests/test_state_machine.py`、
  `competition_rescue/tests/test_runner.py`、`rescue_map/tests/test_map_app.py`。
- 文档：本文件、`docs/t265_f407_center_debug.md`、`docs/t265_side_mount_baseline.md`、
  `localization/README.md`、`t265_map/README.md`。

## 既有审核机制

上位机定位配置已按 MG310 霍尔版换算：70 mm 轮、13 线四倍频、减速比 20.409，对应

```ini
wheel_diameter_m = 0.070
counts_per_wheel_revolution = 1061.268
```

`localization/build/t265_omni_localizer` 启动时打印 `counts/rev=1061.268`。`TYPE=0x15` 仍传 F407 的原始累计计数；每条里程计算链路仅在原始计数转换为距离时应用一次比例，不能传入已缩放的计数再套比例。正式启动继续使用 T265-only；默认参数、配置模板、当前 runtime 配置和 `t265_map_builder` 的轮式对照换算已同步。历史日志保留当时参数，不能改写。

完整比赛协议已配套 mode49/mode50：mode49 半闭爪后退阶段只发送非 STABLE、frame=0、证据全零的 `CARGO_AUDIT`，保留可确定的角色标志，等待新鲜 mode23 并建立新的 frame floor；第二轮合法审核后在mode50持续发送同一 STABLE 审核，直到当前任务上下文的新鲜 mode22、闭爪和 `AUDIT_READY/AUDIT_VALID` 同时成立，才进入导航。上位机不向 mode49 发送运动或机构命令。

F407 4071c71的PWM上限60%，速度PID为5/0.12/0，直行航向PID为2/0.005/0，
视觉转向PD为0.45/0.0100。电池经保险丝直接给驱动供电；下位机按12V建模，无实时
电压补偿，也未自动应用空载标定表。上位机不叠加这些PID，临时磨合脚本不集成进比赛。

历史84b1777交接时公开main为`d3913f4`，2026-10-10更新至`2550025`；当前配套基线
已更新为上文4071c71的NormalRun，公开源码不能证明实机已烧录。

审核在 mode23、mode48 回到140°且 CLAW_VISIBLE 后观察500ms稳定期，并建立新frame floor。相邻有效采样至少100ms，三帧总跨度至少200ms。普通/核心类别切换、数量、危险/未知/混装变化重计数；左右分布和track变化不单独改变语义。开局藏堆保持任意非空，mode43保持扫障非空。mode50使用已接受的第二轮合法结果等待闭爪完成，不因机械闭爪期间视觉变化重审；mode22或扫障mode40必须匹配当前上下文。

完整空爪画面生成真实空审核；缺图、ROI未标定或不完整、画面近乎全黑/过曝/均匀时，结果为“无法观察”。正式运输mode23/45/48还检查裁切、部分入ROI与明显遮挡。INITIAL_STASH在图像、相机和ROI有效且存在真实ROI内非空检测时，允许多件、混合与重叠堆；mode43只让涉及本次目标的质量问题阻断取障审核，不受无关物资之间的重叠影响。对应占位包的视觉帧号为0，不累计命中。质量检查依据图像和检测框，不能证明所有潜在遮挡均已识别。实际图像使用摄像头采集时间，避免把推理完成时间误当成新证据。

普通审核超过500ms没有新证据时清除旧结果；不可观察画面立即失效。审核 wire mode 的17s期限独立于命中、audit_id、重发和相机摆动；期限到仍未收到恢复状态则记录 `audit_handoff_timeout` 并ABORT。mode24沿原恢复流程等待mode3；mode47沿现有RETURN H/D进入mode17、mode3，不改投送计数。没有增加堵转、反向计数或普通运动无进度的故障锁存。

## 本地三帧与上位机STABLE的先后关系

上位机先独立按采集时间形成三个不同视觉帧、有效采样间隔至少100ms、总跨度至少200ms的稳定证据，并发送STABLE；这一步不以READY为前提。F407还必须收到三个不同audit_id、满足其本地100ms间隔，并收到当前审核的STABLE，才置READY；VALID再检查现有类别、数量和目的地。同帧重发不增加命中或刷新证据时龄。READY未返回时上位机继续处理新帧，过期或不可观察立即撤销旧STABLE。

非法审核按实际序列化字段比较：真实总数、去STABLE后的audit_flags、类别0～6计数直方图，以及数量为0但类别非NONE的类别掩码。左右交换和track变化不影响签名；MIXED_MATERIAL单独计类，不根据未传输的绿色细分计数还原。两位爪侧计数字段维持既有编码，真实total_count不裁剪。开局任意非空和mode43受控非空仍使用各自语义。

## 扫障等待与异常交接

- mode43真实审核及无观察占位保留SWEEP_PICKUP。mode45与扫障后mode23固定原物资目的地，伤员保留DESTINATION_INJURY；不得因眼前类别变化而丢失原角色。
- 无观察包只保留角色和audit_id元数据，类别、侧计数、总数、危险/未知/混装证据为0，frame=0且无STABLE。它不代表实测空爪。
- context_pending期间mode43/45及CLEAR_SAFE_ZONE的审核阶段继续消费视觉；明确拒绝只在状态新鲜、task_id和rejected_action_id都匹配时消费。退回42/44后重建frame floor，恢复APP/HOLD查找。
- 扫障GRAB需要此前实际转发的STABLE包、匹配的已接受审核上下文及READY/VALID。已接受的mode39机械动作只用原接受上下文HOLD心跳等待，不重新请求CLEAR。
- mode50保存第二轮合法pending及原audit_id/frame，不重新审核。mode22/40交接需要对应pending、匹配上下文、闭爪和READY/VALID；成功交接后保留证据至NAV/ALIGN被下位机观察到，避免下一周期误报缺失。
- mode50/22/40缺少合法pending时记录missing_committed_audit，不从selected_batch猜载荷。等待F407既有释放恢复；mode22/40下位机15秒仍未交接会放弃，上位机同wire mode17秒仍无恢复则ABORT。批次丢失不会停用此看门狗。
- 人工PAUSE暂停上位机握手计时，恢复后续计；摄像头恢复暂停仍保留异常握手时限。人工暂停入口为set_paused(..., reason="manual")，相机路径显式标记camera_recovery。通信暂停和现有STOP/ABORT路径保持。

## 同步部署与验证边界

完整比赛请将上位机本分支增量与F407 `255002509144aec510c560a042b29200a70f2ec2` 的NormalRun配套部署，重启定位/串口转发和比赛进程。使用F407已有NormalRun构建配置重新编译、烧录并确认实际提交与模式；T265桌面测试的MotionDebug属于单独运行模式。无需重套旧0007补丁，也没有修改下位机源码或生成补丁。

本次仅做必要针对性检查：独立STABLE/重复帧/失效、49/23/50/22、48及扫障23/40交接、扫障pending/拒绝/42与44恢复、角色占位、非法串口签名、质量筛选、缺失pending及PAUSE/超时恢复；现有协议脚本和Python语法/差异检查通过。未新增测试程序，未运行旧全量回归；尚无此次协议的实车运行日志，图像遮挡启发式和完整比赛仍需配套固件实测。

部署上位机后重新启动启动界面，使其生成本轮 runtime/localization.conf；使用本次重新编译的 localization/build/t265_omni_localizer 和 t265_map/build/t265_map_builder。串口帧类型、同SEQ紧邻配对方式及现有导航速度、路径保持。下位机负责15s审核放弃、半闭爪后退及二次合爪等机械动作，上位机只作审核与交接。
