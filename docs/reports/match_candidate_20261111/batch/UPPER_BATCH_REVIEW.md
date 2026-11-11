# 首件后2/3件材料：上位隔离候选与两端配套

本次新增用户要求：首件仍按原单绿；后续2或3件绿色/黑核心（含混合）直接合法合爪搬运；伤员仍单件、危险蓝禁止。多件应先推→抬爪后退→放爪再推→抬爪后退。以下仅候选；未部署正式、未改对端/烧录/启动设备。

## 上位最小实现

RDK_MATERIAL_BATCH_2_3_CANDIDATE.patch最初由/tmp副本生成，现已由主代理应用并完善到隔离候选源码，未部署正式。最终补丁相对77d49dc生成，包含以下新检查及两处超量样本修正；原生成阶段指纹与记录保留在upper_batch_candidate_checks.json。

- Settings max_batch_count默认3，只允许1..3；首件是否合法仍由原first_common_delivered阶段决定，首件必须且只单绿。first_green_visually_confirmed只控制主动单橙优先，不拿它替换上下位机既有合法性相位。
- _audit_valid允许首件后1..3件、manifest全部属于green_supply/core_black。现有混侧编码mixed_material=6/count0..3已能表达，不增加协议字段。混侧必须有至少2件及0<green_count<count的本地构成；unknown/danger/伤员混合、侧计数和总数不一致仍非法。伤员仍只能单件。
- runner原audit_from_cargo已完整保留full ROI class_counts；绿黑混侧不生成side_invalid/unknown，不需要修改该生产者或截断成1件。当前搜索聚堆的CLUSTER_APPROACH只是靠近，实际DISPERSE由非法审核触发；合法2/3审核将沿原GRAB，无需新建分组选取系统或替换已锁目标。
- 审核后的_latch_carried_manifest保留全部类别与数量并更新selected.confirmed_total_count；不会把搜索时的一个锚点ID当最终物件数。delivery_count沿原语义是投送批次数，不能拿它称送入物件总数。
- 多件视觉确认不能沿用bool(any inside)。仅carried_total_count>1时，已知可见ID数量、完整manifest匹配，且全部inside，才发布delivery_target_inside_safe_zone；缺框、陌生替代ID、错类或仍有区外件不证明整批成功。单件保原行为，未知仍沿原1秒超时收尾，不增加等待、握手、停车或新时间门。

## 必要检查及真实记录

6项针对检查通过（upper_batch_candidate_checks.json）：三种2/3件材料沿真实方法离线跑21审核→GRAB→23/22完整载荷交接→48运输审核→ALIGN；危险/伤员/超量及首件限制；原单橙优先与危险蓝两项既有检查；线上mixed/count字节；多件必须整批inside的正负例。以上mode/回执由离线输入提供，不是实车或UART验收。

真实raw_audit复算：第一轮6087左右各一绿一黑与11427同侧绿黑2件，原政策非法、候选合法；第二轮2017处尚未完成首件，混合2件仍非法。未把命令或审核接受当物理成功。

原state主入口与逐函数基线/候选对照另见state_full_baseline_candidate_checks.json：基线32函数有13个既存失败，候选34函数保留同13个失败，另两处旧“2绿非法”的测试预期需按新规则调整。旧13项涉及默认camera_pitch=0、历史FAULT/边界API、未设置实际目的地等，不能声称全套通过，也不为本轮修改无关流程。

需要更新的两项旧非法样本：test_mode51_reference_audit_exits_and_budget和test_range_cumulative_and_transport_sort。建议把用于非法恢复/分离分支的2绿改为左右2+2总4绿，保持原分支验证；原两绿现在应走合法运输，不再要求释放/分离。上述两项已按4绿样本更新并通过；没有删除这些检查。

## 部分找回按真实数量继续

partial_recovery_candidate_checks.json离线调用现有mode23→40交接：原2→实际1、原3→实际1、原3→实际2均继续ALIGN；carried_total_count、manifest长度与selected.confirmed_total_count均更新成实际1或2。不为凑原数量新增等待/门；其余留场件未声称找回。下位双推应按最后接受的真实audit_last_total_count锁存，不能用历史sweep_original_total覆盖新审核。

## 下位配套边界

上位不能单边部署：actual自报e85c738/dirty1对应仓库参考源码仍total==1并拒mixed_material。下位需要对同一既有审核合同允许首件后1..3纯材料，初次单绿/单伤员/禁危险保持，并按真实已审总量锁存多件双推。

双推必须在下位ENTER/EXIT内部完成；重复ENTER当前只ACK，不会重做动作。第一推后若mode15已经开放上位可能按现1秒确认/超时发TASK_COMPLETE，因此下位应保证第二推完成前不把该命令解释为本趟已退出；可保持既有动作身份并在原内部阶段完成，无需新增反复握手。具体机械位移/抬爪角/顺序由下位候选说明，不能由上位文档假定完成。任何部分找回以新实际审核数量决定单/双推。

本报告不承诺绿黑不再漏出、两次推送必然成功、首件超时旧相位风险已解决，或正式比赛已启用批量；这些仍须人类审查与原入口实车验证。

## 最终入口交接补核

三组成分均扩展原21入口和真实CLUSTER38→37确认交接；38虽ACK但未转37的负例仍保持AUDIT_CONFIRM，不提前GRAB。随后23/22完整manifest与48 ALIGN均通过。最初扩展夹具误把38当确认态，已按实际Task.c修夹具，无需改状态机或添加握手。最终针对20个原函数通过与语法5文件结果见../FINAL_VALIDATION.json；全套旧13项失败未冒称通过。
