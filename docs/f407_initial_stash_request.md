# 可直接转交F407的开局藏堆联调要求

请基于F407最新版核对以下流程，只修改必要的流程冲突，不增加停车超时、FAULT或ABORT。
本次核对版本为16c4b77；其中task_audit_semantic的开局非空优先规则已经正确，请保留。

1. 开局临时藏物资与正式送物分开处理。收到CARGO_AUDIT的INITIAL_STASH标志后，
   只要total_count>0，按STASH_NONEMPTY审核；不限制最多3件、不限制类别，
   不因危险物、未知、伤员混装、左右计数不等于总数而判非法。
   左右两位计数允许饱和到3，不能据此否定整堆；仍要求3个不同audit_id。
   连续非空期间类别、数量、左右位置变化不应打断STASH_NONEMPTY累计。
2. 上述规则必须同时覆盖合爪前和mode23合爪后复审。审核合法后允许GRAB，
   mode23复审合法后进入mode22；保持audit_initial_stash，随后NAV进入route_to_stash。
   开局非空不能误走FIRST_GREEN_BUMP、DISPERSE、单侧释放或正式首件绿色策略。
3. 藏点NAV到达后接受RELEASE_BOTH，实际释放完保持mode34供上位机确认，
   再按现有RETURN_CENTER流程退让、返中、进入mode3。
   藏点双开不能因为audit_valid=true而拒绝；16c4b77已允许audit_initial_stash例外，请保留。
   藏物资完成不能置first_delivery_done；返中后的第一件正式投送仍必须单绿色。
4. 空爪或靠近失败可按现有恢复回mode3。上位机在藏堆未完成时会清旧批次，
   回INITIAL_OBSERVE等待新帧重试，不会把一次恢复视为藏堆完成。
   不要把已确认非空的开局审核按正式非法组合释放后直接进入正式搜索。
5. 上位机已修复INVALID_RELEASE/INVALID_BACKOFF收到mode24/3仍等待旧完成mode的冲突；
   不需要通过放宽SEARCH中旧RELEASE/DISPERSE命令的接受条件解决。
6. 上位机在mode15视觉观察最多1秒，之后持续TASK_COMPLETE，未确认会记录超时而非视觉成功。
   如果要求实际退出也不多等200 ms，将APP_DELIVERY_VERIFY_WAIT_MS从1200U改为1000U。

保持正式投送的危险物优先释放、20度观察、mode24恢复、STAGE/ALIGN/ENTER、
0x13及mode39/23/40、mode41协议不变。

上位机仍缺现场homography.txt和homography.txt.meta.json，这不是F407对齐故障。
需在1280×1024下使用vision/calibrate_ground.py实测生成两文件，确认距离不再为null；
不得用虚构矩阵掩盖缺标定。F407不需要为此更改ALIGN或ENTER协议。
