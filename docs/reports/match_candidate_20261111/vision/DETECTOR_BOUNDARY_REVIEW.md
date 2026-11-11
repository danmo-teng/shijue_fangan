# 部署后漏框边界复核（只读，未运行BPU）

本项只读当前部署代码、两轮原日志和已有事件视频。未加载模型、启动BPU/相机/SDK/电机，未改算法、阈值或原日志。两轮实报模型SHA均`2a7b3f7406dbf59a39ea9a227e7bd32e597fa435e7d1783f560d8a08c1703065`，全局score与绿色score均0.20。NMS代码默认0.45，启动器未显式覆盖，但本场日志没有单独记录其值，不能把默认称实报。

## 当前检测不是跨帧学习

`vision/rescue_vision/yolo26.py`每次传单张640×640 NV12到同一HB_HBMRuntime，解码六个类别/LTRB输出头；代码没有跨帧特征、长时间观察累积分数或“待久了才激活”机制。YOLO26 worker与多帧跟踪分开。看久后才有框仍可能来自姿态/曝光/清晰度/遮挡变化、网络分数跨过阈值、部署预处理或后处理，现有日志不能单指训练数据不足或模型内部要等待。

流水线：原1280×1024图→VSE 640×512并居中letterbox成640×640（或原CPU等效路径）→NN→每anchor取六类最大logit与单一argmax类别→score≥0.20→有效正宽高LTRB框→**逐类别**NMS→逆letterbox/裁剪/取整及零面积剔除→runner绿色额外0.20过滤→tracking→ROI/任务选择。

绿色单独阈值只作用green_supply；黑/橙没有类似额外score删除。两轮全局与绿色恰好同0.20，绿色附加过滤在本场通常不再删除框；黑/橙不会被它误删。NMS按类别分开，因此safe_blue大框不能直接跨类NMS掉injured_orange。

## 日志“detections”不是NN原始输出

`competition_detections.jsonl`和VisionReplay的`detections`来自runner中make_detection后的列表：已过全局score、argmax、NMS、逆变换与绿色过滤，尚未过tracking/ROI。它不包含低于阈值框、未选类别分数、NMS删除项或NN原始tensor。`tracks`则只记CARGO_CLASSES的跟踪幸存者，含旧框与visible=false；安全区类即使在detections内，也不会出现在cargo tracks中。10Hz检测日志、8Hz事件视频均抽样，不代表其间每帧未检出。

队列均有界：worker可跳过尚未推理的旧输入，parent可只取多个已完成结果中的最新一个；这是保最新的既有策略，不是所有摄像头帧都逐一记录。记录中某一frame_sequence确无detections能定位到该次部署后处理输出为空，不能反推整个连续时段都没有正样本。

## tracking会隐藏旧框，不能倒删本帧已检出的新框

当前实际与候选`rescue_vision.json`相同（SHA `c33ed74fb9e2a0050c62f1fcb688db45b1ac88c3a6c78a9197ca1eac0b36613d`），黑min_hits5/max_misses4/match_distance176px，橙3/5/227px，绿3/5/186px。tracking匹配类别和位置，confidence做0.35当前＋0.65旧值EMA；这个EMA没有后续最低分门。

重要区别：runner的visible是misses==0且非LOST，**新TENTATIVE的hits1也可visible=true**，不是必须等黑5帧/橙3帧才画框或进入夹内计数。普通远目标选择可能要求hits≥2，合爪审核仍要求原两实帧间隔/跨度；这些是业务证据，不是网络等待。

当前帧检测正常会新建或匹配Track并misses=0；上一帧曾检出、当前帧未检出时misses递增，visible立即false，超过max_misses再移除，因而旧框不能被当新观察。GUI draw_overlay仍遍历含旧框的cargo，显示有框不保证当前detector有框；灰色本身仍是前景区域状态，而非模型confidence判定。本轮不改显示。

## 两组逐阶段证据

| 数据 | 部署后detections | tracking/任务端 | 能确定的边界 |
|---|---|---|---|
| 第一轮video0008，10427/capture467.005 | 黑0.2568，bbox284,366,158,93 | ID892/hits3/misses0/visible=true | 低hit并未被全局5帧门挡掉 |
| 同轮10431/10441/10446 | 无检测 | 旧892分别misses2/1/3，visible=false | 是当前最终检测缺失，tracking只是撤旧框可见性 |
| 同轮10450/capture467.745，原检测日志同sequence | 空列表 | 旧893/conf0.2043/hits1/misses1/visible=false | 画面肉眼有黑，最终部署检测无框；不能怪ROI先删了检测 |
| 第二轮检测日志7385/mono966.366 | 橙0.2039，bbox567,598,533,287 | 新514/hits1/misses0/visible=true | 贴阈值的单帧确进入tracking；不是min_hits3丢掉它 |
| 同轮7389→7392 | 只有safe_blue | 514先misses4/visible=false，随后被删 | 没有持续新橙观察，不能拿旧0.2039称当前仍识别 |
| 同轮video0004，7400/capture966.745 | 只有safe_blue | cargo tracks空 | 真图横置橙仍在前沿，最终部署链漏框；safe_blue不跨类NMS压橙 |
| 同轮7422→7435（原后退改变视角后） | 橙0.3319→0.7309等 | 新515/hits3→15/visible=true | 改视角后恢复了新的track，不能说旧491仍被可靠跟踪 |

7385的514即使visible=true，也不是原持货491。现delivery阶段按已审载荷身份观察，不会把任意新同类地物直接当已送成功；所以“已出现一个橙框但任务没认载荷”也可能是身份不连续，而非NN没框。这一严格归属用于防混计，不能只放松hits/全局阈值就声称解决。相关橙横置残留由forensics/P1_DELIVERY_RESIDUAL_REVIEW.md独立说明。

没有NN原始输出，不能彻底排除score/argmax、同类NMS、回归退化、量化或预处理异常。正常逐类NMS至少保留同类高分框，黑/橙近处框也并非本来就在原图零面积边缘；但没有pre-NMS候选/张量，不能把“代码没发现明显跨类误删”提升为“全部部署后处理已证无误”。正确表述是**部署后检测输出漏框**，训练、导出、量化、预处理和阈值边界的责任尚未分开。

## 最小可行动候选（未实施）

1. 先复用当前VisionReplay事件链，针对已触发load_empty_reobserve/投送未见载荷的一小段保存少量**原生未叠字输入**；当前AVI是960×768缩放、叠框、MJPG有损，不能当NN精确原输入。若要严格查0.20边缘，再只保一帧实际640×640 NV12输入张量和对应frame_id、阶段候选数/分数，而非保存整场大tensor；必须沿原有有界异步路径，先审成本，不加新测试系统/计时/控制门。
2. 对这些相同输入在原YOLO26实现中离线比较score前、NMS后、逆变换后各阶段。若主要是低分，先验证黑压黑爪/极侧边、橙横置近距遮挡/围栏前沿的定向数据及难负例，再训练/量化对照；不凭少数0.2039样本就全局降到0.1。
3. 现有单帧模型仍不稳时，候选比较“原输入”与“同一140°夹内ROI放大重识别”。收益是提高细小/暗黑目标占比，代价是再跑一次640网络、上下文和坐标映射变化、假阳性及尾延迟。先离线同帧验证；本轮不加入比赛第二条推理、旧框补观察或新等待。

原两次138/142复看已确认执行并最终恢复，不再额外增加复看次数。开局新增start_clear录像只补中心观察证据，并不能替代这一步原始模型输入取证或证明模型准确性。
