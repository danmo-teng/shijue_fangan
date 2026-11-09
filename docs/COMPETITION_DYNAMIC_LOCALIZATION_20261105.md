# 正式比赛复用桌面动态定位链

## 当前正式测试准备（2026-11-09）

用户决定直接烧录NormalRun后跑正式比赛，独立MotionDebug暂缓，不作为前置；C++导航迁移列低优先级。当前b70538a已将高频传播/来源/历史/位姿发布迁C++，正式入口与测试共用native实现，Python比赛决策仍按原逻辑；见LOCALIZATION_NATIVE_CONTROL_20261109.md。下文带日期内容保留历史边界，不能据旧板速或旧清理策略覆盖当前入口。

原桌面“智能救援完整比赛流程”：底盘/dev/ttyS1、921600；雷达/dev/ttyS6、230400；官方轮速输入＋雷达/T265/轮速主yaw降级链，YOLO26与所选正式起点不变。原待机预热后按3绑定当前Pose/gyro到所选起点并放行，按4仍走原人工停止。此轮不自动发车。

下位机GitHub main已只读核对至f1cf2a5909f408b28d821b29b0fc6a63d91fb8dc：最新为烧录回执文档，固件实现仍14b2a3a；CMake两个既有preset共用921600。NormalRun应为board405/mode5/service2（RESCUE_TASK），不能只看mode5，因为MotionDebug也为mode5、service1。该信息是源码/构建依据，实际NormalRun尚待用户烧录后读取，不加SHA白名单或时间资格门。无需本轮下位机新功能。

此前正式软件就绪检查已通过（之后本轮另修纯T265外参入口，见下节；无墙打滑限制仍在）：

- 12个现有相关检查全部通过，`12 passed in 2.70s`：热启动起点绑定、任务/审核与测距交接、mode51、预备点/回中D=0、停止优先和原状态年龄采用政策；仅Mock/临时目录，无真实比赛CONFIG/STOP。
- 原桌面与登录自动入口均指向当前`run_competition_map.sh`；shell语法、正式地图/runner原入口`--help`、cv2/numpy/PIL/hbm_runtime/Gst导入和雷达配置读取通过。
- 定位ELF、`libmotion_runtime.so`、`liblidar_geometry.so`、JPU/VSE库、YOLO26模型/标签/ROI/配置齐全，动态链接无缺失；10项候选源码/ELF与115931实测SHA完全一致。RealSense实际链接为`/home/sunrise/文档/ChatGPT/T265/build-250/librealsense/librealsense2.so.2.50.0`。
- 当前ttyS1/ttyS6/video0/jpu/bpu可访问，没有正式/测试/SDK/视觉进程争用；GPIO控制器存在，T265 USB枚举为8087:0b37。这不代表当前SDK已输出Pose或按键已实按。
- 现阶段仅完成软件就绪。用户烧录NormalRun后，重开原桌面入口进行正常待机预热，按原遥测记录actual身份与07/08/SDK结果，再在实际起点按3跑正式流程。预热不启动比赛任务；不把历史MotionDebug身份代替NormalRun。现有自动入口已安装，本轮未启动监听/预热、未打开设备或发送运动，原始日志未修改/清理。没有整场实车通过结论。

本轮12项必要检查的可重复命令（不访问真实设备）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  competition_rescue/tests/test_runner.py::test_formal_dynamic_start_wiring \
  competition_rescue/tests/test_runner.py::test_warm_start_binds_latest_pose_gyro_and_keeps_sdk_owner \
  competition_rescue/tests/test_runner.py::test_warm_start_stop_wins_before_release_and_failure_does_not_retry \
  competition_rescue/tests/test_runner.py::test_warm_config_partial_start_remembers_abort_obligation \
  competition_rescue/tests/test_state_machine.py::test_start_handshake_suppresses_task_frames_until_start_clear \
  competition_rescue/tests/test_runner.py::test_published_post_grab_audit_survives_changed_frame_before_paired_done \
  competition_rescue/tests/test_runner.py::test_range_move_camera_handoff_never_rearms_pixel_approach \
  competition_rescue/tests/test_runner.py::test_received_status_age_does_not_pause_navigation_but_claw_audit_keeps_check \
  competition_rescue/tests/test_runner.py::test_mode51_context_priority_and_separation_rejection \
  competition_rescue/tests/test_state_machine.py::test_mode51_reference_audit_exits_and_budget \
  competition_rescue/tests/test_state_machine.py::test_safe_staging_arrival_keeps_zero_handoff_and_stash_window \
  competition_rescue/tests/test_state_machine.py::test_return_center_sends_zero_until_search
```

## 纯T265入口外参修复及安全区空转边界（2026-11-09）

原`CompetitionMapApp`只有启用官方轮速时才选择当前安装模板；同时关闭动态链和官方轮速，会误用`localization.example.conf`的旧杆臂/轴向。现将模板选择与注入开关解耦：正式`t265`模式统一使用`t265_debug_new_mount_candidate.conf`，默认动态/官方链不变，显式旧`fusion`模式不在本轮修改范围。

当前六项值保持：forward=-0.0346105m，left=-0.0599471445m，forward_C=(0.8660254,0,0.5)，up_C=(0,-1,0)，scale_enabled=false，scale=1.000。正式写配置仍覆盖所选1～4号起点与135°/45°/225°/315°，不会移植测试中心起点。候选文件仅更正“TEST ONLY/正式模板未变”的过时注释。

完全关闭SDK轮速与外部轮式降级的已有纯T265入口是以下双开关组合（仅说明，本轮未启动）：

```bash
./competition_rescue/run_competition_map.sh --no-dynamic-localization --no-official-wheel-input
```

只关闭dynamic仍保留官方注入；只关闭official而保留dynamic，会按原设计启用官方注入，均不能称纯T265。模式切换须结束旧SDK会话再启动；新SDK会话也不能冒称设备内部状态已彻底清空。纯模式不创建WheelInput/dynamic组件、SDK轮速配置或注入/反馈socket；原始编码器诊断不等于位置输入。

必要检查在原`test_runner.py`内：`test_formal_t265_mount_is_independent_of_wheel_input`、`test_formal_wheel_start_and_pose_adoption`、`test_formal_dynamic_start_wiring`全部通过，约2.2秒，覆盖纯模式四起点/六参数/pose jumping/无注入组件以及官方/动态原接线。仅Mock/临时目录，不访问UART/SDK/相机，不发电机命令、不清原始日志，没有实车精度结论。修改文件为`competition_map_app.py`、原检查文件及配置注释/本轮文档；无协议或下位机改动，commit/push见Git。

**当前未解决的空转边界：**动态链完全看不到双轴墙时，T265/轮式位移冲突仍可触发`WHEEL_FALLBACK`，不能区分好T265与打滑轮式；恢复亦需要双轴墙及运动一致性。已有墙证据判滑不是无墙安全区预案，不能说原T265备用已自动兜底。

**供人决定的方案，尚未实现：**对安全区有意顶推/退出的风险阶段，暂停SDK新轮速调用，同时排除上位机轮式尾段、轮式冲突/恢复比较；轮计数/gyro及日志继续接收，位置用T265增量＋主yaw，雷达只修可观测方向。退出策略时以当前输出重新建立T265偏置和当前累计轮式锚点，保持输出连续，不补发空转段或伪造零速，不清原始编码器。暂停SDK不立即清除既往轮速影响；若T265也异常且无墙，不能凭软件恢复真实XY。

ENTER从接近即开始，不是接触证据；现有线上反馈没有实际接触或final_push阶段。不能一收到ENTER就冻结XY，也不能把mode17当已确认驶离斜坡。上位机粗阶段屏蔽无需新固件但覆盖较长；若要精确按既有最终顶推/退出子阶段切换，可讨论复用下位机已有状态/保留字段，仍不证明物理接触，不新增传感器判断、中断、握手或年龄门。两种阶段范围和无可用T265时的处理须先说明再决定，本轮未发布F405修改任务。

真正已知贴墙且固定朝向时可讨论墙法向约束；不能冻结仍可能横移/转动的整个XY/yaw。下位机最终顶推和退出自身仍以编码器累计行程限程，轮滑也会影响动作结束；改变上位机注入不能顺带解决这部分，若需修改另作最小提案。


2026-11-06更新：9b6a661已完成基于e783f71的迁移，本轮继续修复只读审查发现的四项问题。保留正式1～4号起点和初始航向，不等待围栏初始化才出发；后续定位使用桌面测试同一套实现。源码接入及必要检查已完成，尚未实车跑完整场。

## 本轮异常路径修复

- `lidar_yaw_debug/worker.py`把串口打开、启动命令及读取错误隔离到雷达连接。失败后清理未处理扫描和当前墙约束，保留同一个DynamicFence/MotionSource、场地参考、已积累平移修正，继续消费T265/实测轮速/主yaw。每秒最多一次尝试重开雷达端口；这是端口重试节奏，不检查反馈年龄、不阻止运动传播。重连重建雷达解析段，保持操作者原先暂停/启用修正的选择，不重新初始化车位。
- `competition_dynamic_localization.py`与地图显示/记录实际`lidar_connected`和`lidar_error`；雷达断开不会被标为当前仍有墙约束。若定位工作进程因其他未处理的算法/进程异常退出，本轮未新增另一套定位器接管，不能宣称任何进程故障都已恢复。
- `debug_wheel_input.cpp/.hpp`去掉单次SDK false/异常后的会话锁存；该样本仍记录失败且只尝试一次，后续新真实样本继续调用。`frame_num`仍为序号，不重放失败速度。日志区分历史API失败计数、末次调用结果和历史依赖；未知设备状态不伪称成功，不增加USB复位/第二SDK进程。这不保证物理断线的SDK设备自动恢复。
- 正式`competition_wheel_input.py`只从实际身份回包派生NormalRun/TASK或MotionDebug/MOTIONDEBUG，保存原始数值；正式地图显示SHA、dirty、协议和波特率。没有回包仍unconfirmed，未知组合明确unknown。启动metadata保持当时的未确认状态，实际回包见`chassis_identity.json`，最终身份见`final_status.json`；没有新增身份查询、等待或拒发。
- 回放由Planner真实`safe_zone_acquire_no_box`事件触发，事件独立于抽帧时当前状态，待录事件上限32条。帧队列满不消费待录事件；重叠事件合并到片段并延长尾录，事件原时刻/上下文保存于清单。没有图像或录像进程失败时不编造画面，结束回执明确未交付/溢出，录像异常不阻断命令。

本轮必要检查：构建正式定位器与既有C++测试，覆盖SDK false→新样本成功、异常→新样本成功和失败样本不重放；既有动态传输检查用PTY覆盖端口启动缺失、中途断开后轮速＋主yaw传播和自动重连，保持参考及原暂停选择；正常扫描路径通过。既有回放检查实际MJPG编码回读覆盖状态切换后触发、重叠事件延长尾窗和队列满后交付；正式启动接线及反馈适配检查通过。仅软件/模拟接口，无物理串口、相机、SDK或电机动作；未新增独立测试程序、未清理日志。

## 实际调用关系

- 原C++进程仍唯一持有底盘UART和T265 SDK：保持官方wheel_odometer输入、Pose投影、原始日志及0x18/0x1D任务配对。
- `competition_wheel_input.py`仅观察同一批真实07/08并转发动态定位；2026-11-08起SDK输入配对/换算由原C++ UART/SDK所有者内的F405WheelMeasurements完成，Python不再产生第二路SDK输入。观察通道用有界SEQPACKET保留C++接收时间，没有新增底盘串口或F405握手。
- `competition_dynamic_localization.py`只负责传输。它复用`RadarWorker`独立工作进程，原`DynamicFence`负责去畸变/围栏约束/位置重获，原`MotionSource`负责T265与轮速来源切换。
- 测试入口与正式入口共用`lidar_yaw_debug/inputs.py`，该函数从测试入口原样抽出，保持gyro方向、MCU区间计算与实际时间表达。
- 动态结果送回C++ `DynamicPoseRelay`，从同一快照写正式`localization_result.json`和0x16。独立线程不等待新的SDK Pose帧；原T265独立0x16发送关闭，原T265 JSON另存`raw_t265_result.json`。
- 正式比赛状态机消费动态JSON，原命令/动作配对保持。不会把原T265位置和动态位置按权重再次混合。

## 保持与唯一坐标适配

T265外参、比例1.000/关闭、pose jumping、SDK速度源T/W=0、0.1m/s工程初始噪声均不改。雷达倒装外参、145°定位视场、147°展示视场、坏角屏蔽和算法阈值使用原`lidar_yaw_debug.json`。

正常：T265锚点＋实际轮反馈覆盖尾段；空间冲突/跟踪丢失：轮速＋主yaw传播、雷达只修可观测轴；恢复：运动一致性与双轴墙支持后平移重对齐。两次一致的双墙位置重获、单墙不补另一轴、支持范围外保持端点等原逻辑不改。未修改`motion_source.py`、提线/墙面拟合算法。

正式原始Pose已按所选角落投影到场地坐标，因此动态初始位置转换保持恒等；用第一个有效Pose的实际位置作为运动锚点，避免把启动中已经发生的位移再置零。所选起点单独保存在`reference.selected_start`，主yaw第一个实际样本与所选初始航向建立固定偏置。无红蓝对称重选，无测试地图中心起点移植。

0x16仍15字节：VALID代表整个定位结果可用，WHEEL_ACTIVE表达实际轮反馈传播；WHEEL_FALLBACK不伪置T265_GOOD。原始tracker/mapper confidence保持原义，sigma高位使用饱和值，不能当实测协方差。JSON标记`qualified_pose=false`、`accuracy_verified=false`，可运行不等于精度验收。

## 使用及日志

重开原桌面“智能救援完整比赛流程”，默认`--dynamic-localization --official-wheel-input`，底盘/dev/ttyS1为921600，雷达/dev/ttyS6为230400。原起点/阵营选择保持；无需新增桌面入口。`--no-dynamic-localization`保留既有纯T265＋官方轮速对照入口，需要新SDK会话，不支持热切换。

地图显示当前T265/轮速来源与雷达可观轴，保留测试的“越障暂停/恢复雷达”按钮及L键。自动识别减速带遮挡尚未实现；没有为了迁移擅自新增自动屏蔽规则。比赛期间不可操作电脑的限制意味着该人工按钮不能被当作自动越障能力。

日志继续位于`rescue_map/runtime/history/competition_wheel_input/<会话>/`：

- 原`t265_raw_pose.csv`、`localization_debug.csv`、`wheel_input.jsonl`、`wheel_sdk.jsonl`。
- `pose_motion.jsonl`、`dynamic_scans.jsonl`、`dynamic_localization.jsonl`、`dynamic_events.jsonl`、雷达收发及工作进程日志。
- `control_pose.jsonl`保存送给C++的动态快照，`dynamic_status.json`/`dynamic_summary.json`记录来源、可观轴、位置重获及处理错误。
- 正式JSON/0x16中继写盘失败不终止UART；动态路径的原始T265诊断JSON失败也不退出控制循环。会话结束时此类原始JSON错误返回非零，避免标成完整会话。没有宣称所有旧日志写盘路径都已重构。

当前按容量告警和完整会话手动清理，不再自动仅保留5次；保护当前/不完整/用户文件。回放文件仅按同一SDK会话的完成清单识别；未知文件保留。本轮没有清理原始日志。

## 安全区无框回放（不改业务动作）

用户本轮明确不加入H±8°扫描或人工重试，保留原安全区识别流程。既有5秒无框等待到期只多记录一次`safe_zone_acquire_no_box`事件，包含位姿、相机角、识别框状态和任务上下文。

复用YOLO真正处理过的相机图像，独立`vision_replay.py`进程保存超时前约5秒、后约15秒；同一次等待只产生一个超时事件，重叠事件合并并延长尾录。预滚动在内存压缩缓冲，未触发不录整场；没有再开相机或SDK。

- `safe_zone_replay_0001.avi`：MJPG，画面960×768，另加96像素状态栏，8FPS上限，检测框/ROI及英文状态叠加。
- 同名`.jsonl`：每视频帧的原始采集时间、相机/视觉帧号、检测结果、上下位机状态、命令、位姿；位姿是当时最新快照，不伪称与相机严格同采样，也不是地面真值。
- `vision_replay_manifest.json`：会话归属、片段清单、真实触发事件列表/时间、实际帧数、结束原因、丢帧/错误及未交付事件。AVI为固定播放FPS；实际间隔以JSONL为准，不复制旧帧填补缺口。
- 队列最多4帧；满则丢录像帧并计数，失败只影响回放完整性，不改变命令、状态或定位。退出可提前结束片段，记录`session_end`。不能从旧日志补回未采集的画面。

## 必要验证与限制

已经通过：

1. C++编译、既有`test_omni_localization`：动态0x16/JSON相同位置与采样时刻、会话配对、无年龄门、降级VALID及原始T265标志、已有协议与外参检查。
2. 既有Python测试中的动态运动来源、去畸变、单墙/双墙与重获，以及F405真实格式转换检查。
3. 四个起点的首Pose偏移6cm＋轮反馈回归，确认不丢位移、不误降级；PTY雷达工作进程与Unix IPC传输、关闭、无新Pose时仍输出快照（不是实际硬件）。
4. 正式启动命令/配置、唯一UART、原始与控制输出分离、mode51原任务配对、临时目录日志保留检查。
5. 合成图像真实MJPG编码/回读、视频帧与JSONL索引/时间匹配、预滚动/后续窗口、队列满和写盘失败非阻断；只在临时目录运行。

未启动物理UART/SDK/相机/电机；下一次实机仍需核对联合负载、画质、场地精度、越障缺墙及完整投送。不能把上述软件检查说成实车跑通。

## F405和完整比赛遗留

已只读核对`51f1d070`：STAGE累计行程限停删除、藏点末端全向平移保留，当前上位机H/D接口兼容。正式构建NormalRun=F405/mode5/service2；运动调试MotionDebug=service1；协议1.3，UART4为230400/8N1。实际烧录未知，正常定位迁移不需要新协议。

mode51已在`828c65ca`落实整体VALID兼容，取消原批准范围的T265/附加时间门，保留重参考、合爪审核和旧动作取消；`51f1d070`保留此修改。原任务单`F405_DYNAMIC_POSE_HANDOFF_20261105.md`仅供核对，不重复要求修改；未在本窗口修改/烧录，也未代发。

安全区无框无限等待按用户决定暂不改动作，仅增强回放。故“完整比赛已实车通过”仍未成立；实际固件身份、mode51恢复和下一轮记录仍需核实。本轮history目录已为空，现存runtime会话记录为e783f71，不能冒充新版动态链/回放的实车结果；本轮未执行日志清理。
