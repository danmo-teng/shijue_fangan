# 正式比赛复用桌面动态定位链

2026-11-05，基于上位机e783f71增量迁移。用户明确选择：保留正式1～4号起点和初始航向，不等待围栏初始化才出发；后续定位使用桌面测试同一套实现。源码接入及必要检查已完成，尚未实车跑完整场。

## 实际调用关系

- 原C++进程仍唯一持有底盘UART和T265 SDK：保持官方wheel_odometer输入、Pose投影、原始日志及0x18/0x1D任务配对。
- `competition_wheel_input.py`把同一批F405真实07/08反馈同时送入原WheelInputBridge和动态定位；没有新增底盘串口或额外握手。
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

重开原桌面“智能救援完整比赛流程”，默认`--dynamic-localization --official-wheel-input`，底盘/dev/ttyS1、雷达/dev/ttyS6，均230400。原起点/阵营选择保持；无需新增桌面入口。`--no-dynamic-localization`保留既有纯T265＋官方轮速对照入口，需要新SDK会话，不支持热切换。

地图显示当前T265/轮速来源与雷达可观轴，保留测试的“越障暂停/恢复雷达”按钮及L键。自动识别减速带遮挡尚未实现；没有为了迁移擅自新增自动屏蔽规则。比赛期间不可操作电脑的限制意味着该人工按钮不能被当作自动越障能力。

日志继续位于`rescue_map/runtime/history/competition_wheel_input/<会话>/`：

- 原`t265_raw_pose.csv`、`localization_debug.csv`、`wheel_input.jsonl`、`wheel_sdk.jsonl`。
- `pose_motion.jsonl`、`dynamic_scans.jsonl`、`dynamic_localization.jsonl`、`dynamic_events.jsonl`、雷达收发及工作进程日志。
- `control_pose.jsonl`保存送给C++的动态快照，`dynamic_status.json`/`dynamic_summary.json`记录来源、可观轴、位置重获及处理错误。
- 正式JSON/0x16中继写盘失败不终止UART；动态路径的原始T265诊断JSON失败也不退出控制循环。会话结束时此类原始JSON错误返回非零，避免标成完整会话。没有宣称所有旧日志写盘路径都已重构。

完整会话保留最近5次，保护当前/不完整/用户文件。回放文件仅按同一SDK会话的完成清单识别；未知文件保留。本轮没有清理原始日志。

## 安全区无框回放（不改业务动作）

用户本轮明确不加入H±8°扫描或人工重试，保留原安全区识别流程。既有5秒无框等待到期只多记录一次`safe_zone_acquire_no_box`事件，包含位姿、相机角、识别框状态和任务上下文。

复用YOLO真正处理过的相机图像，独立`vision_replay.py`进程保存超时前约5秒、后约15秒；每次等待只触发一段。预滚动在内存压缩缓冲，未触发不录整场；没有再开相机或SDK。

- `safe_zone_replay_0001.avi`：MJPG，画面960×768，另加96像素状态栏，8FPS上限，检测框/ROI及英文状态叠加。
- 同名`.jsonl`：每视频帧的原始采集时间、相机/视觉帧号、检测结果、上下位机状态、命令、位姿；位姿是当时最新快照，不伪称与相机严格同采样，也不是地面真值。
- `vision_replay_manifest.json`：会话归属、片段清单、实际帧数、结束原因、丢帧/错误。AVI为固定播放FPS；实际间隔以JSONL为准，不复制旧帧填补缺口。
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

已只读核对main `f4be806`：STAGE累计行程限停已删除、藏点末端已改全向平移，当前上位机H/D接口兼容，实际烧录未知。正常定位迁移不需要新协议。

mode51仍有T265专属恢复门，会拒绝真实轮速降级位姿。用户已确认最小配套，见`F405_DYNAMIC_POSE_HANDOFF_20261105.md`；未在本窗口修改/烧录，也未代发。

安全区无框无限等待按用户决定暂不改动作，仅增强回放。故“完整比赛已实车通过”仍未成立；下位机配套/实际身份和下一轮记录需要闭环。
