# 三全向轮轮滑检测与 T265 轮速融合 RFC

**2026-11-10 · WHEEL-SLIP-RFC-20261110 · 仅供上位机负责人评审、选择，不授权实现**

RDK X5 评审基线为 `danmo-teng/shijue_fangan` 的 `codex/gamepad-teleop@2046c0104311a5c5b958c72c0f8a8bd3b5df7a19`。下位机 `kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot@41f23a605bbd36c2a48fae2eb85022da5a4c44af` 只用于核对既有遥测与固定入口，源码提交与实烧身份分开。本轮交付仅包括 RFC 和上位机固定入口登记。下文的数据结构、算法、验证和动作均为待选设计。 [RDK 源码基线][rdk] · [F405 固定入口][f405-entry]

## 1. 请负责人分别决定的事项

1. **采用哪种证据。** 角差可先复用已有 07/08，原始 IMU 可扩大到瞬态平移，已接受的双轴墙位移可补持续平移。三个通道各自给出可观范围和不一致原因。结果分为一致、不一致和证据不足。
2. **证据可以改变什么。** 可保持现状、只离线/旁路计算、只控制新一帧轮速是否送入 SDK，或另立上位机传播仲裁。暂停 SDK 注入与排除上位机轮式尾段是两项选择。角残差只涉及编码器推导的转动。该转动量本身未送 SDK，单凭角差停整组三维速度是需要明确批准的保守策略。
3. **何时恢复。** 旋转和 XY 分开恢复，证据必须覆盖最近一次冲突后的新运动。当前完整 XY 墙观测可能迟到、降秩或依赖轮式先验。评审需决定能否接受墙证据缺失时的停送持续时间，以及允许哪些经过验证的替代证据。
4. **双异常如何输出。** 轮速可疑且 T265 跳变时，现有代码可退回轮式。是否保留并显式标注这个退路，或改用可观轴传播、分量级不确定度，单独评审对控制输出的影响。

最低风险的取证顺序是先选 A（主 gyro 角差离线对照）或 B（T265 原始 IMU 旁路）之一，继续用现有策略做 A/B 基线。只有在留出正常样本、已标注异常和停/续注入对照都有结果后，才有依据选择仅 SDK 门控或另做传播仲裁。也可以保留现状。选项、成本及审查条件见第 9 节。

## 2. 当前代码和证据

| 正常链路 | 基线实况 | 对设计的约束与源码 |
| --- | --- | --- |
| T265 同一 SDK 进程 | 配置 Pose 6DOF，鱼眼为可选。`debug_test_session` 启用独立 Pose 回调队列，回调只接 Pose 和鱼眼 frameset，其他入口用常规 pipeline 消费。`raw_pose` CSV 的 angular velocity 来自 `rs2_pose`，当前没有独立 GYRO/ACCEL 样本。 | 增加运动流时需保 Pose 原有频率和鱼眼采集路径，避免让低频流拖慢 Pose。`raw_pose` 只能作 SDK 输出对照。 [配置与 resolve][rdk-stream] · [回调][rdk-callback] · [Pose CSV][rdk-pose] |
| 真实轮速与主 gyro | `F405WheelMeasurements::decode()` 读 07 的三轮 `raw1/2/3_u32`、`delta1/2/3`、测量 ID、MCU `t0_us/t1_us` 和体心前/左速度，08 含主 yaw 和 rate 各自源 ID 与接收事件时刻。按 07 有符号计数复算 XY 并与回传核对。 | 直接从 07 取纯编码器运动。 `prepare()` 发给 SDK worker 的 `omega_body_rad_s` 是同一 MCU 区间主 gyro 积分的平均值，参与相机杆臂换算。当前 schema 2 测量包未透传 07 的三轮增量或纯编码器角量。 [F405 遥测][f405-telem] · [RDK 07/08 解码][rdk-wheel-decode] · [区间、计算与包][rdk-wheel-prepare] |
| SDK 注入 | `DebugWheelInput::configure()` 在启动前用同一已 resolve 的 `pose_sensor` 装轮速 JSON。native worker 按反馈唤醒，处理 07/08，`latest()` 只取一条准备完成的最新测量，允许合并。`process()` 先校验和消费序号/区间，暂停时略过 SDK 写入。 | 检测窗口应在 `latest()` 合并之前消费每个有效、唯一的 07 区间。已有发送路径继续只写一条真实当前速度。`next_frame_` 是 SDK 序号，已由 `process()` 消费的暂停样本不能重放。仍等主 gyro 端点配齐的 07 可留在 `pending_`，见第 6 节的停送边界。 [worker][rdk-wheel-worker] · [latest][rdk-wheel-latest] · [SDK 配置][rdk-wheel-config] · [停送与序号][rdk-wheel-send] |
| 上层传播 | 原生 `motion_runtime` 从同一 UART/SDK 的反馈副本与 Pose 消费，`NativeMotionSource` 用 T265 锚点和轮式尾段，主 yaw 负责场地航向。`step` 或位移差 >0.05 m 可进 `WHEEL_FALLBACK`，SDK 失跟踪也可触发。安全区排轮分支 `T265_PHASE_NO_WHEEL` 单独存在。 | 普通位移冲突不会自动让轮可疑。轮可疑后 SDK 跳变/失跟踪仍可走轮式；已有首因保留与输入暂停，无法据此宣称双异常已解决。 [阶段处理][rdk-phase] · [跳变和传播][rdk-native-pose] · [降级与恢复][rdk-fallback] · [现有离线双异常][repo-priority] |
| 墙面证据 | `MotionRuntime::apply_wall()` 用已接受扫描的参考时刻及候选车心。`observe_walls()` 需要完整 XY 和 raw SDK、轮历史在同一参考时刻可取，活动量（墙/SDK 位移、轮累计行程三者最大值）≥0.25 m 才比较。用 ≤0.025 m 一致和 >0.05 m 冲突形成四种 verdict，两次同 verdict 才触发对应仲裁。重获先建基线，活动 ≥0.50 m 且有确认或歧义时可换基线。 | 单墙当前仍可修可观轴，但不能投完整 XY 的来源票。墙解受主 yaw、选用的去畸变轨迹及几何求解先验影响。原有恢复还需 ≥0.50 m 轮/SDK 对照行程、至少两次完整 XY 修正，轮可疑时另需两次 `BOTH_AGREE`。这些是当前实现门限和窗口，未经本车新残差验证。 [修正入口][rdk-wall-apply] · [墙残差与门限][rdk-wall] · [恢复][rdk-fallback] |
| 暂停策略 IPC | 既有 `wheel_input_paused()` 为阶段屏蔽 OR `wheel_suspect`。原生 `publish_policy()` 和 Python 回退 `publish_sdk_policy()` 只以 `(pause, reason)` 去重。即使带 `wall_motion_evidence`，SDK worker 的 `source_policy()` 对同状态同原因提前返回，也没有证据消费者。 | 如果由 SDK 进程以墙证据恢复本地暂停，需发布、接收两处都分开处理状态和证据。保留墙裁决在原生传播所有者内也是一个选择，见第 7 节。 [暂停定义][rdk-pause-flag] · [原生发布][rdk-policy-pub] · [Python 回退][rdk-policy-py] · [接收][rdk-policy-rx] |

**已有离线和静止证据。** 115931 记录的实际 SDK 为 librealsense 2.50.0、T265 固件 0.2.0.951。004815 静止暂停约 2 s，暂停区间 399 个 Pose、约 199.56 Hz，200 条暂停样本未注入，恢复后 840 次 API 成功。这个结果只覆盖当时的静止通路。97 个 MotionDebug 动作及两场正式日志用于复算轮/主 gyro 和来源对照，缺少原始 T265 IMU、独立地面位姿和标注滑移起止。历史约 28 ms 是主 gyro 对 SDK/轮的重复相位差，无法分离物理采样、内部滤波和时钟映射。把 0.115 m 当作有效轮臂的离线候选可降低角差，运行模型仍按实际配置，不用残差最小值替代机械与地面标定。 [实际版本][repo-control] · [静止停续][repo-pause] · [角差与反例][repo-gyro] · [时序相位][repo-sync] · [当前待办][repo-todo]

## 3. 官方 SDK 与论文依据

### 3.1 本机接口能支持的融合动作

| 项目 | 官方材料的证据 | 对本车的约束 |
| --- | --- | --- |
| 版本、帧和单位 | 以部署记录对应的 v2.50.0 为准，交叉核对 v2.53.1。两版 `doc/t265.md`、wheel 示例、C++ 包装及 `tm-device.cpp` 内容相同，C 头文件对应 wheel 签名相同。公开文档演示 Pose、GYRO、ACCEL 同管线。Rev004 数据表和白皮书为 gyro 200 Hz、accel 62.5 Hz、Pose 200 Hz，早期 Rev002 的惯性速率表相反。实际 SDK profile 枚举 accel 为整数 62，gyro 为 200，物理采样率以帧戳和唯一帧号复核。 | 采 `motion_frame::get_motion_data()` 的三轴 rad/s 与 m/s²（比力），区分 `rs2_pose` 的融合输出。以选中的序列号、profile 与外参为准，不强行请求 62.5 这个非整数 profile。v2.53.1 仅用来对照，TM2 后续已从上游移除。 [v2.50.0 运动流 API][sdk-motion] · [TM2 profile][sdk-profile] · [Rev004 数据表][sdk-datasheet] · [Rev002 数据表][sdk-datasheet-old] · [官方白皮书][sdk-white] · [v2.53.1][sdk-253] · [TM2 移除记录][sdk-remove] |
| 轮速调用 | `send_wheel_odometry(sensor_id, frame_num, translational_velocity)` 接收标定表索引、帧号和 m/s 三维线速度。v2.50.0 TM2 发送 `DEV_SAMPLE`，两个时间字段置零，注释说明固件接收时填写，`dwFrameId` 与时间字段分开。SDK 调用成功可记录主机路径，固件使用了多少权重、是否接受样本没有融合回执。 | 发送/停送是最小改法。已经送入的样本无法撤回。按当前顺序发送当前真实测量。没有逐样本的外部时间戳、速度置信度、协方差或角速度形参。等残差窗口凑满后再发旧速度会把旧运动当作当前。 [C API][sdk-api] · [C++ 包装][sdk-cpp] · [TM2 实际发送][sdk-tm-send] |
| 标定与权重 | 官方示例先取同设备 `pose_sensor.as_wheel_odometer()`，在 `pipe.start` 前装 JSON。每 velocimeter 配 `scale_and_alignment`、`noise_variance`、外参 T/W，T 为米，W 为轴角弧度。源码配置走 `SLAM_APPEND_CALIBRATION`，无固件成功回执。示例为人工速度配小方差以看出位姿变化，也记录全遮挡可能只剩姿态。 | 保留现有单源 id 0、恒等 scale、T/W 为零的 C 系速度约定及原噪声。静态方差是会话配置，不用于运行中逐样本改写。停送、真实零速、人工零速、缩速、重复旧速各有不同物理含义。只有收到真实零速且策略放行时，才照原义送零。停送与恢复瞬态、长时间停送的定位表现须在批准后验证。 [标定文档][sdk-calib] · [官方 wheel 示例][sdk-example] · [TM2 装载][sdk-tm-config] · [当前外参约定][repo-context] |
| 坐标、时间、定位质量 | T265 Pose 的跟踪原点 C 是两鱼眼之间的中心。设备轴 X 向右、Y 向上、Z 向后，SDK 世界 Y 朝上。原始运动传感器各有 profile 外参，TM2 将运动 profile 注册到 Pose profile。官方 `GLOBAL_TIME` 为设备到主机相关时间，非 F405 时钟。Pose jumping 可修正平移跳变，置信度为 SDK 估计。 | 用实际 profile 取 motion→Pose 外参并验证方向，再组合车体安装。SDK 的 Pose/速度与原始 IMU 共享传感器，注入时还与编码器共享输入。停送不重置估计器，也不恢复独立性。置信度高时仍要处理跳变与共享输入。 [官方坐标与时间][sdk-coord] · [TM2 外参注册][sdk-extrinsics] · [Pose jumping][sdk-jump] |

### 3.2 研究方法与本车可借用的部分

| 一手资料 | 原方法、假设和输出 | 可借用部分与迁移代价 |
| --- | --- | --- |
| Yin 等，Ground-Fusion（ICRA 2024） | 论文 `III-B 的惯性与轮里程预积分位移差阈值为 0.015 m，并以插值去偏的 IMU 角速度改善轮里程航向。作者源码 `wdetect` 的判据为 `(dP_wheel-dP_imu).norm()>0.02`，位移累计依赖 `Rs/Vs/Bas/g` 与作者轴映射。`wheelanomaly` 是当前优化标志，主优化跳过轮因子，对应边缘化也有条件分支。论文数值、实现数值及作用范围须分开。 | C 是同一运动区间的位移一致性基线。B 借剔除原则改用速度增量，省去独立初速假设。作者 IMU 替换轮 yaw 的做法不能用来构造独立角残差，源码坐标和 15/20 mm 都不能直接移植。优化器能选择窗内因子，SDK 门控只能影响后续调用。 [论文][gf-paper] · [检测][gf-check] · [两路累积][gf-prop] · [因子跳过][gf-factor] · [边缘化][gf-margin] |
| Geneva 等，OpenVINS（ICRA 2020） | `UpdaterMSCKF` 先投影视觉特征 Jacobian，再算创新 `S=H P Hᵀ+R`，用 Cholesky 求 `rᵀS⁻¹r`，和对应维数的 95% 卡方分位乘系数比较。其门控对象是视觉特征创新。 | 可用相同统计量给角/速度增量/投影墙残差标准化，模型和协方差要从本车定义。阈值用于异常检验，数值不是打滑概率。外置检测无需引入整个 MSCKF。 [论文][ov-paper] · [官方推导][ov-doc] · [源码][ov-src] |
| Potokar、McGann、Kaess，ROSE（RA-L 2024） | 在 SE(3) 上预积分，轮角速模型加入左右轮滑移量，在线估计两轮半径和轮距。用一阶 Jacobian 修正预积分，并给零滑移先验套 Tukey 核，让强反证下先验饱和。它用于成对轮、平均侧向/竖向速度为零等越野假设，需视觉等观测拉开滑移和参数。 | F 采用三轮三分量模型和独立外部观测，标定参数慢变、滑移快变。原 GTSAM 因子接两个轮速及二维 slip，不能接三全向轮或约束 `v_y=0`。属于外置估计研究，暂不把修正速度送入 SDK。 [论文 `IV–V][rose-paper] · [双轮预积分][rose-src] · [slip/内参修正][rose-corr] |
| Zhang 等，Ground-Fusion++（IROS 2025） | M3DGR 有轮滑、视野退化及 LiDAR 退化场景。其系统优先使用 3D LIO，特征数/配准误差退化时切换至成功初始化的增强 VIO；VIO 继承 Ground-Fusion 传感器退化策略，两个子系统有 SE(3) 对齐与平滑。发布代码保留 `wdetect` 的 0.02 比较。 | 借用分来源健康判断和重锚，写入 E 的退化选源。当前 2D 围栏定位仅支持所观轴，无法直接套 3D LIO、Cauchy 对齐或把两路都坏的情况变成“平滑可用”。 [论文 `III–IV][gf2-paper] · [VIO 检测源码][gf2-check] · [作者仓库][gf2-repo] |
| Quan 等，单目＋轮＋MEMS 陀螺；Lee 等，VIWO 在线标定 | 前者在视觉/轮联合优化后超过一半原匹配成为离群点时，重新匹配并仅用视觉求解当前帧。后者用差速模型进行轮预积分、内外参和时差估计，并分析充分激励下的可观性及退化动作。 | T265 已融合 Pose 没有独立的视觉特征内点或 VIO 创新接口可接。可在 D 做不吃编码器的鱼眼/外部视觉相对位移，在 F 用多方向激励校几何与时序。D 的参照应能区分视觉退化与轮滑，F 的可观性须按三全向轮重算。 [Quan 原文][quan] · [Lee 原文][lee] |
| Kilic 等，本体滑移检测；Censi，ICP 协方差 | Kilic 用 INS 与轮速度比较，利用独立识别的静止事件做零速/零角速更新，还依赖非完整运动伪测量。Censi 分析配准欠约束时的可观子空间与测量相关性。 | B 可借独立静止重置偏置，D 用墙法向投影和秩评价。横移是本车正常自由度，无法套非完整伪测量；仅凭 IMU 平静也不能断定静止。 [Kilic 原文][kilic] · [Censi 原文][censi] |
| Reginald 等，学习补偿；Huang 等，模糊 IESKF | 前者用有地面真值的轮模型误差训练 GPR，在差速 VIWO 中补偿滑移。后者用轮/惯性与视觉的运动和质量特征分别调整 IESKF 过程噪声和观测噪声。 | 有标注、跨地面和负载复测后，可把两者纳入离线对照；若应用连续权重，只能写进自己掌控的测量协方差或因子。现有两场日志不能支持迁移训练、概率标定或在线补偿。 [GPR 原文][gpr] · [模糊 IESKF 原文][fuzzy] |

## 4. 统一输入、坐标和时间区间

### 4.1 三轮观测与可观测性

车体系 B 为前、左、上，逆时针为正。车心到第 $i$ 轮的矢量为 $\mathbf r_i$，驱动方向为平面单位向量 $\mathbf d_i$，等效旋转力臂为 $L_i=\mathbf d_i^T(\hat{\mathbf z}\times\mathbf r_i)$。取 07 有符号区间计数 $\Delta n_i$ 和测量时长 $\Delta t$。用当前代码的轮径 $D=0.070\,\mathrm m$、$N=1061.268\,\mathrm{count/rev}$ 作源模型复核，$k=\pi D/N\approx2.07216\times10^{-4}\,\mathrm{m/count}$，$u_i=k\Delta n_i/\Delta t$。任意布局的行模型为 $A_i=[d_{ix},d_{iy},L_i]$。理想 120° 等臂模型为

$$
\mathbf u=A\boldsymbol\xi_e,\quad
A=\begin{bmatrix}\sqrt3/2&1/2&L\\-\sqrt3/2&1/2&L\\0&-1&L\end{bmatrix},\quad
\boldsymbol\xi_e=
\begin{bmatrix}
(u_1-u_2)/\sqrt3\\
(u_1+u_2-2u_3)/3\\
(u_1+u_2+u_3)/(3L)
\end{bmatrix}.
$$

A/B 的数据提取复用 07 解码，保存 `(boot_epoch, reference_epoch, measurement_id, t0_us, t1_us, delta[3], vx, vy)` 与原 flags。只在 RDK 内部派生 $\Delta\psi_e=k\sum_i\Delta n_i/(3L)$，或附加明确来源的 `omega_encoder_rad_s`。同一帧的 MCU XY 与主机复算用于检查符号/协议，作为同源量。可先用 0.130 m 控制模型做复算，0.115 m 仅作为离线敏感性对照。三轮计数和每轮力臂以后若有独立测量，则换成实际 $A$，先检查秩与条件数。 [07 源码][f405-telem] · [角差分析与 L 候选][repo-gyro]

设真实速度满足 $\mathbf u=A\boldsymbol\xi+\mathbf s$，$\mathbf s$ 表示每轮驱动方向的等效滑移速度。三轮三个未知车体自由度，在 $A$ 可逆时无轮内冗余。IMU yaw 只加一个约束，看到的是近似 $\sum s_i$。$\sum s_i=0$ 的滑移可有任意明显 XY 误差，同量的三轮共同误差则可能只有编码器角差。即使只有一只轮坏，从一个角残差也有三个候选轮身份。删掉猜中的一轮、平均减去角差或将横向速度设零，都需要额外物理证据。匀速直行/横移例中，编码器为 0.50 m/s、车体真实 0.20 m/s，双方角速度和速度增量可同时接近零；有墙法向位移或独立视觉才看得见此误差。 [数学反例及计算][repo-gyro]

### 4.2 外参、比力和参考点

- 记 $R_{XY}$ 为把 Y 系向量变到 X 系的主动变换，G 是窗口内固定、Z 向上的局部惯性系，C 是 SDK Pose 跟踪中心，I 是原始加速度测量点。加速度流的 `get_extrinsics_to(pose_profile)` 给 $R_{CI},t_{CI}$，即 $\mathbf p^C=R_{CI}\mathbf p^I+\mathbf t_{CI}$。按 SDK 矩阵存储顺序读出，验证正交、行列式和方向。gyro 也分别取其到 Pose 的 profile 外参。原始 motion API 表示 SDK 提供的运动样本，是否需额外偏置估计由数据决定，避免把厂校矩阵重复乘上。
- 当前 $R_{CB}$ 的三列为车头、左和上在 C 系的方向。已用配置 $f_C=(0.8660254,0,0.5)$、$u_C=(0,-1,0)$、$l_C=u_C\times f_C$，相机中心杆臂 $\mathbf r_{BC}^B=(-0.0346105,-0.0599471445,0)\,\mathrm m$。由此 $R_{BI}=R_{CB}^T R_{CI}$、$\mathbf r_{BI}^B=\mathbf r_{BC}^B+R_{CB}^T\mathbf t_{CI}$。保持当前上下倒装和偏航安装约定。IMU 参考点只用于残差，原注入仍用 C 点和主 gyro，原 Pose 转回车心仍走原函数。
- 在独立静止片段用重力方向初始化倾角，或明示姿态来源是 SDK 辅助且考虑相关性。单个静止姿态无法独立解全轴加计 bias 与倾斜，应结合已有厂校和多姿态对照。在可信窗口内用去偏原始 gyro 的中值角增量推进 $R_{GB}$，加计的重力补偿采用 $\mathbf g^G=(0,0,-g)$。静止须验证补偿后水平分量接近零，标记震动、饱和、显著倾斜或坡面造成的平面假设失效。$1^\circ$ 倾角误差在 100 ms 内可产生约 0.017 m/s 的水平速度增量误差。
- 车心到 I 点速度需补 $\boldsymbol\omega^B\times\mathbf r_{BI}^B$。用窗口端点的参考点速度差，可涵盖切向、向心及转坐标系项，无须对有噪声的 gyro 再数值微分。若改在加速度层从 I 还原车心，则需同时处理 $\dot{\boldsymbol\omega}\times\mathbf r$ 和 $\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf r)$。校验至少覆盖静止六轴方向、纯前/左、正负原地转和边走边转，确保残差计算没二次应用 C 点杆臂。 [安装约定][repo-context] · [SDK 坐标和外参][sdk-coord] · [profile 注册][sdk-extrinsics]

### 4.3 时钟域和正常路径

| 数据 | 记录、配对和约束 |
| --- | --- |
| 07 / 08 | 07 为 MCU 测量区间，08 的 yaw 和 rate 为各自 MCU 接收事件。以 boot、reference、各自 source ID/measurement ID 去重，积分使用实测区间和截断端点。轮/主 gyro 在同一 MCU 事件域配对，不能用相同 UART SEQ 代替同一物理采样时刻。 |
| T265 motion | 保留每流 `{kind, stream_index, frame_number, SDK timestamp_ms, timestamp_domain, receive_monotonic_ns, xyz}`。`GLOBAL_TIME` 与主机系统钟关联。对齐时用同一主机时间锚映到 monotonic（与现有 Pose 映射同类），保留映射方式和误差。系统钟跳变或 profile/reconnect 时切断分析窗口并单独记录来源。避免把接收抖动当真实车体动态。 |
| MCU ↔ 主机 | 若已有时钟交换可用，复用现有区间估计，保留 `capture_time_qualified=false` 和未知边界含义。无共同区间时该残差记为证据不足，现有注入和 0x16 仍按正常链处理。原日志可做离线相位扫描和按动作留出验证，将主 gyro 滤波相位与 clock offset 分开报告，约 28 ms 不直接写成全局常数。 |
| 墙面 | 只按扫描 `reference_ns` 与基线时刻比较运动，用同一定位参考和测量支撑区间。几何计算完成时刻、数据报到达时刻与扫描时刻分别记录。连续两帧 ID 不保证两个非重叠证据窗。 |

同一已拥有 T265 的 `main.cpp` 中可按待批准的诊断模式打开 GYRO/ACCEL。采用原 Pose 回调分流，Pose、motion 和鱼眼各用适合的有界队列。回调只复制小样本并交给消费者，不在回调写盘、做墙拟合或调 SDK 轮速发送。启用 callback 的入口从原 Pose 队列取数据，保持普通 Pose-only 消费可独立回退。先核实裸 motion 回调和 frameset 中子帧的实际交付方式，按 profile/帧号去重，逐流测有效频率。优先保原 Pose 与注入通路，在诊断溢出时记录丢失并使该对照窗为证据不足。 [原 Pose 消费][rdk-callback] · [现有 Pose 时间映射][rdk-pose-map] · [SDK 官方 API][sdk-motion] · [时间采用规则][repo-rules]

任何候选诊断都用已经到达的真实数据，窗口决定何时能出结论。保持现有 gyro 区间配对，增加原始 IMU 后不额外卡住反馈等待配对，不用 age、未知时间质量或日志可用性作为接收条件，不追加时钟握手、定时巡检或超时自动复位。延迟、丢失和覆盖检查用于本次残差含义或经另行批准的性能测量。

## 5. 检测器 A/B/C/D 的计算接口

每个残差输出 `{session, ref_revision, source, interval, window_id, components, raw_residual, covariance_or_scale, score, verdict, evidence_dependencies}`。`verdict` 为 `AGREE`、`CONFLICT` 或 `UNOBSERVED`。前者限定在本通道和运动分量，后者带缺覆盖、退化、弱激励、换参考等具体原因。给出速度、转角和位移原量，方便区分模型错误与真实滑移。

### 5.1 A 与 B 的旋转分支

同一 `[t0,t1]` 累计唯一、连续的 07 $\Delta\psi_e$。A 的参考 $\Delta\psi_g$ 由 08 的 `rate_source_id`、`rate_receive_us` 按区间线性插值和梯形积分求得。可把 08 累计 yaw 差另外列出，但与 rate 共用模块。B 用 T265 原始 gyro 去偏、旋转到 B、推进窗口姿态后比较 yaw 增量。在近水平条件下也可积分 $\omega_z^B$，显著侧倾或俯仰下用姿态 yaw 增量并核查平面模型的适用性。以 $r_\theta=\Delta\psi_e-\Delta\psi_g$ 为有符号 rad 残差，并报告 $r_\theta/\Delta t$。对照建议 50/100/200 ms 多窗口和原 10 ms，先取非重叠统计。按 $L=0.130\,\mathrm m$，三轮和变化一计数在 10 ms 约等效 3.04°/s，窗口加长能降量化影响，同时提高确认所需运动时间。原始 gyro 与主 gyro 的冲突优先作为参考链问题复核。 [角差量化与 07/08 回放][repo-gyro] · [主 gyro 相位对照][repo-sync]

### 5.2 B 的平移速度增量分支

选取已接收轮速估计的两个代表时刻 $t_a,t_b$。若用 07 平均速度，先将其对应区间中点，再用相邻已接收区间平滑/插值，轮和 IMU 必须采用同一端点定义。允许将评估端点置于已有共同支撑内。以实际采样间隔按时间对加计梯形积分，用 gyro 插值姿态到加计节点，在边界截断，记录真正用了多少不同 accel 帧。80/160/240 ms 可作离线网格（名义约 5/10/15 个新加计间隔），重采样不会增加独立信息。用 $\Pi_{xy}$ 投影 G 的水平分量，计算

$$
\begin{aligned}
\mathbf v_{I,e}^G(t) &= R_{GB}(t)\bigl([v_{x,e},v_{y,e},0]^T
                    +\boldsymbol\omega_{\mathrm{ref}}^B(t)\times\mathbf r_{BI}^B\bigr),\\
\Delta\mathbf v_{\mathrm{imu}}^G &=
  \int_{t_a}^{t_b}\bigl[R_{GB}(t)R_{BI}(\mathbf f^I(t)-\mathbf b_a^I)+\mathbf g^G\bigr]\,dt,\\
\mathbf r_{\Delta v} &= \Pi_{xy}\bigl(\mathbf v_{I,e}^G(t_b)-\mathbf v_{I,e}^G(t_a)
                     -\Delta\mathbf v_{\mathrm{imu}}^G\bigr).
\end{aligned}
$$

主假设为在 G 同点比较轮端点速度差与惯性积分。$\boldsymbol\omega_{\mathrm{ref}}$ 可取同一原始 gyro 用于杠杆，因而杠杆和姿态共享噪声。该通道检测加减速/转向相关的差异，特别是空转起始、受阻和急停瞬态。均匀速度误差、时窗内误差相消或 IMU 剧烈震动时，低残差不能作为 XY 恢复。方向正负、杆臂、同时间窗和漏采分别做离线对照。

### 5.3 C 的位移分支

同一参考点、同一窗口积分 $\Delta\mathbf p_e=\int \mathbf v_{I,e}^G(t)dt$。独立初速可取得时，用 $\Delta\mathbf p_I=\mathbf v_0^G\Delta t+\int_{t_a}^{t_b}(t_b-t)\mathbf a_I^G(t)dt$，比较水平差，速度先验误差以 $\Delta t^2 P_{v_0}$ 进入位移协方差（共享数据另计交叉项）。初速可来自独立静止事件（体心静止且无转动）、未用轮约束的短窗视觉/墙速度，或显式标记依赖的短时估计。以同一编码器建立 $v_0$ 会隐藏初始等速误差，只能作敏感性对照。优先离线比较 B/C 的相同动作和检测时延，无法得到可解释的初速就把 C 判为不可用。该分支适合比较位置量级与 Ground-Fusion 的迁移成本。 [Ground-Fusion 论文及源码][gf-paper] · [累积依赖][gf-prop]

### 5.4 D 的持续平移分支

优先复用已接受的双轴墙面位置，在同一参考坐标系算 $\mathbf r_w=H_o[(\mathbf p_e(t_b)-\mathbf p_e(t_a))-(\mathbf p_{\rm wall}(t_b)-\mathbf p_{\rm wall}(t_a))]$。$H_o$ 用两个端点共同可观的单位法向作行。在墙角/两非平行墙可取 rank 2，单墙用 rank 1，只说明法向，沿墙为未知。rank 0、误匹配、几何重获首帧、遮挡、基线先验变更或坡面破坏假设时只记录证据缺失。保留墙—轮和墙—T265 两个残差，以及当前主 yaw、去畸变/先验来源。若用同一条轮轨迹约束墙解，低残差可能受选择偏倚，应在离线用原始束自由拟合车心、替换先验或独立外部真值核对。不把当前 `BOTH_AGREE` 当完全独立认证。首版若复用现有裁决，只接受完整 XY，rank 1 诊断和独立几何对照另列。 [现有墙裁决][rdk-wall] · [既有先验偏倚分析][repo-priority] · [可观子空间方法][censi]

D 的替代参考可以是用独立地标/标定鱼眼作短窗相对运动。若使用单目需独立尺度，纯单目特征流也可能被动目标和旋转污染。保留特征覆盖、内点与尺度质量，输出所能观测的分量。无注入 Pose 是对照组，它仍使用同一 IMU，缺视觉或重定位时仍会漂移/跳变。轮注入开启或刚停送时的 SDK Pose 仅可参与仲裁参考健康度，不能单独认证轮已恢复。

### 5.5 残差尺度和判定

对角残差用 $J_\theta=r_\theta^2/\sigma_\theta^2$，对 B/D 用 $J=r^T S^{-1}r$。按原轮计数量化、gyro/accel 噪声和 bias、安装/轮臂误差、时基偏差、窗口长度、速度端点相关性及共享姿态传播建模。对差分 $r=x-y$，$S=S_x+S_y-C_{xy}-C_{xy}^T$。未知交叉项先用正常样本对整个残差拟合经验尺度并设测量分辨率噪声下限，不把独立模型的卡方置信度套给相关证据。用 Cholesky 求解，奇异时只保有支撑的秩或给 `UNOBSERVED`。统计判定可比较分工况经验分位或验证过的卡方分位。候选参数 `(window, min_excitation, J_enter, J_exit, confirm_enter, confirm_exit)` 中 $J_{\rm exit}<J_{\rm enter}$；标定用干净正常数据和独立异常真值，报告重叠窗相关性。以独立动作/非重叠窗计数，不把滑窗相邻输出当独立重复证据，不拿 P95 当最坏边界，也不拟合测试集来宣称准确率。 [统计检验参考][ov-src] · [仓库误判边界][repo-todo]

## 6. SDK 停送、恢复和运行语义

### 6.1 待选状态机

维护 `rotation_suspect` 和 `translation_suspect`，各自保存最后冲突区间末端、参考版本、触发原因、计数和恢复证据。`UNOBSERVED` 既不触发新增暂停，也不清除已经确认的异常。下面给的是可审查的转换，窗口、门限和持续影响均由负责人选定。

| 可用证据 | 本地状态转换 | 拟议 SDK 动作 |
| --- | --- | --- |
| 新会话、正常或暂时无足够观察，当前无本地确认异常 | 开始时各残差待观察。对照窗口标记 `AGREE` 或 `UNOBSERVED`，静止/弱激励分开报告。 | 由已有输入校验和阶段/墙策略决定。允许时送当前真实轮速。 |
| 确认角冲突，XY 无独立冲突 | 记旋转异常，查轴向、轮臂、滤波相位与纯共同分量误差。 | 建议先只记录角异常。若选择“角冲突单独停送”，须另验其收益大于误停，届时按所选原因暂停整组 SDK 速度。 |
| 可信动态 XY 残差重复越过进入阈值，或获可靠的双轴墙—轮反证 | 记平移异常。区分经独立证据确认和仅模型不一致。双源互相冲突、参考异常或墙秩不足，保留歧义。 | 经批准的确认条件成立后暂停后续样本。强 B 证据能否单独停送，以及单轴反证能否触发保守停送，属于显式策略选项。 |
| 角冲突结束，出现冲突结束之后、有转动激励的低角残差窗 | 清旋转原因，要求使用可靠 gyro，使用同一参考版本并按独立窗口计数。静止的零角差只作静止证据。 | 如果旋转原因被选择用于暂停，仅释放这一个原因。 |
| XY 冲突结束，出现新的完整 XY 真实位移对照 | 以 `baseline_ns >= last_conflict_end_ns`、`reference_ns > baseline_ns`、同会话/同参考和新扫描为因果条件。通过有运动量的墙—轮一致性，按评审确认数清除平移原因。可考虑经验证的独立视觉替代。 | 一致的加速度增量、旧扫描迟到、单墙沿墙方向或 SDK 置信度回升都不能清此原因。缺完整证据时保持已确认的暂停，记录仍缺哪个方向。 |
| 重获、参考切换、断开或重连 | 同一 SDK 会话内切断旧窗口并作废旧恢复证据，不能借参考变化或安全区退出解除确认异常。全新 SDK 会话仅按其新数据和现有业务策略开始。 | 恢复依据和手工回退边界明确写入选型。没有定时自动续送、复位 SDK 或强制重启。 |

对于平移恢复，可先以原完整 XY 墙裁决为基础，复用 `BOTH_AGREE` 的条件及依赖标记。若要支持“T265 此刻坏、墙仍可证明轮式恢复”，需新增可独立审计的墙—轮双轴证明和门限，评审后才允许把现有 `T265_SUSPECT` 或墙反证用于清本地暂停。证据长期不足时选项是维持暂停、使用已验证独立替代，或由人明确回退到原策略。回退只退出本候选的本地暂停，仍遵循原安全区和墙可疑策略。停送时车仍可能在运行，需测该影响。

### 6.2 放到唯一发送点

~~~text
收到 07/08 或 T265 motion 后，用已有到达数据更新诊断
收到新墙证据后，按 session、reference 和运动区间更新对应恢复状态
消费正常路径准备好的当前真实轮速
existing_policy_pause = 原阶段屏蔽 OR 原墙面轮可疑
local_slip_pause = 已批准会暂停的本地旋转/平移异常
effective_pause = existing_policy_pause OR local_slip_pause
若暂停，记录已消费且本次无 SDK 调用
若放行，按原数值、相机系、id 0 和发送序号送当前轮速
~~~

接线点是 `DebugWheelInput` 的 worker 和现有 `process()` 调用点。计算以采集到达事件驱动，SDK 调用前读取同一所有者的暂停状态。对检测/IPC 生效前已提交的样本记录为已送，不能撤回。暂停期间继续消费和审查真实 07/08、保留原始累计，但不把被拒样本积压给 SDK。恢复只允许新的正常发送机会，不补停送区间，不生成零速，不乘权重，也不为该样本临时重写标定。原生 `latest()` 遇到 `waiting_for_bracketing_gyro` 可暂存尚未配齐的 07，已有 `process()` 去重只覆盖已消费样本。如果选门控，应让已处在暂停区间的 07 即使后来配齐也只用于诊断。实现上可在 owner 应用策略时以当前已接收批次和每 epoch 的 07 ID 为边界，标记 SDK 送样代次，复用原序号回绕处理。继续消费、诊断和留原累计，恢复只送恢复之后首次正常接收的候选。边界只区分停/续批次，不按数据年龄丢样。API false/异常仍沿现有按样本记录与后续运行，不能借判滑新增控制拒发、清 `VALID/ready`、`STOP` 或 `FAULT`。静止真正测得的零速仍按正常语义送。 [现有停送、实验模式和 SDK 调用][rdk-wheel-send] · [时间采用规则][repo-rules]

## 7. 墙证据交付与接线任务

### 7.1 所有者二选一

| 接法 | 任务切分 | 代价与验证重点 |
| --- | --- | --- |
| **SDK 所有者持有本地暂停** | `main.cpp` 分流原始 IMU。`F405WheelMeasurements::feedback()` 的 07 路在合并前给窗口，或用同进程小结构体旁路，保留 `prepare()` 的主 gyro 语义。`DebugWheelInput` 持有本地状态。由 `motion_runtime.cpp`（及 Python 回退 `worker.py`）透传墙端点/证据，SDK 接收处消费。 | 决定最接近 `send_wheel_odometry`，最小范围可只影响 SDK 注入。上位机传播仍可能吃到坏轮。接收需处理扫描迟到和不同参考，墙信号频率与计算延迟决定恢复能力。 |
| **原生传播所有者持有裁决** | SDK 进程给 `motion_runtime` 单向发送有界的已计算残差摘要，复用原反馈/控制邻近入口，不搬整段 motion、日志或旧帧。`NativeMotionSource` 在同一墙/轮/Pose 历史中建立冲突后的新基线，`publish_policy()` 只发最终暂停状态及原因。若决定接入传播，使用这一处状态，同时覆盖 Python 回退一致性。 | 避免在 SDK worker 重写墙/传播逻辑，适合 E。引入 SDK→传播→SDK 跨进程路径，需比较真实生效延迟、丢包与回退，保留单个状态所有者。只有在此设计下墙端点不必发回 SDK 用于本地恢复。 |

两种接法都保持唯一 UART 和唯一 SDK 设备所有者。墙面与诊断不能在 UART 读回调、Pose 回调和 SDK API 临界区阻塞。默认正式不增加高频计时或新定时器，先用现有事件、队列丢失计数和离线回放查证开销。仅在有证据且批准后测新增路径。

### 7.2 选择 SDK 侧恢复时的本机消息调整

1. **生产端。** 保留原 `schema=3` 策略的 `pause/reason`，新增可选证据描述。按 `(policy_session, ref_revision, scan_id, baseline_ns, reference_ns, axes, verdict)` 或等价的单调证据代次识别新墙结果。带当次已接受两端点的 ID、两端点参考时刻、共同观测轴、残差/运动量、先验依赖和可信级别。先发有用的证据变化；同状态同原因仍可送新证据。Python 回退要同步实现，原生启用时保持只有一个策略发布者。
2. **接收端。** 先校验会话和消息类型，再独立处理状态变化及证据变化。同状态同原因只能省略状态变更日志，不能提前略过新的墙信息。按同一参考、窗口端点和证据 ID 去重。参考重获时重新建基线。旧消息无证据字段，仍按原策略处理状态，不计作新确认。单调版本防止迟到旧 `resume` 覆盖更新的暂停。
3. **因果与丢包。** 若发送端现有 `wall_window` 从冲突前开始，要等到真实的冲突后新基线。可以选择每次已接受墙端点单向送 SDK，由接收端按冲突边界挑两端点。也可以让原生所有者消费残差，直接在本地建立新基线。两次确认按有支撑的独立区间或已知重叠的序列规则计算。乱序、丢包、合并不补造确认次数，下一次自然事件可继续发最新状态，成功 sendto 只代表本机传输。没有新 MCU 报文、ACK、资格等待或定期重试。仅在有新策略时复用已有唤醒；如果沿原反馈处理节奏应用，验证下次 SDK 调用前读到的策略和可能多送的样本数。 [原生去重][rdk-policy-pub] · [Python 回退][rdk-policy-py] · [SDK 接收与线程][rdk-policy-rx] · [墙窗当前构造][rdk-wall]

## 8. 上位机传播与更深融合选项

### 8.1 E 的分量级仲裁

首先区分三个量：T265 C 点的真实 SDK Pose，导航用的 B 点场地位姿，按测量反馈推进的轮式尾段。方案只做 SDK 门控时，`WHEEL_FALLBACK` 和控制输出仍由原逻辑决定。要解决双异常，需另批上位机状态传播。候选行为如下。

| 轮式、SDK、墙分别可用的组合 | 传播候选 | 证据和副作用 |
| --- | --- | --- |
| 轮式与 T265 相容 | 保留 T265 锚点 + 当前轮式尾段，主 yaw 沿原参考。 | 对照原正式起点、动态入口和阶段屏蔽。 |
| 轮式有 XY 反证，T265 连续且墙支持 | 用当前 `T265_PRIMARY_WHEEL_SUSPECT` 的连续重锚思路，后续轮尾段不写导航，主 yaw 与可观墙修正保留。 | T265 先前融合坏轮的影响仍可能在，输出记录依赖。 |
| T265 跳变/失跟踪，有足够证据支持轮式 | 用原 `WHEEL_FALLBACK`，只把实际支持的轮式区间作为增量。 | SDK 失效自身不能判轮好，保留当前冲突首因。 |
| 两路均可疑、墙 rank 1/2 或无墙 | 可保持原退路并显式标为两路冲突，或以最近连续锚点为基准，仅让可信传感器更新其可观轴。rank 1 给墙法向，不填沿墙为零速。无可信 XY 时单列不确定度；若考虑冻结 XY，需评估控制仍消费的 0x16 及到位/距离判断。 | 缺独立信息时无可靠二维真值。源码不支持“总切 SDK 速度会更准”的推断。冻结位置和改控制输出都属于行为变化，需单独批准。 |

从最后可信锚点向前只积对应可信区间，恢复时按同一时刻的连续选定 XY 重锚，不补被排除的历史轮段。保持高频传播与低频墙几何职责分离。建议回放原始两场中双异常后的两个墙端点，已有轮式残差 14/37 mm、SDK 速度积分 36/80 mm，是与相关墙参照的局部差值，足以否决无条件切 SDK 速度，不能据此排名绝对精度。 [现有降级与恢复][rdk-fallback] · [两场比较][repo-priority]

### 8.2 E 的连续降权

如选择拥有协方差的外部估计器，可从平面误差状态 EKF（或固定窗）开始。状态含车心 XY、yaw、局部速度与必要偏置。选 raw IMU 或经证实的传播源作预测，用观测方程 $\mathbf z_e^{B}=\Pi_{xy} R_{BG}\mathbf v_B^G+\boldsymbol\eta_e$ 融入轮体心速度，用墙法向和已知墙位置做投影观测。先以独立预测创新计算门控，再使用创新接受的完整原轮速度更新。对中等冲突可用 $R_{\rm eff}=R_{\rm wheel}/w$，$0<w\le 1$ 为经标定的分量或总体信息缩放，严重时删相应更新。在固定窗可用白化后的轮残差和 Huber/Cauchy 等稳健损失。实际支持分量的维数和噪声来源一并记录。

同一轮样本如果已参与传播，再把它视为独立轮更新会重复使用信息。注入式 SDK Pose 与本机轮速、原始 IMU 也共享误差。可按来源选择单一约束、构造独立测量或显式建交叉协方差，并做留出一致性检验。把已融合 Pose 按一个固定权重与轮轨迹直接相加会掩盖这些相关性。本方案在主机内改变测量信息量，SDK 的动态接口仍为原速度送或停送。 [OpenVINS 创新推导][ov-doc] · [自适应噪声实例][fuzzy]

### 8.3 F 的显式滑移估计和全套 VIWO

将 ROSE 的轮径/轮距、滑移分开这一点迁到三轮。每个短窗口定义 $\mathbf s=[s_1,s_2,s_3]^T$，通过 $\boldsymbol\xi=A(\kappa)^{-1}(\mathbf u-\mathbf s)$ 在 SE(2) 预积分。以已标定的 $\kappa$（各轮比例、轮向和 $L_i$）线性化，累积对 $\mathbf s,\kappa$ 的 Jacobian，残差同时接可信 gyro yaw、可观墙位移和独立视觉。内参固定起步，后续仅在多方向、正反和多转速、参数可观的无滑数据里慢更新；对 slip 给可饱和的零先验和有限窗模型，按资料选择鲁棒核并验证收敛。没有独立 XY 位移时，三轮滑移不能由一个 yaw 约束唯一求解。轮臂比例、gyro 比例、速度尺度、时差也可能互相代偿。若出现秩亏、边界或多解，输出可疑并保留不确定分量。对这种候选应先比较真值误差、计算负载和病态窗行为，再讨论是否让估计速度影响主机导航或 SDK。 [ROSE 原始模型][rose-paper] · [VIWO 标定可观性][lee]

另一条路线是在主机运行完整 Ground-Fusion、OpenVINS 派生 VIWO 或 Ground-Fusion++ 式双子系统。需要单独评估原始鱼眼帧率、标定、同步、前端视觉质量、解算 CPU/内存、与当前正式控制接线的兼容性。Ground-Fusion 的 RGB-D 前端如照用需独立深度来源。改为现有 T265 双鱼眼和原始 IMU 需要重新做视觉前端/轮模型，并为现有 2D 围栏建约束。单轴精密 gyro 仍可保留主航向，若用于三轴 IMU 融合只能在轴向和时序明确后替换相应分量，传播每轴不确定度。不能把 D435i 或额外六轴当作 A/B 所需硬件。训练式滑移补偿和电流/动力学法还需要明确地面、负载、轮接触标签或新增数据来源，优先级由负责人按证据决定。 [Ground-Fusion 系统][gf-paper] · [Ground-Fusion++ 系统][gf2-paper] · [GPR 原文][gpr] · [全向轮滑动力学][omni]

## 9. 选项、成本和建议的拆分

下表是可独立批准的工作包。A/B/C/D 生成证据，E/F 处理主机融合或参数估计。G 是完整系统迁移路线。SDK 门控必须另选是否启用、接法及恢复策略。

| 选项 | 最小任务与可交付物 | 覆盖 / 盲区 | 对现有运行的影响和选用条件 |
| --- | --- | --- | --- |
| **S0 保留现状** | 对照已有墙判滑、业务阶段屏蔽和 `WHEEL_FALLBACK`，作为全部回放的基线。 | 已有完整 XY 墙条件下能仲裁；缺墙、延迟及双异常仍有边界。 | 零接线，适合暂不增加行为变化或没有原始证据时。 |
| **A 主 gyro＋三轮角差** | 复用 `wheel_gyro_residual.py`、07/08 和既有相位对照，输出旋转残差及每动作错误分类。 | 旋转一维，量化、轮臂、相位混淆。等角平移滑移不可见。 | 成本最低，可直接用已有数据。角冲突单独停送需要单独选和验证。 |
| **B T265 原始 gyro＋accel** | 同 SDK 进程采样，验证 profile、时间与外参。计算 `r_theta/r_dv`，先回放/旁路，再评审 SDK 门控。可先选 raw gyro 子集。 | 覆盖转动冲突和瞬态加减速；持续等速 XY 偏差仍需 D，坡面/振动降可信。 | 无新增硬件，增加 motion 约 262.5 样本/s 名义输入和有界处理。必须比对 Pose 和反馈→API 的长尾。 |
| **C Ground-Fusion 位移判据** | 用与 B 同一批原始数据实现双积分位移对照，明确独立初速和方向。 | 初速可信时提供短窗位移证据。初速和倾角误差可压过信号。 | 候选对照或在明确独立初速条件下用。无独立初速时优先 B。 |
| **D 墙或独立视觉平移证据** | 首选复用已接受双轴墙和原始束离线对照。需时将墙证据按第 7 节送达。进一步可用 rank 1 法向或独立视觉相对位移。 | 补持续平移、提供恢复证据，覆盖受墙几何/可见性和视觉质量限制。 | 保留现有单墙修正。无观测时不能承诺自动恢复；独立视觉增加前端和校准工作。 |
| **E 原生选源或外部稳健融合** | 先做分量仲裁/双异常回放；需要连续降权再估计协方差和创新。 | 能同时处理 SDK 退化和坏轮对导航的传播。联合失效时仍只能使用可观分量。 | 涉及 `NativeMotionSource`、Python 回退和正式控制发布，影响到位误差及连续性。单独批准与回归。 |
| **F ROSE 式三轮 slip/参数估计** | 先固定几何离线验证可辨识性和三轮 slip，后评估慢变几何和求解器。 | 足够外部 XY＋yaw 才能区分三轮、模型误差与滑移。 | 最高建模与标注成本之一。输出估计和置信范围，后续是否接实时需另审。 |
| **G 完整 VIWO / 多子系统** | 对照 Ground-Fusion、OpenVINS 派生和 Ground-Fusion++，单独比较相机前端、2D 墙约束与软硬件负载。学习补偿可列为此后有标注数据的试验。 | 接管状态估计可更完整地处理相关性和退化，但依赖更多同步、几何与算力。 | 与既有嵌入式 T265 和正式链差异最大，独立立项评估。 |

若负责人选最小可评估路线，里程碑可定为 `A 或 B 旁路 + D 双轴证据 → 仅 SDK 门控的离线状态回放 → 经实证再选运行门控`。若主要目标是解决 `WHEEL-ARBITRATION-02` 中双异常影响导航，则在保留上述证据的同时单列 E。通过 SDK 门控的验收不能代替传播验收。

## 10. 验证矩阵与验收记录

以下验证为获准实施后的工作计划。每一阶段先按固定输入复用现有入口，分开记录软件、实机条件和独立真值。现有报告可作历史对照。 [既有离线入口与验收][repo-priority] · [原停/续报告][repo-pause] · [仓库流程][repo-workflow]

| 阶段 | 必要输入和案例 | 观察结果与通过条件 |
| --- | --- | --- |
| **离线通路与符号** | 复用 `wheel_gyro_residual.py` 和原会话 `motion_source_review.py`。覆盖静止、前后、左右、斜移、正负原地转、混合运动、低速量化、刹停、坡面、安装倒置、缺帧和参考重获。加两类反例，三轮误差和为零的持续 XY 滑移，以及三轮共同误差导致仅角差。 | 检查符号/单位、C/I 与 B 参考点，纯转无假平移、运动区间完全匹配。前者角差盲区按设计暴露，后者不能在纯角诊断中误判为已证明 XY 坏。原始输入文件按只读处理。 |
| **有标注的检测评估** | 获准后按既有 MotionDebug/正式入口记录不同表面、载荷、速度、光照和墙可见性，使用独立地面标记、外部相机或测量装置作位姿/角度基准。滑移案例覆盖短暂失附着、持续空转、横移、抵住不走、单轮可疑、真静止及两路同时坏。 | 按会话/场景划训练和留出，标签独立于算法输出。报告误报事件/正常运动分钟、检出事件率、未观测比例、各方向错判、首个有效证据到决策、真实异常开始到停送，以及恢复后首个稳定可用样本。报告样本量/区间和漏检，别只统计已检出延迟。 |
| **无设备软件回归** | 复用已有 `test_omni_localization.cpp` 的发送回调与策略检查、`competition_rescue/tests/test_runner.py` 的原生/回退及控制回归。加入原状态不变但新墙证据到达、重复、乱序、跨参考迟到、阶段切换、本地与现有暂停并存、断流、队列合并、暂停期 07 到恢复后才配齐 gyro、API 单样本失败及重获。 | 对一份固定事件序列核对每次决策和唯一发送。暂停期 API 调用为零，原反馈仍消费，恢复只从新发送机会起。旧/缺失墙信息不能清本地原因，原模式未启用时 SDK 调用序列、正式起点、主 yaw、0x16 与 STOP 行为保持基线。 |
| **同 SDK/固件动态对照** | 经独立授权在相同外参、比例、pose jumping、动作/环境下做 T0 无注入、T1 连续真实注入、T2 无滑正常运动按明确区段停/续、T3 已标注滑移用候选门控。增加 T4 阴影模式判定但仍按旧策略发送，可分离新增流负载。无注入对照应从干净的独立会话开始，并记录会话和地图保留设置。 | 对每组保存实际 SDK/固件、帧计数和真实调用数，比较短/长停送及恢复瞬态、姿态/位置跳变、区间位移和绝对轨迹误差、围栏重获。比较 T1 与 T2 才能识别停/续本身副作用。记录独立真值方法和不可比较段。 |
| **导航和联合负载** | 只有选 E 或涉及正式路径时，复用 07/08、原 Pose、雷达和正式控制回放，重点覆盖安全区进入/退出、雷达遮挡、错误墙、SDK 跳变＋轮滑。获准后在相同负载测真实控制路径。 | 原阶段屏蔽和人工 STOP 保持。无新增虚假 0x16 或控制停发。报告位姿连续性、可观轴、到位误差及双异常退路。用已有记录口径比较轮反馈→SDK、Pose 接收→消费和控制输出的 P50/P95/最大、丢包/合并和队列峰值。由负责人先定容许回归和位置误差目标。 |

提交后续实施结果时可按此格式记录。**采纳范围**（A–G、诊断/SDK/导航）、**证据区间**和**恢复依据**、**基线/候选版本与实际设备身份**、**留出误报和漏检**、**正常与异常的定位误差/尾延迟**、**剩余盲区和回退方式**。无真实标注、原始 IMU 或有效独立墙/视觉的场景保持未知。本次 RFC 审查未采集设备或电机数据，也未运行候选定位程序。

[rdk]: https://github.com/danmo-teng/shijue_fangan/tree/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19
[rdk-stream]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/main.cpp#L1377-L1397
[rdk-callback]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/main.cpp#L1493-L1522
[rdk-pose]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/main.cpp#L1689-L1740
[rdk-pose-map]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/main.cpp#L1813-L1827
[rdk-wheel-decode]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/f405_wheel_measurements.cpp#L57-L85
[rdk-wheel-prepare]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/f405_wheel_measurements.cpp#L145-L225
[rdk-wheel-worker]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/debug_wheel_input.cpp#L129-L151
[rdk-wheel-latest]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/f405_wheel_measurements.cpp#L232-L243
[rdk-wheel-config]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/debug_wheel_input.cpp#L331-L351
[rdk-wheel-send]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/debug_wheel_input.cpp#L416-L482
[rdk-phase]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_source_native.cpp#L101-L128
[rdk-native-pose]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_source_native.cpp#L156-L176
[rdk-fallback]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_source_native.cpp#L229-L324
[rdk-wall-apply]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_runtime.cpp#L511-L525
[rdk-wall]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_source_native.cpp#L367-L415
[rdk-pause-flag]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/include/motion_source_native.hpp#L49-L50
[rdk-policy-pub]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/motion_runtime.cpp#L569-L581
[rdk-policy-py]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/tools/lidar_yaw_debug/worker.py#L364-L384
[rdk-policy-rx]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/localization/src/debug_wheel_input.cpp#L129-L162
[f405-entry]: https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/41f23a605bbd36c2a48fae2eb85022da5a4c44af/docs/CROSS_REPO_REQUESTS.md
[f405-telem]: https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/41f23a605bbd36c2a48fae2eb85022da5a4c44af/Main/Src/DebugTelemetry.c#L116-L153
[repo-rules]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/AGENTS.md
[repo-context]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/LOCALIZATION_CURRENT_CONTEXT.md
[repo-control]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/LOCALIZATION_NATIVE_CONTROL_20261109.md
[repo-pause]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/reports/localization_native_pause_20261109.json
[repo-gyro]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/WHEEL_GYRO_SLIP_ANALYSIS_20261109.md
[repo-sync]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/analysis/sensor_sync_20261109/ANALYSIS.md
[repo-priority]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/analysis/priority_todo_20261110/README.md
[repo-todo]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/PROJECT_TODO.md
[repo-workflow]: https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/CHANGE_WORKFLOW.md
[sdk-motion]: https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md#L78-L113
[sdk-profile]: https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-device.cpp#L482-L502
[sdk-datasheet]: https://www.realsenseai.com/wp-content/uploads/2019/09/Intel_RealSense_Tracking_Camera_Datasheet_Rev004_release.pdf#page=12
[sdk-datasheet-old]: https://www.intelrealsense.com/wp-content/uploads/2019/04/Intel_RealSense_Tracking_Camera_T265_Datasheet_Rev002.pdf#page=10
[sdk-white]: https://dev.realsenseai.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera/
[sdk-253]: https://github.com/realsenseai/librealsense/blob/v2.53.1/src/tm2/tm-device.cpp#L1799-L1837
[sdk-remove]: https://github.com/realsenseai/librealsense/pull/11287
[sdk-api]: https://github.com/realsenseai/librealsense/blob/v2.50.0/include/librealsense2/h/rs_sensor.h#L572-L592
[sdk-cpp]: https://github.com/realsenseai/librealsense/blob/v2.50.0/include/librealsense2/hpp/rs_sensor.hpp#L629-L660
[sdk-tm-send]: https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-device.cpp#L1818-L1837
[sdk-calib]: https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md#L163-L211
[sdk-example]: https://github.com/realsenseai/librealsense/blob/v2.50.0/wrappers/python/examples/t265_wheel_odometry/t265_wheel_odometry.py#L13-L61
[sdk-tm-config]: https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-device.cpp#L1799-L1816
[sdk-coord]: https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md#L13-L34
[sdk-extrinsics]: https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-device.cpp#L548-L565
[sdk-jump]: https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md#L146-L159
[gf-paper]: https://arxiv.org/pdf/2402.14308#page=4
[gf-check]: https://github.com/SJTU-ViSYS/Ground-Fusion/blob/665cecd17964d929fb9fa4c3a01e9c5b8cd2f7cb/vins_estimator/src/estimator/estimator.cpp#L591-L636
[gf-prop]: https://github.com/SJTU-ViSYS/Ground-Fusion/blob/665cecd17964d929fb9fa4c3a01e9c5b8cd2f7cb/vins_estimator/src/estimator/estimator.cpp#L754-L838
[gf-factor]: https://github.com/SJTU-ViSYS/Ground-Fusion/blob/665cecd17964d929fb9fa4c3a01e9c5b8cd2f7cb/vins_estimator/src/estimator/estimator.cpp#L3120-L3139
[gf-margin]: https://github.com/SJTU-ViSYS/Ground-Fusion/blob/665cecd17964d929fb9fa4c3a01e9c5b8cd2f7cb/vins_estimator/src/estimator/estimator.cpp#L3368-L3377
[ov-paper]: https://yangyulin.net/papers/2020_icra_ov.pdf
[ov-doc]: https://docs.openvins.com/update-feat.html
[ov-src]: https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterMSCKF.cpp#L195-L227
[rose-paper]: https://www.cs.cmu.edu/~kaess/pub/Potokar24ral.pdf
[rose-src]: https://github.com/rpl-cmu/rose/blob/556005669cb9f76b94669d0ad36361ef8978f4a6/rose/src/WheelRose.cpp#L23-L62
[rose-corr]: https://github.com/rpl-cmu/rose/blob/556005669cb9f76b94669d0ad36361ef8978f4a6/rose/src/WheelRose.cpp#L186-L226
[gf2-paper]: https://arxiv.org/pdf/2507.08364#page=4
[gf2-check]: https://github.com/sjtuyinjie/Ground-Fusion2/blob/aa1c8dc80b05df7a536f5759d2f7725ad12015a6/Ground-Fusion%2B%2B/vins_estimator/src/estimator/estimator.cpp#L681-L688
[gf2-repo]: https://github.com/sjtuyinjie/Ground-Fusion2/tree/aa1c8dc80b05df7a536f5759d2f7725ad12015a6
[quan]: https://arxiv.org/pdf/1804.04854
[lee]: https://yangyulin.net/papers/2020_iros_viwo.pdf
[kilic]: https://arxiv.org/pdf/2207.13629
[censi]: https://censi.science/pub/research/2007-icra-icpcov.pdf
[gpr]: https://www.mdpi.com/1424-8220/25/5/1537
[fuzzy]: https://www.mdpi.com/1424-8220/24/23/7619
[omni]: https://people.ohio.edu/williams/html/PDF/IEEETRA02.pdf
