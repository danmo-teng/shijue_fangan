# 高频定位迁入C++：实现与实测边界

2026-11-09，基于93d6538增量实现。用户授权将Python高延迟部分迁入C/C++；本轮只处理定位输入、传播、历史和控制传输，不改变比赛业务或下位机。

## 结果与测量口径

完整会话：`rescue_map/runtime/history/t265_f407_motion_map/20261109_115931_interactive/`。
原入口同时运行真实F405、唯一T265 SDK、正式YOLO26、地图绘图和10秒历史雷达PTY回放。车静止；场地参考明确为合成计算参考。没有电机、STOP/reset、TASK CONFIG或实际底盘0x16发送。

以每个真实轮sample_id的**首次控制报文**计数；后续同一轮ID的Pose刷新另列，不能当成新增轮端点。雷达负载段1001个不同端点，内核入队约99.998Hz。

| 原始物理主机UART接收完成后的路径 | 中位ms | P95 ms | P99 ms | 最大ms |
| --- | ---: | ---: | ---: | ---: |
| C++处理→控制报文内核入队 | 1.529 | 2.442 | 2.997 | 4.464 |
| 内核入队→独立Python终端读取 | 0.198 | 3.915 | 7.466 | 15.837 |
| UART接收→独立Python终端读取 | 1.898 | 5.831 | 9.410 | 17.123 |

本段生产链端点没有超过10ms；端到端仍有0.599%的端点超过10ms。端点入队间隔最大20.43ms，不能将平均100Hz说成每10ms必达。全部同ID刷新报文的参考年龄与首次端点延迟分开保留，见派生报告。

终端用SO_TIMESTAMPNS取得实际数据报入队戳。每包原始CLOCK_REALTIME/单调时钟括号的公共偏移区间宽1294ns，全包统一转换，未删除慢样本。测量期间只在有界8192记录缓存中保存原报文及时间，结束后解析写盘：4692记录、零溢出。这是测量接收器的缓存，不是扩容控制队列或实际正式消费端。

102643基线按同一“首次轮ID”口径，雷达段UART→Python终端P95为24.428ms；初版native112218使用同一个旧终端记录器，降为6.265ms，约74.4%。最终115931为5.831ms，但同时改了底盘诊断日志执行位置和测量接收器，不能把全部差值归因于C++生产链。旧报告全报文P95 27.60ms不可与首次端点P95直接相减。

## 修改与所有权

- `localization/include/motion_source_native.hpp`、`localization/src/motion_source_native.cpp`：原MotionSource数值逻辑逐项迁入C++14，保留T265锚点、轮式尾段、跳变降级、双向墙证据判滑、停/续SDK策略和恢复阈值。没有另造位置权重融合。
- `localization/include/motion_runtime.hpp`、`localization/src/motion_runtime.cpp`：独立原生线程直接消费原07/08完整帧及原接收时间、原Pose流；执行时钟映射、传播、历史维护、原控制JSON序列化和本地发送。只复用已绑定socket，既不开第二个UART，也不开第二个SDK。
- `localization/tools/lidar_yaw_debug/native_motion.py`：ctypes窄接口释放GIL。Python只保留低频围栏几何决策、初始化和显示；历史查询先短锁复制数值，解锁后生成JSON。延迟墙修正在原生锁内基于当前历史提交，保留计算期间尾段运动及单轴可观性。
- `localization/tools/lidar_yaw_debug/dynamic.py`、`worker.py`：原初始化和墙修正只增加原生调用挂钩。native所有者启动后，Python不再消费同一路输入或重复发布控制/SDK策略。缺库或启动失败沿用明确标记的`python_original_fallback`，同时记录错误；该回退不具有本轮原生性能验证结果。构建正式SDK目标时同时构建原生库。
- `localization/include/priority_inheritance_mutex.hpp`、`src/main.cpp`、`src/dynamic_pose_relay.cpp`：运动线程及正式消费线程请求FIFO10；短共享锁和原UART发送锁使用PTHREAD_PRIO_INHERIT。权限或协议不支持时记录实际结果并沿原普通调度运行，不阻断样本。原UART发送50ms预算未改。
- `localization/tools/f405_test_client.py`：只有原生UART的被动测试观察支路复用已有ProcessDebugLog，将原JSON编码/写盘移出GUI解释器。保留原4096容量、全部记录及动作/SEQ配对，独立串口客户端仍用原AsyncDebugLog。
- `localization/tools/motion_log_storage.py`：仅登记新增的模块自有`native_motion_events.jsonl`，没有运行清理。

现实时路径是：唯一C++UART/SDK→本地原生运动线程→原正式JSON→唯一C++正式转发器。SDK注入已在同一C++SDK进程；雷达C++提线保留。高频输入不再等待GUI、Python MotionSource或Python JSON编码。低频几何和诊断仍有Python，不声称整个程序已全部C++化。

保持原安装参数/比例、pose jumping、B→C输入及C→B输出、SDK源T/W=0、正式起点参考、reference/revision、会话归属和任务配对。不新增数据年龄门、时间资格等待、握手或永久FAULT。未知时钟质量仍如实为false，不拒绝已收到的正确反馈。原两路时间区间配对属于速度计算，不用当前gyro替换区间所需gyro。

## 完整性与传感器结果

- 4692控制包与终端缓存、接收记录、control_pose日志逐包一致，ID连续；所选真实轮ID全部对应原始UART。原始底盘UART CRC错误0、丢弃字节0。
- native已消费范围内4884个有效07/08均有原接收时间的运动记录，无遗漏。全会话tap仍报告88丢弃：87个是原生消费关闭后的07/08尾包，1个是关闭订阅回执，含一次EPIPE；不把总tap计数写成0。
- 底盘被动观察支路31113条历史记录全部写完，零传输/写盘丢弃，final_receipt=true；native四个日志、原始CSV/UART和雷达诊断无写盘丢弃。session_complete=true、accuracy_verified=false。
- 实板自报14b2a3a9077e6980a540631ce94345e61eb0487d/dirty1，NormalDebug、MOTIONDEBUG/service1、921600；不是干净构建证明。全程414条状态为IDLE/fault0。
- SDK固件0.2.0.951/librealsense2.50.0：2475次API true、0 false，约99.91Hz；原始4864 Pose帧连续约199.57Hz，confidence均2。API true不是设备融合ACK或精度证明。启动缺区间起点gyro1次、SDK最新值合并2样本保留在日志。
- 雷达负载段配齐wheel+gyro→API返回P95 2.811ms、最大11.600ms；仅wheel接收→API返回P95 10.558ms、最大24.858ms，包含等待真正区间gyro。不能据生产链4.46ms宣称全部SDK输入小于10ms。
- 雷达98/98扫描计算，零拟合合并/原生回退；计算墙钟中位41.437ms/P95 56.309ms/最大67.444ms，数值子进程CPU中位12.054ms。该合成参考下全rank0，没有实际接受XY墙修正，不证明动态围栏定位精度。历史回放雷达原解析仍有2个CRC、3个原时标拒绝和1个扇区拒绝，未修改解析规则。
- YOLO平均37.14FPS；原指标stable_30fps=false。回放视频缓存丢2帧单列，视觉诊断写盘零丢弃，不能说视频也无损。Pose处理全段最大75.947ms、雷达负载段最大10.905ms均保留。

114804会话曾因原Python诊断日志争用GIL造成观察支路丢76容器，无完整标记，**未验收**。修复日志执行位置后才取得115931完整结果。中间所有原始日志保留，不修补标记、不删慢段；详见JSON报告的会话分类及SHA256。

## 必要检查与未完成项

构建`motion_runtime`、`t265_omni_localizer`及原`test_omni_localization`成功。原测试文件中新增原/新数值逐输入对照，覆盖跳变、双向判滑、恢复、单墙、历史尾段、真实本地socket与原控制格式、全历史初始化及身份失效。针对性Python8项通过；被动日志修复后底盘客户端/时间协议/日志恢复3项通过；没有新增独立测试程序。

原C++检查使用实际DynamicPoseRelay接收线程及假UART回调，FIFO10与关闭RT的普通调度两种都通过；只验证原帧和线程行为。硬件联合会话实读运动线程FIFO10和两处PI生效，但没向底盘发送正式0x16。

当前6.1.83是普通PREEMPT，不是PREEMPT_RT；没有重启或修改启动文件。局部FIFO/PI不是硬实时保证，短静止样本不能给出全负载最坏上界。当前剩余可见长尾包括Python终端调度、SDK返回/等待区间gyro及低频几何，需分开评价，不能扩大年龄阈值、伪造gyro或跳过日志来隐藏。

下一项验证保持原集中MotionDebug短操作单，不自动发车：真实短往返、必要一次转动、可观察墙面时核对原降级/重获及历史校正；地面精度、正式NormalRun的实际0x16链及刹停仍未验收。本轮无需下位机修改。

派生数据：[reports/localization_native_control_20261109.json](reports/localization_native_control_20261109.json)。其upper_commit保留测量时真实基线，候选源码/ELF由10项SHA256对应；交付提交为包含本报告的Git提交。
