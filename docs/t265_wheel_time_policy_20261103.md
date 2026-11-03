# TIME-01A：官方轮速采用策略变更

基线a3ff98a。用户选择B，先推进轮速，不拆T265转换模块；没有批准首包参考策略改变。
本次只完成轮速输入通道的一个小提交，正式/测试其余数据读取点仍列TIME-01B，不宣称全项目完成。

## 接口和行为

- `f405_timed_measurements.py`保留实际07轮反馈、08原生CCW gyro和原始源ID/boot。
  wheel增加alignment_start_ns/end_ns=t0/t1_us×1000，gyro增加alignment_capture_ns=rate_receive_us×1000，
  两者alignment_clock=mcu_event。同一MCU事件区间直接用于配对，不依赖主机时钟offset更新。
  这些是已有事件时刻，不冒称IMU物理采样时刻；未知源延迟/漂移仍false/null并记录。
- `t265_wheel_input.py`保留同轮区间的gyro线性插值积分。去掉50ms年龄、5ms误差界和20ms相邻间隔准入门，
  主机映射缺失也可采用。仍需真实正长度区间、同boot/同事件时钟的gyro前后样本；
  没有对应gyro就等待真实数据，或在有界队列溢出时明确计数，不补零，不捏造同步。
- `f405_test_client.py`测量订阅不再等待成功SYNC；同步只提供诊断。普通运动身份/状态规则本轮未更改。
- Python→同一C++SDK所有者的本地IPC改为schema=2、time_policy=received_feedback_no_age_gate_v1。
  新字段alignment_*表示原始事件时钟，interval_start_ns/end_ns仍为可空的主机估计，二者不混算。
  capture_time_qualified和time_uncertainty_ns保留真实诊断值，绝不强填true或0。
- `debug_wheel_input.cpp`删除年龄/未知资格拒绝，按schema2检查来源、会话、区间结构、顺序和换算后真实调用
  send_wheel_odometry；日志保存API结果与接收后年龄。SDK frame_num仍为序号，不是时间戳。
- CRC、坏数值、真实测量身份、重复/跨epoch、错误坐标换算仍拒绝；有界队列、独立worker和异步日志保留。
  日志确实不可记录或SDK真实返回失败仍显式报告，不接成底盘永久锁存。
- 原地图说明改为“等待实测输入/时间诊断”，时间未知不再显示成“不能注入”。

没有采用目标速度或T265速度当输入；旧0x15仅作轮计数诊断，真实注入使用新07/08区间反馈。
保持`v_C^C=R_CB*(v_B^B+omega_B^B×r_BC^B)`、SDK T/W=0、噪声sigma0.10m/s、原安装参数、比例1.000和pose jumping。
没有修改fusion.cpp、正式定位配置、比赛流程或首个Pose原点。

## 必要验证

1. 原`test_f407_motion_protocol.py`通过。包含：旧数据年龄/未知误差仍可发送、100ms gyro间隔仅记录、
   同MCU源事件无SYNC时仍配对、真实测量来源、重复源样本不重放、正反轴向/杆臂、off会话不发送。
   本地IPC和模拟源只证明软件处理，不冒充SDK或实车结果。
2. 原构建通过，ctest 2/2通过。C++现有检查中假SDK接收旧年龄/未知质量样本、仍拒绝重复/换算错误，
   SDK失败按原输入流处理，不认为精度通过。
3. 仅尝试一次原入口6秒无运动检查，使用--wheel-input on --sensor-check-seconds 6 --no-log-prune。

真实日志：`rescue_map/runtime/history/t265_f407_motion_map/20261103_182925_interactive/`。

- 底盘UART RX=0字节；共6次身份查询写出并超时，actual=unconfirmed，新流未启用，没有任何实测速度/gyro。
- T265枚举为03e7:2150 boot设备，原启动器进行正常固件加载，0/9323648字节后15秒LIBUSB_ERROR_TIMEOUT。
- SDK未开始Pose/配置加载，速度API接受没有实际验证。会话无完成标记，退出记录localizer_returncode=-15。
  原启动器日志最后给出了上述真实USB错误；检查结束后未发现本轮遗留的SDK/boot进程。
- 只有身份查询；未发MOVE/TURN/STOP/reset/测量订阅，未启动雷达；没有重复空跑、没有日志清理。

这次阻塞是没有实际数据和USB启动失败，不是样本新鲜度。需要恢复板端通信与T265启动后，
再在原入口检查真实API接受计数；本次仅交付软件、保留实机待验项。

## 新增START-01

用户报告VNC点击开始到实际运行过慢。已记录一次明确的15秒USB固件传输失败等待，
尚未证明它解释所有历史延迟。后续先恢复此链路，再拆分启动、GUI、命令写出和板端动作开始耗时。
不能把首个有效Pose建议当成提高置信度等待，也不能靠盲缩USB超时冒充传输修复。

## 19:09真实接口复验通过

用户上电F405、重插T265后链路恢复。190328纯T265会话正常接收，但无运动/独立测量，不是有效A组精度实验。
190929使用原入口`--wheel-input on --sensor-check-seconds 6 --no-log-prune`，没有发任何底盘运动。
实际SDK配置加载成功，310次send_wheel_odometry均返回true，拒绝0；平均48.775Hz，日志无丢弃。
所有v_B为实际轮反馈的零速度，不是缺数据伪造零。310个样本capture_time_qualified=false仍被采用；
实际接收至SDK最大298.5ms也未按年龄拒绝。此结果证明输入API通道接通，不证明SDK内部融合或运动精度。
启动/收尾阶段32次IPC目标不可用、12次无gyro起点配对均保留，不重放。
TIME-01A接口项完成；TIME-01B其他采用点、交互简化和WHEEL-01地面测量继续分别列待做。
