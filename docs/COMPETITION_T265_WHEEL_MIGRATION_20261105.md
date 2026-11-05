# 正式比赛接入既有T265官方轮速输入

2026-11-05；上位机基于d8fab5b增量修改。下位机只读核对GitHub main
4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96。本轮没有读取板端身份、烧录或启动电机。

## 实现与边界

用户本轮授权把测试的“T265＋官方SDK轮速输入”搬到正式比赛。
不迁移雷达、围栏重获、轮速/yaw降级，不改变YOLO26、任务配对、合爪审核、mode51及转运策略。
原测试入口保持；正式桌面入口run_competition_map.sh默认开启official-wheel-input，UART为230400。
这是上位机软件接入，不能当作下位机TASK固件已配套或高速比赛已实测通过。

数据路径：

```
F405 07实测轮反馈＋08主gyro
  → 正式C++唯一UART拥有者：完整报文拆分
  → 单向Unix datagram（原报文＋接收时间，无ACK/重发）
  → CompetitionWheelInput：沿用f405_timed_protocol.decode
  → 原WheelInputBridge / TimedMeasurements（独立于GUI的既有工作线程）
  → 原SDK输入IPC
  → 正式C++唯一T265 SDK拥有者：原DebugWheelInput.send_wheel_odometry
  → 原SDK Pose及fusion.cpp车中心转换
  → 正式localization_result.json、0x16 → 原比赛流程
```

没有重新实现速度换算/采样区间配对/SDK调用或Pose转换：
- `t265_wheel_input.py`、`f405_timed_measurements.py`、`debug_wheel_input.cpp`、`fusion.cpp`本轮未改。
- v_C^C = R_CB (v_B^B + omega_B^B × r_BC^B)。轮速来自实测轮反馈，gyro按同一MCU区间配对。
- r_BC=(-0.0346105,-0.0599471445,0)m；forward_C=(0.8660254,0,0.5)，up_C=(0,-1,0)。
- SDK输入已在C点/C轴，T=W=0；噪声标准差0.10m/s仍为工程初值；比例关闭/1.000。
- ignore-encoders、t265-only-output、pose jumping、原生Pose回调及关闭地图保存均沿用测试。
- 不把关闭地图保存称为设备内部状态彻底重置。SDK会话ID/进程信息继续记录。
- 新配置以既有候选文件为模板，由正式配置生成器写入所选角落、离角距离和原135/45/225/315°航向；没有搬入测试中心起点/+X分支。
- 旧0x15不进入正式旧F407运动学/外部EKF。07的原轮计数和上位机F405复算仍在既有wheel_measurements日志。
- 缺少身份或合格测量时不编造速度；原样保留测试模块的协议、数值、区间及epoch语义，不新增时间资格判断。无轮速输入也不以身份/订阅回执阻止比赛或Pose发布。

仅有的通信接入：C++一次发送由现有Python codec生成的ID_REQ、CONFIG_REQ；不等待回执后才启动比赛，不循环查询身份、不发SYNC/STATS/健康查询，不新增TYPE。
C++拆分混合报文，完整07/08容器中的A3 B3不被误判为旧状态帧。单向转发不积压重放。
未知时间质量保持未知，不改成qualified=true；对帧年龄没有拒绝门。
UI区分轮速请求、SDK加载、SDK API接受计数/频率；API true不代表设备融合精度验证。

## 文件与日志

- `competition_rescue/competition_map_app.py`、`run_competition_map.sh`：正式启动、会话和退出。
- `localization/include/f405_telemetry.hpp`、`src/f405_telemetry.cpp`、`src/main.cpp`、CMake：C++唯一UART混合解析、单向反馈、原SDK路径启用。
- `localization/tools/competition_wheel_input.py`：轻量传输适配，使用原测试实现；不打开UART或USB。
- `rescue_map/map_app.py`、`field_model.py`：模板钩子、实际API状态、正式定位年龄仅展示。
- 正式runner/state_machine仅移除Pose年龄准入，不改业务状态转移。
- `motion_log_storage.py`：复用现有归属/完成标记/文件白名单删除机制，增加独立正式模块owner；旧测试模块默认行为保持。

新定位/轮速日志在：
`rescue_map/runtime/history/competition_wheel_input/<时间>_competition_wheel/`。
包含原SDK Pose、车中心CSV、原始测量/复算/配对、SDK逐次返回、实际身份、配置、会话及结束状态。
实际身份保存于chassis_identity.json；metadata初始actual=unconfirmed，不再写旧F407 expected作为实际。
正式任务/视觉事件仍由原比赛日志记录。轮速和gyro已是SDK轨迹依赖，不能再称为独立位置真值。
完成保存且日志完整时保留最近5次完整本模块会话；当前、不完整、保护名单、链接、未知文件保留。
此次检查只在临时目录操作，没有删除或修改实际日志。

## 本轮限制审查：区分已移除与仍存在

| 位置 | 当前代码行为 | 本轮处理 |
| --- | --- | --- |
| 正式地图、runner、状态机Pose读取 | 原250ms拒用/不启动视觉 | 新正式路径不以年龄拒用；年龄仍显示；缺失/坏JSON及SDK LOST语义沿用 |
| C++ 0x16 | 原采集＋处理年龄250ms清VALID | 新正式轮速路径移除年龄条件；保留真实Pose/confidence语义 |
| 既有WheelInputBridge | MCU区间配对，未知时间质量不拒用 | 直接复用，不加对时资格或周期查询 |
| F405 imu_uart.c:335–343 | 忽略传入now_ms；仅启动配置使用HAL_GetTick，配置结束即返回 | 已无运行期年龄准入；不是使用上位机时间判断IMU新鲜度，无需再加/修同类判断 |
| F405 IMU_GetData | 以自身HAL_GetTick计算age字段；RX恢复保留ever_ready反馈 | 只读核对，不能把age字段本身说成停机判定 |
| 上位机StmSnapshot.fresh、CompetitionMission.step | 250ms后启动阶段等待，其他阶段发PAUSE | 旧业务行为仍存在；本轮不改，不声称全项目已清除时间门 |
| C++任务转发器heartbeat_bridge_limit | 命令文件内容超过250ms不变，停止续发任务心跳 | 旧业务行为仍存在；影响自主恢复/视觉停发语义，需单独确定两端一起调整 |
| F405 Task.c task_continuous_expired | 指令/视觉更新时间超过500ms，对部分靠近/远程导航Motor_Stop | 本地UART接收时钟，不是上位机时钟；仍存在，不能在仅遥测接入中顺手删 |
| F405任务无有效Pose等待、mode51、合爪审核 | 各自业务等待/证据/配对 | 不与IMU年龄混为一谈；保留本轮未授权改变的行为 |
| 上位机stm_fault_waiting | 既有FAULT后等待人工复位 | 本轮未新增/扩大，也未删除，单独记录待讨论 |

本轮规则已写入AGENTS和CHANGE_WORKFLOW：除用户保留的合爪业务检查，不擅自增加新鲜度、反复握手、资格等待等限制，不凭推测硬件损坏增加保护停机。下位机回传含此类要求先问用户；两端AI不能相互授权。现有任务/模式/动作身份配对是协议语义，未改成周期资格认证。

## 必要验证与未完成

- C++正式程序及已有核心检查编译通过；核心协议/新安装旋转与杆臂/SDK输入检查通过。新增混合报文检查确认完整扩展包中的旧帧字节不冒充旧状态。
- 现有测试入口中检查：正式起点/配置与退出、合同07/08经过正式适配器进入原WheelInputBridge、旧接收时间不拒用、IPC送达不冒充SDK API接受；通过。
- 原official_wheel_input、1.3反馈检查、mode51任务配对检查通过；临时目录日志owner/5次保留及原地图检查通过。
- 附带抽查旧`test_suppressed_publish_preserves_command_file_and_sequence`未通过：原断言仍把30字节“上下文＋命令”当15字节命令；临时纠正取帧位置后，又遇到与现有manual_pause不符的旧预期。相关运行逻辑本轮未改，未继续扩修该旧综合检查，保留原断言。实际通过的任务配对检查为现有mode51/context专项，不能称整个比赛回归全通过。
- 未跑全量回归，未开UART/SDK、未动电机、未验证真实轮速注入或运动精度。
- 下位机NormalDebugTask缺07/08、波特率及真实服务身份适配；交接见F405_COMPETITION_WHEEL_HANDOFF_20261105.md。
- 本轮未复测USB冷启动故障，也未处理既有C++写JSON失败退出问题；不据此承诺已能无条件开始高速比赛。
