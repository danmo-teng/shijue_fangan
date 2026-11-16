# 原始T265 IMU与地图显示

2026-11-16。用户要求先开IMU记录下场跑几局，WIO当前暂缓；不自动启动设备。

IMU：唯一SDK拥有者同时启Pose/Gyro/Accel，正式比赛及桌面MotionMap默认写t265_raw_imu.csv。保存sdk_run、stream、帧号、SDK capture时间和域、三轴原始值、可用温度；缺温度留空、非有限原始值保留。gyro为rad/s、accel为m/s²，原始传感器轴，未扣重力/未转车体坐标。启动actual_sdk_session记录实际profiles、内参/噪声/偏置参数、IMU到Pose外参和运行编号；读取失败如实记calibration_read_error。没有WIO、控制注入或新增数据年龄检查，日志失败沿原异步机制记丢失/失败，不停控制。

复用原AsyncCsvLog后台写盘。离线2000条模拟记录格式化/入队约9.42微秒/条、无丢失，平均74字节；按262条/秒约0.25%单核格式化/入队、约12MB/10分钟，实际字段长度可能更多。该数不含真实SDK取帧及联合运行负载，下一局核对真实采样数、丢日志及Pose输出。

必要检查：独立完整快照构建通过；CSV格式、SDK重新运行编号、缺温度、NaN保留及写失败行为检查通过；原正式入口和桌面模拟入口通过。同快照程序及配套库已发布，旧文件已备份，清单见reports/imu_map_display_20261116/imu_deployment.json。原未提交实验保留且默认关闭。
