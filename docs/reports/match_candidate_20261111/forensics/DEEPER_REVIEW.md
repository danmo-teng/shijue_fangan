# 定向深化与候选审查

## 板端Location重算

源为设备自报提交e85c738对应仓库的Location.c/app_config.h；dirty1下不保证与烧录字节一致。F405配置：轮径70mm、编码器每圈20.409×13×4，计数0.2072157888mm；raw三delta都先乘−1，forward再乘−1，left采用F405+1。中点主yaw旋转平移，float32累计path/XY，±30000mdeg单步截断保留。按MCU gyro yaw_receive_us取wheel.t1_us前最近样本。复算实测type7车体系速度与其已发布值的最大误差<0.0006mm/s，未触发yaw步长截断。

重建原点是下位±1350mm与135°，不是上位±1321mm及其雷达地图。参考起时取首次0x17 mode1反馈附近的wheel.t0，实际Location_Reset发生在该状态反馈之前，最多约一个60ms状态周期的不确定；不是板端内部实际读回，不能当绝对标定。所有JSON显式标为approximate_source_reconstruction_not_actual_Location_readback。

第二轮绿蓝扫障：CLEAR起309.599的重算(145.9,−1688.7)mm；trace开始314.680(156.1,−1722.6)；抓后318.899(465.0,−1710.2)；卡住末348.731(168.1,−1716.1)。抓后相对净位移约297.1mm（同段上位控制位姿净移126.5mm）；末距CLEAR起约35mm，接近final30mm，但不能据此猜consume_to/trace剩余/cross。

第一轮489.43→520.70，板端重算相对净移279mm、XY跨度60×277mm；不能把上位T265/雷达控制XY约0.75×0.67m跨度当板端trace记录。

派生：*_lower_location_reconstructed.csv/json。源的未知启动采样偏差及轮滑都保留，不称地面真值。

## 上位HOLD/SEQ是否打断

- 第一轮0→7.8的130个mode1及第二轮0→30.423的500个mode1反馈，acceptedcmd/task/action/status/ack始终全部0。第二轮10→30.4的340个样本也全0，未见本次开局接受HOLD。
- actual Task.c的legacy HOLD末分支确实会对START调用Motor_Stop（不能只看Task_Process下方已豁免START的循环）。因此是源码潜在问题，但不能冒认此次停20s主因。
- mode39在legacy HOLD和运行HOLD停机两处都明确豁免，只有ACK/heartbeat；同action重复有限动作只ACK不重启。抓后39持续acceptedcmd2和同action，不能指称8位SEQ换号将机械状态重置。
- mode42接到HOLD则明确清sweep_pickup_target_active，回SEARCH并清tracking。因此几何误排除→上位HOLD→板端再搜索有明确链路。
- wheel_feedback_requests.bin只有42字节的能力/订阅请求，不是完整任务TX录包。事件Replay有原始最后发送payload/序号/时间但仅抽样，不能伪造完整mission TX时间线。

## 有意义的候选漏项/边界

1. **跨39停车/转身的旧目标像素邻域**：只改脚点区域仍不够。原4365/4368蓝322框代入完整safe_sweep_target，都拒绝4502新335。state候选已补已知39→42非找回阶段重新绑定，保留class和原货排除；完整实际边界检查由stateagent完成。
2. **inside_safe_zone不是独立真值**：runner以bbox面积覆盖>80%形成，且safe_zone_filter_blocked仍传入safe_bbox。14144/pitch117/filter_blocked=true的橙1160脚点y205在框外（safe bottom202），覆盖97.5%仍inside=True；候选脚点函数先信inside会继续拒绝。8对象帧见blocked_view_inside_classification.json。这不授权删除真正区内货排除，需要核实际图/同视角来源；已转cargo/state审查。
3. **真模型全漏框**：10450依然有黑三棱锥而raw detections无框。ROI扩展只修有框被筛掉；不应靠上一帧预测框伪造新审核或说全部识别问题已好。需原入口实际黑侧缘图再验，不能把此项隐去。
4. **39低速执行与trace几何要分别验证**：第二轮最后近5s轮数[0,9,0]，几乎真静止；负remaining可能继续后退是源码可证明边界，但现有日志未记录内部目标，不能证明本次都是它。若候选只停掉继续退而仍给过小速度/错误方向，原卡住可能还在。需要复用真实wheel/gyro的定向演算及下位实际最小可执行速度，不加新握手/巡检。
5. **重绑同类不等于物体身份绝对正确**：39阶段放下的原货发生新ID时，单靠old original_ids未必能排掉。当前候选至少保同类、走廊、真区内排除，已知同类多物由中心排序；这仍是图像关联方案，不能承诺永不抓到同类原货，尤其大角度搜索时。物理侧放/已知航向设计要与下位同读，不能用任意新超时遮盖。
