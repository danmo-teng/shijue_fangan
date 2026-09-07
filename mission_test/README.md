# 连续物资寻找、抓取与分区投送

本测试项目默认使用X5 BPU YOLO物资识别、`rescue_map`选择结果以及T265+三轮编码器融合位姿，
验证连续搬运闭环。任务状态机使用`green_supply`、`core_black`、`danger_cyan`和
`injured_orange`四类目标；同时识别本方`safe_red`或`safe_blue`安全区，物资框中心位于
安全区框内时被过滤，不会向F407发送该物资坐标。YOLO只接受置信度不低于0.50的结果。

安全区过滤只在本方安全区被视觉检测到时生效。安全区内目标会在窗口中以橙色框显示，并发送空的
`TYPE=0x12`报告覆盖上一帧坐标；如果已经进入APPROACH/GRAB_CHECK，也会退回SEARCH，避免继续抓取。

```text
开局只搜索并投送一次普通物资
  → 识别本方安全区并过滤安全区内物资
  → STM32按1280×1024目标坐标居中并靠近
  → STM32下转摄像头并置CLAW_VISIBLE
  → RDK连续3帧确认物资仍出现在画面任意位置
  → RDK重复发送抓取命令，等待STM32确认GRIPPER_CLOSED
  → RDK发送安全区航向和编码器定距，F407执行
  → 调整为正对安全区入口
  → 直行进入安全区
  → 地图接触或STM32定距完成，锁存已到达
  → CHECK状态下融合位置持续稳定不动
  → TASK_COMPLETE，STM32张爪并后退退出
  → RDK再次发送返中航向和定距，在距中心600 mm处停车
  → 搜索下一件物资/伤员并重复以上流程
```

## 启动顺序

先选择出发区和红蓝方，并启动融合定位/串口转发：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/rescue_map
./run_rescue_map.sh
```

确认地图显示`GOOD`后，在另一个终端运行任务窗口：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/mission_test
./run_mission_test.sh
```

正式出发点固定为1号`(-1350,+1350)`、2号`(+1350,+1350)`、3号`(-1350,-1350)`、
4号`(+1350,-1350) mm`。任务程序在发送配置前检查第一帧有效融合位姿，平面误差超过20 mm
立即报`START_POSE_MISMATCH`并禁止启动。地图径向参数默认212.132 mm，与定位配置
`start_center_m=1.350`一致。

需要回退为原传统视觉物资识别时：

```bash
./run_mission_test.sh --detector traditional --vision-fps 30
```

任务程序不直接打开`/dev/ttyS1`。`localization`是唯一串口所有者：任务程序原子更新
`rescue_map/runtime/uart_command.bin`，独立串口线程校验并转发。任务命令以100 Hz刷新SEQ和CRC，
不再依赖T265取帧循环。独立50 Hz任务规划线程直接读取最新融合位姿和STM32状态，NAV及
RETURN_CENTER不再等待新摄像头帧；配置和SEARCH/APPROACH视觉坐标仍只按新图像结果发送。

融合位姿无效或超过250 ms时，规划线程不更新NAV/RETURN命令；串口心跳最多再维持250 ms旧命令，
随后停止刷新，让下位机看门狗停车。位姿恢复后立即基于当前位置重新计算航向和剩余距离。

识别窗口中的“任务状态切换”在状态变化时打印，动态导航中最多每0.5秒打印一次，并不表示UART只发送一帧。
窗口的`relay seq/age/tx/err`才是实际串口转发状态；正常运行时`age`应远小于250 ms且`tx`
持续增加。

## 首轮普通物资门控与抓取确认

STM32确认摄像头已经下压后，只要普通物资仍出现在画面任意位置，连续确认3帧即发送抓取确认。
不再限制目标的X坐标、Y坐标或底部区域。确认帧数可通过命令行调整：

```bash
./run_mission_test.sh --confirm-frames 4
```

只有STM32的`TYPE=0x17`状态帧中`CLAW_VISIBLE=1`且状态帧不超过250 ms时，画面内物资确认才会累计；状态失效或物资消失都会把连续确认计数清零。

第一次投送完成前，RDK只向F407发送普通物资目标。第一次普通物资确实进入安全区后，才允许
从四类目标中选择画面内面积最大的目标。选中后锁定类别，避免靠近途中跳到另一类目标。

确认目标后进入`GRABBING`状态，任务程序按实际视觉循环频率（通常20～50 Hz）持续发送
`GRAB_CONFIRMED`，直到新鲜状态帧置`GRIPPER_CLOSED=1`后才开始导航，不设置抓取失败倒计时。

F407应保持2秒物理合爪窗口，并且只在左右两只爪子的动作都真正完成后置
`GRIPPER_CLOSED=1`，随后每50 ms状态帧持续携带该位。重复抓取帧只更新ACK，不能重新计时或
再次启动舵机；收到未合爪的NAV只能停车等待，不能执行旧的强制抓取兜底。

## 对应分区中心

- 普通、核心和危险物资送往物资半区正中心：红方`x=-150 mm`，蓝方`x=+150 mm`。
- 伤员送往伤员半区正中心：红方`x=+150 mm`，蓝方`x=-150 mm`。
- 安全区内区为600×300 mm，场内侧内边界`|y|=1200 mm`；高围栏场地侧表面为`|y|=1140 mm`。
- `±150 mm`就是两个300 mm宽半区各自的几何中心，不再叠加额外向左或向右偏置。
- `robot_body_radius=130 mm`只用于车体碰撞圆；`push_plate_offset=105 mm`是车中心到实际贴住高围栏的推板距离；`front_pusher_offset=150 mm`只预留给独立前拨板动作；`zone_center_x=150 mm`始终是分区中心。
- 推板理论贴栏时车中心为`1140-105=1035 mm`。NAV停车点再向场内留7.5 mm余量，即红方`y=+1027.5 mm`、蓝方`y=-1027.5 mm`，不能用`1200-车体半径`代替。
- NAV几何到达同时检查轴向误差≤30 mm、横向误差≤50 mm和车头朝向围栏；也可由新鲜`DISTANCE_DONE`在合理横向/朝向范围内锁存到达。
- `TYPE=0x18`置`DISTANCE_VALID`后，`P2..P3`改为行驶距离毫米，`P4..P5=0`，`P6..P7`为绝对航向；F407用IMU对向并用编码器完成定距。

RDK计算：

```text
bearing = atan2(target_y - pose_y, target_x - pose_x)
distance = hypot(target_x - pose_x, target_y - pose_y)
```

`NAVIGATE_WAYPOINT`持续发送最新`bearing + remaining_distance`，但不下发X/Y位置坐标。里程计存在累计误差时，下一帧会根据T265+编码器融合位置修正航向和剩余距离；到达后由RDK地图锁存到达并直接发送`ENTER_SAFE_ZONE`。下位机新版本已删除`ALIGN_SAFE_ZONE`执行步骤，RDK不再等待`mode=11`。

返航期间方向和目标bearing始终来自T265位姿/航向；编码器只把三轮平移投影到当前目标方向，
作为本段剩余距离补偿，不再用错误的完整二维轮式位姿拉动地图。地图日志中的
`navigation.distance_compensation_m`可直接核对本段编码器累计距离，`navigation.t265_innovation_m`
用于判断T265与加权轮式预测的分歧；仅T265模式不启用该补偿。

航向和距离命令在运动期间持续更新；50 Hz规划生成最新值，100 Hz串口心跳发送最近一次有效规划。
如果运动中任务命令失联，下位机应停车；恢复后只能采用上位机新算出的航向和剩余距离，不能
重新执行失联前缓存的完整距离。

投送完成后，RDK重复发送`TASK_COMPLETE`，直到F407张爪并后退离开围栏。F407进入返中状态后，
RDK持续计算指向原点的航向以及`当前位置到中心距离-0.60 m`，发送`RETURN_CENTER`。小车运动
期间航向和剩余距离随融合位姿更新，到距中心600 mm的位置即恢复`SEARCH`。

NAV阶段只要地图车体圆与安全区相交，或收到新鲜的`mode=10 + GRIPPER_CLOSED + DISTANCE_DONE`，
就锁存本次已经到达并直接发送`ENTER_SAFE_ZONE`。下位机新版本不再执行ALIGN，对齐命令只保留
协议兼容解析；随后进入张爪/CHECK流程。发送`TASK_COMPLETE`必须满足：到达已锁存、融合位姿
有效、新鲜`mode=15`，并且融合位置在半径25 mm范围内持续稳定0.8秒。

## 围栏接触参考与现场调参

完成时围栏接触校正参考使用推板几何：高围栏场地侧表面`|y|=1.140 m`减去推板偏置
`0.105 m`，理论车中心为红方`y=+1.035 m`、蓝方`y=-1.035 m`。程序只用这一有物理依据的Y轴约束，
X和航向保持观测值，并把观测位置、相切参考位置和二者差值原子写入
`rescue_map/runtime/delivery_contact_pose.json`，供后续标定分析。该差值不会直接重置T265/EKF，
以免一次轮胎打滑或非正面接触造成定位跳变。

默认参数可按实测噪声小幅调整：

```bash
./run_mission_test.sh \
  --zone-center-x-mm 150 \
  --delivery-stationary-seconds 0.8 \
  --delivery-stationary-tolerance-mm 25 \
  --center-stop-radius-mm 600
```

25 mm容差只用于判断CHECK阶段是否静止；安全区到达由地图接触或下位机`DISTANCE_DONE`锁存。

## 测试

```bash
PYTHONPATH=../vision:. python3 tests/test_state_machine.py
python3 ../vision/tests/test_vision_protocol.py
ctest --test-dir ../localization/build --output-on-failure
```
