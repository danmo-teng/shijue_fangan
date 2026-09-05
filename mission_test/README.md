# 连续物资寻找、抓取与分区投送

本测试项目默认使用X5 BPU YOLO物资识别、`rescue_map`选择结果以及T265+三轮编码器融合位姿，
验证连续搬运闭环。当前六类别YOLOv8s模型虽然仍能输出红/蓝安全区类别，但任务状态机只使用
`green_supply`、`core_black`、`danger_cyan`和`injured_orange`四类目标；安全区识别结果不参与
投送完成判断。YOLO只接受置信度不低于0.50的结果。

```text
开局只搜索并投送一次普通物资
  → STM32按1280×1024目标坐标居中并靠近
  → STM32下转摄像头并置CLAW_VISIBLE
  → RDK连续3帧确认物资仍出现在画面任意位置
  → RDK重复发送抓取命令，等待STM32确认GRIPPER_CLOSED
  → RDK发送安全区航向和编码器定距，F407执行
  → 调整为正对安全区入口
  → 直行进入安全区
  → 地图车体圆与本方安全区相交，且融合位置持续稳定不动
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

需要回退为原传统视觉物资识别时：

```bash
./run_mission_test.sh --detector traditional --vision-fps 30
```

任务程序不直接打开`/dev/ttyS1`。`localization`是唯一串口所有者：任务程序原子更新`rescue_map/runtime/uart_command.bin`，定位程序校验帧头、TYPE、CRC和长度后转发，因而不会和编码器/T265进程争抢串口。

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
- 入口边界为红方`y=+1200 mm`、蓝方`y=-1200 mm`；区内撞送目标为红方`y=+1320 mm`、蓝方`y=-1320 mm`。
- `±150 mm`就是两个300 mm宽半区各自的几何中心，不再叠加额外向左或向右偏置。
- 地图按半径120 mm绘制小车圆。只有该圆与本方安全区矩形发生接触或重叠，才进入对正阶段；仅凭距离阈值或视觉看到安全区不能提前切换。
- 抓取闭合时，RDK用当时的融合位置计算到对应分区入口相切点的航向和直线距离，并锁存发送。
- `TYPE=0x18`置`DISTANCE_VALID`后，`P2..P3`改为行驶距离毫米，`P4..P5=0`，`P6..P7`为绝对航向；F407用IMU对向并用编码器完成定距。

RDK计算：

```text
bearing = atan2(target_y - pose_y, target_x - pose_x)
distance = hypot(target_x - pose_x, target_y - pose_y)
```

`NAVIGATE_WAYPOINT`只发送上述锁存的`bearing + distance`，不再依赖下位机持续接收实时位置。STM32先调整到航向，再用编码器定距直行；到达后由RDK地图判断切换`ALIGN_SAFE_ZONE`和`ENTER_SAFE_ZONE`。

航向和距离命令在运动期间仍会重复发送以保证通信可靠，但内容保持不变，不会逐帧更新位置。
如果定距运动中任务命令长时间失联，下位机应停车并进入故障，不能把完整原距离重新执行一遍。

投送完成后，RDK重复发送`TASK_COMPLETE`，直到F407张爪并后退离开围栏。F407进入返中状态时，
RDK读取一次当前融合位置，计算指向原点的航向以及`当前位置到中心距离-0.60 m`，发送
`RETURN_CENTER`定距命令。F407到距中心600 mm的位置即恢复`SEARCH`，没必要驶到原点。

发送`TASK_COMPLETE`必须同时满足：融合位姿有效、地图小车圆仍与本方安全区相交、融合位置在
半径15 mm范围内持续稳定0.8秒，并且F407已经进入前冲/撞送阶段。车体未接触安全区、仍在移动
或夹爪尚未张开时，RDK持续发送
`ENTER_SAFE_ZONE`，不会提前发送`TASK_COMPLETE`。

## 围栏接触参考与现场调参

车体半径为120 mm，安全区场内侧边界为`y=±1.20 m`，因此车体前缘刚好相切时的理论圆心为
红方`y=+1.08 m`、蓝方`y=-1.08 m`。完成时程序只用这一有物理依据的Y轴约束修正参考位置，
X和航向保持观测值，并把观测位置、相切参考位置和二者差值原子写入
`rescue_map/runtime/delivery_contact_pose.json`，供后续标定分析。该差值不会直接重置T265/EKF，
以免一次轮胎打滑或非正面接触造成定位跳变。

默认参数可按实测噪声小幅调整：

```bash
./run_mission_test.sh \
  --zone-center-x-mm 150 \
  --delivery-stationary-seconds 0.8 \
  --delivery-stationary-tolerance-mm 15 \
  --center-stop-radius-mm 600
```

如果停车时定位抖动超过15 mm，应先检查融合定位和车体半径标定；不建议直接把容差大幅放宽。

## 测试

```bash
PYTHONPATH=../vision:. python3 tests/test_state_machine.py
python3 ../vision/tests/test_vision_protocol.py
ctest --test-dir ../localization/build --output-on-failure
```
