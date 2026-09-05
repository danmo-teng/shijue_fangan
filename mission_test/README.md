# 普通物资抓取与安全区投送测试

本测试项目默认使用X5 BPU YOLO物资识别、`rescue_map`选择结果以及T265+三轮编码器融合位姿，
验证以下闭环。当前六类别YOLOv8s模型虽然仍能输出红/蓝安全区类别，但任务状态机只使用
`green_supply`；安全区识别结果不再参与投送完成判断。YOLO只接受置信度不低于0.50的结果，
正常任务流程不再自动调用传统视觉。

```text
搜索普通物资
  → STM32按1280×1024目标坐标居中并靠近
  → STM32下转摄像头并置CLAW_VISIBLE
  → RDK连续3帧确认物资仍出现在画面任意位置
  → RDK重复发送抓取命令，等待STM32确认GRIPPER_CLOSED
  → 直线导航到本方安全区前置点
  → 调整为正对安全区入口
  → 直行进入安全区
  → 地图车体圆与本方安全区相交，且融合位置持续稳定不动
  → TASK_COMPLETE
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

## 抓取确认参数

STM32确认摄像头已经下压后，只要普通物资仍出现在画面任意位置，连续确认3帧即发送抓取确认。
不再限制目标的X坐标、Y坐标或底部区域。确认帧数可通过命令行调整：

```bash
./run_mission_test.sh --confirm-frames 4
```

只有STM32的`TYPE=0x17`状态帧中`CLAW_VISIBLE=1`且状态帧不超过250 ms时，画面内物资确认才会累计；状态失效或物资消失都会把连续确认计数清零。

确认目标后进入`GRABBING`状态，任务程序按实际视觉循环频率（通常20～50 Hz）重复发送
`GRAB_CONFIRMED`。只有新鲜状态帧置`GRIPPER_CLOSED=1`后才开始导航；默认3秒超时后发送
`STOP`并进入`FAULT`。可用`--grab-timeout`调整超时，但不建议取消握手。

F407应保持2秒物理合爪窗口，并且只在左右两只爪子的动作都真正完成后置
`GRIPPER_CLOSED=1`，随后每50 ms状态帧持续携带该位。重复抓取帧只更新ACK，不能重新计时或
再次启动舵机；收到未合爪的NAV只能停车等待，不能执行旧的强制抓取兜底。

## 安全区导航与物资区偏置

- 红方普通物资区位于左半区，入口边界目标：`(-150,+1200 mm)`，正对方向`90°`；区内目标为`(-150,+1320 mm)`。
- 蓝方普通物资区位于右半区，入口边界目标：`(+150,-1200 mm)`，正对方向`270°`；区内目标为`(+150,-1320 mm)`。
- `±150 mm`是300 mm宽普通物资半区的中心，车体圆到中央分隔线和外侧边界均留有约30 mm余量，避免把普通物资送进伤员区，也不会过度贴近外沿。
- 地图按半径120 mm绘制小车圆。只有该圆与本方安全区矩形发生接触或重叠，才进入对正阶段；仅凭距离阈值或视觉看到安全区不能提前切换。
- `TYPE=0x18`给出场地绝对目标X/Y和最终航向；STM32结合持续接收的`TYPE=0x16`融合位姿计算控制量。

STM32直线导航时推荐：

```text
bearing = atan2(target_y - pose_y, target_x - pose_x)
distance = hypot(target_x - pose_x, target_y - pose_y)
```

`NAVIGATE_WAYPOINT`由RDK计算并发送`bearing`，同时置`USE_FINAL_HEADING`，STM32先调整到该航向再锁定方向直行；到前置点后，`ALIGN_SAFE_ZONE`和`ENTER_SAFE_ZONE`改用红方90°或蓝方270°。任何视觉、任务命令或融合位姿看门狗超时都应停车。

返航/导航方向帧0～250 ms内使用正常速度，250～1000 ms限速250 mm/s，超过1000 ms临时停车
但保留NAV状态。临时STOP和融合位姿失效同样只停车等待；新NAV或有效位姿恢复后继续。
只有ABORT是永久停止。

发送`TASK_COMPLETE`必须同时满足：融合位姿有效、地图小车圆仍与本方安全区相交、融合位置在
半径15 mm范围内持续稳定0.8秒。车体未接触安全区或仍在移动时，RDK持续发送
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
  --material-target-x-mm 150 \
  --delivery-stationary-seconds 0.8 \
  --delivery-stationary-tolerance-mm 15
```

如果停车时定位抖动超过15 mm，应先检查融合定位和车体半径标定；不建议直接把容差大幅放宽。

## 测试

```bash
PYTHONPATH=../vision:. python3 tests/test_state_machine.py
python3 ../vision/tests/test_vision_protocol.py
ctest --test-dir ../localization/build --output-on-failure
```
