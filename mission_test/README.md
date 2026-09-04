# 普通物资抓取与安全区投送测试

本测试项目默认使用X5 BPU YOLO物资识别、`rescue_map`选择结果以及T265+三轮编码器融合位姿，
验证以下闭环。当前六类别YOLOv8s模型同时识别四类物资和红/蓝安全区，且只接受置信度不低于
0.50的结果；正常任务流程不再自动调用传统视觉。

```text
搜索普通物资
  → STM32按1280×1024目标坐标居中并靠近
  → STM32下转摄像头并置CLAW_VISIBLE
  → RDK连续3帧确认物资仍出现在画面任意位置
  → 直线导航到本方安全区前置点
  → 调整为正对安全区入口
  → 直行进入安全区
  → 视觉连续3帧确认普通物资中心位于本方安全区框内
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

## 安全区导航

- 红方前置点：`(0,+950 mm)`，正对方向`90°`；安全区目标中心`(0,+1320 mm)`。
- 蓝方前置点：`(0,-950 mm)`，正对方向`270°`；安全区目标中心`(0,-1320 mm)`。
- 小车距前置点250 mm以内后进入对正阶段；航向误差不超过8°后才允许直行进入。
- `TYPE=0x18`给出场地绝对目标X/Y和最终航向；STM32结合持续接收的`TYPE=0x16`融合位姿计算控制量。

STM32直线导航时推荐：

```text
bearing = atan2(target_y - pose_y, target_x - pose_x)
distance = hypot(target_x - pose_x, target_y - pose_y)
```

先原地调整到`bearing`，再锁定该方向直行；到前置点后，使用命令给出的`heading_cdeg`对正安全区。任何视觉、任务命令或融合位姿看门狗超时都应停车。

## 安全区视觉完成条件

进入安全区阶段只检测`green_supply`和本方`safe_red/safe_blue`。普通物资检测框中心连续3帧位于安全区检测框内部（保留3%边缘裕量）才发送`TASK_COMPLETE`。目前没有根据物资面积发送停车信号。

## 测试

```bash
PYTHONPATH=../vision:. python3 tests/test_state_machine.py
python3 ../vision/tests/test_vision_protocol.py
ctest --test-dir ../localization/build --output-on-failure
```
