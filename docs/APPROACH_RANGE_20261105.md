# 正式视觉五区定距表（2026-11-05）

本轮基于RDK `ea86bfb`增量录入用户提供的距离。正式入口启动时读取
`vision/config/approach_range.json`，不使用单应矩阵推导这两档距离。

## 实际采用数据

图像1280×1024，取检测框中心Y；B1在最上方，区间左闭右开。
分界来自本机测距程序已保存的 `vision/captures/range_calibration/layouts/{105,120}deg.json`；
原布局、截图和用户资料未修改。

| 区 | 105° Y范围 | 105°距离 | 120° Y范围 | 120°距离 |
|---|---|---|---|---|
| B1 | [0,150) | 400mm（暂用） | [0,73) | 380mm |
| B2 | [150,291) | 400mm（暂用） | [73,217) | 300mm |
| B3 | [291,432) | 370mm | [217,381) | 150mm |
| B4 | [432,603) | 160mm | [381,580) | 50mm |
| B5 | [603,1024) | 40mm | [580,1024) | 20mm |

按用户要求把cm直接转换mm作为现有 `advance_mm` 定距接近行程，不额外扣除或叠加爪偏移、补进量。
距离表的既有语义是车体向夹爪入口接近所需行程，不是相机斜距；本轮未独立测量零点或精度。
105°前两区过远，用户要求暂用40cm；配置保存每区来源及measured标记，整个105°表不标为全实测。
解析器接受显式 `user_approved=true` 的暂用表并返回 `range_user_approved_provisional`；
实测区沿用 `range_measured`，旧的全实测表保持兼容。状态提示改称“分区距离表”。

重新启动正式比赛入口即可加载，不需要重编译C++定位程序。
之后在测距画面拖线、保存布局，不会自动更改本次正式距离表。

## 下位机适配结论

只读核对F405远端main `078836078f2aa08e8d8623aeecf2fc6e1a43015e`，本次分区/距离不需要下位机修改。
不代表实机已烧录该提交，也不替其他遥测迁移待办作完成验收。

- 上位机继续使用0x18 COMMAND=9、APPROACH_RANGE和DISTANCE_VALID，距离mm、俯仰10500/12000、aux=0；原0x1D任务/动作配对不变。
- `Main/Src/Task.c` 5423起将target_x_mm存入approach_range_mm；6090起允许0..2500mm并匹配角度；2391起通过`Motor_MoveDistanceLinear(approach_range_mm * 0.001f, ...)`执行。
- `Main/Inc/app_config.h`现有距离容差3mm，定距最大2500mm；速度参数仍为500mm/s上限、200mm/s末速、250mm减速段。本次20..400mm可传入；源码容差不是实车精度保证，短行程实际制动效果未测试。
- 下位机无须知道B1..B5及像素分界。未增加协议、握手、新鲜度检测或等待，也未改既有状态机。

源码固定链接：

```text
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/078836078f2aa08e8d8623aeecf2fc6e1a43015e/Main/Src/Task.c#L2391
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/078836078f2aa08e8d8623aeecf2fc6e1a43015e/Main/Inc/app_config.h#L457
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/078836078f2aa08e8d8623aeecf2fc6e1a43015e/Main/Src/motor.c#L2019
```

## 必要验证

- `python3 -m pytest -q competition_rescue/tests/test_runner.py::test_range_context_and_calibration`：1 passed。
  覆盖两角度10区首点/中点/末点、1024越界、暂用来源与旧表兼容，以及52/53定距动作配对/保持。
- `git diff --check`：通过。
- 未启动相机/UART/电机，未修改下位机、定位、原始标定布局或日志。定距效果、105°远区实际距离仍待实车确认。
