# T265 环境扫描与定位地图工具

这是一个独立的 RDK X5 调试程序，用于：

1. 让 T265 在固定比赛场地慢速走一圈，建立并导出 T265 的 localization map；
2. 下次启动前导入 map，观察是否产生 relocalization；
3. 同时显示 T265 原始 tracking-origin、杠杆臂修正后的三轮旋转中心、F407 三轮里程计三条轨迹；
4. 保存每帧 T265、编码器、F407 状态、通知和运动命令日志。

程序链接仓库现有的 librealsense 2.50.0 和定位投影代码，但不会修改主任务状态机，也不会修改 F407 仓库。

## 编译和启动

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/t265_map
cmake -S . -B build \
  -DREALSENSE_ROOT=/home/sunrise/文档/ChatGPT/T265 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./run_t265_map.sh
```

首次编译后安装桌面入口：

```bash
./install_desktop_launcher.sh
```

桌面入口会带 `--enable-motion` 启动，监听 `/dev/ttyS1` 并启用运动测试按钮；启动本身不会让车移动，只有点击运动按钮才发送命令。若 F407 新接口尚未实现，可直接用命令行的默认观测模式：

```bash
./run_t265_map.sh --no-uart
```

## 建图流程

1. 使用最终比赛安装方式固定 T265，保持镜头朝上、线缆和支架不再改变。
2. 启动桌面程序，确认左侧轨迹的 tracker confidence 稳定在 2 或 3。
3. 在场地中慢速经过四周、四个角、中心和几条对角线；到过的区域再从不同方向回访一次。尽量避免人员、箱子和其他会移动的物体作为主要特征。
4. 可在四个已知位置按 `1`、`2`、`3`、`4` 保存 T265 static node。只有 tracker confidence=3 时才允许保存。
5. 点击 `EXPORT MAP` 或按 `E`。退出时程序还会再导出一次，避免只保存到中途。

每次运行默认创建一个独立目录：

```text
rescue_map/runtime/history/t265_map_builder/YYYYMMDD_HHMMSS/
├── metadata.json
├── t265_localization.raw
├── pose.csv
├── encoder.csv
├── f407_status.csv
└── events.jsonl
```

`metadata.json` 固定记录本车已经校准的 T265 信息：

```json
{
  "t265_serial": "944222110255",
  "t265_firmware": "0.2.0.951",
  "librealsense_version": "2.50.0",
  "camera_offset_forward_m": -0.0296,
  "camera_offset_left_m": -0.0301,
  "installation_axes": {
    "robot_forward": "+X",
    "robot_left": "-Y",
    "robot_up": "-Z"
  }
}
```

偏置单位是米，参考点是 T265 两个双目成像器中心相对三轮运动学旋转中心，不是 T265 外壳中心，也不是推板、拨板或轮子半径。轨迹中的 `center` 由现有定位代码按以下杠杆臂定义修正，而且只修正一次：

```text
r0 = (-0.0296, -0.0301) m
r_current = R(relative_yaw) * r0
robot_center_delta = tracking_origin_delta - (r_current - r0)
```

## 导入和验证

先把确认可用的 `t265_localization.raw` 作为只读基准保存，再执行：

```bash
./run_t265_map.sh \
  --load-map /path/to/t265_localization.raw \
  --save-map /path/to/verification-copy.raw
```

导入发生在 `pipeline.start()` 之前。验证模式只有收到 T265 的 `POSE_RELOCALIZATION` 通知后，才开始建立本次相对轨迹；如果在 `--relocalization-timeout` 内没有通知，界面会明确显示超时，而不会把未重定位的坐标假装成已导入地图坐标。

这个 raw 文件是 T265 内部的视觉 localization map，不是比赛场地 CAD 图，也不是可直接编辑的占据栅格图。它没有“扫描百分之百”的状态；应以覆盖路线、回访一致性和冷启动 relocalization 成功作为验收标准。

## 界面操作

- `TURN +90 / +180 / +360`：原地逆时针旋转测试；
- `MOVE FWD 1M`：车体 forward 正方向行驶 1 m；
- `MOVE LEFT 1M`：车体 left 正方向横移 1 m；
- `OUT 1M + BACK`：直行 1 m、原地转 180°、直行 1 m 返回起点；
- `RESET ODOM`：要求 F407 清零自己的轮式里程计基线；
- `STOP / E-STOP`：发送停止命令并取消当前测试序列；
- 键盘 `W/S`：相对车头前进/后退 500 mm；`A/D`：相对车体左/右横移 500 mm；
- 键盘左/右方向键：原地左/右转 10°；`X` 或空格：停止；
- `1..4`：保存 `scan_anchor_1..4`；
- `E`：导出地图，`R`：只重置显示和本次比较参考，`F`：全屏，`+/-`：缩放，`q/Esc`：退出。

运动按钮是一次只发送一条原子指令，并等待 F407 回报 `DONE` 后才发送多步返航序列的下一步。窗口关闭、状态超时或 F407 回报错误时，程序会尝试发送 `STOP`。

## RDK X5 → F407 扫描运动接口（待下位机实现）

当前最新 F407 主线仍使用 `0x15` ODOM、`0x17` STM 状态和 `0x18` 任务命令。本程序不复用这些含义，而是提出独立的扫描调试接口：

- `TYPE=0x19`：RDK X5 → F407 扫描运动命令；
- `TYPE=0x1A`：F407 → RDK X5 扫描运动状态。

两者均使用现有固定 15 字节格式：

```text
A3 B3 TYPE SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

CRC 覆盖 `TYPE..P7` 共 10 字节，CRC-16/Modbus，低字节先发。所有多字节参数使用大端。

### `0x19` 命令载荷

```text
P0       command
P1       flags
P2..P3   signed arg1
P4..P5   signed arg2
P6..P7   unsigned speed
```

| command | 含义 |
|---:|---|
| `0` | `STOP`，立即停止并回报当前命令已停止 |
| `1` | `HOLD`，保持停止 |
| `2` | `TURN_REL`，`arg1` 为 0.1° 有符号相对角度，正值逆时针/左转；`speed` 为 0.1°/s |
| `3` | `MOVE_BODY`，`arg1` 为 forward mm，`arg2` 为 left mm，允许有符号；`speed` 为 mm/s |
| `4` | `MOVE_FIELD`，`arg1` 为场地 +X mm，`arg2` 为场地 +Y mm；`speed` 为 mm/s |
| `5` | `RESET_ODOM`，建立新的 F407 编码器基线，不移动 |

flags：

```text
bit0 VALID
bit1 KEEP_HEADING       MOVE_BODY 时使用 F407 IMU 航向保持
bit2 FIELD_FRAME        仅 MOVE_FIELD 使用场地坐标
bit3 ACK_REQUIRED       要求 0x1A 回报该命令序号
bit4 CLEAR_FAULT        RESET_ODOM 时清除可清除故障
```

`0x1A` 载荷为：

```text
P0       状态帧序号对应的已确认命令 SEQ
P1       state: 0 IDLE, 1 RUNNING, 2 DONE, 3 ERROR, 4 STOPPED
P2       fault code
P3       当前/最近 command
P4..P5   progress：移动为有符号 mm，旋转为有符号 0.1°
P6..P7   当前 IMU 航向，0..35999，单位 0.01°
```

F407 侧实现时应把 `0x19` 接入现有唯一字节流解析器，不能另开第二个定界状态机；运动应复用三轮运动学和 IMU 航向保持，不应由上位机直接发送 PWM。需要保留独立安全看门狗：命令超时、CRC 错误、T265/程序退出或状态长期不新鲜时停车。只有在命令完成后回报 `DONE`，上位机才会进入多步测试的下一步。

这个接口只写入本上位机仓库的协议定义和发送端，F407 仍由电控侧另行实现；下位机接口未实现时不要点击桌面中的运动按钮：

```bash
./run_t265_map.sh --enable-motion
```

启用后，若状态一直没有回报，程序会在约 4 秒后自动发送 `STOP` 并取消当前测试。
