# F407 手柄控制模式

这是上位机侧独立的飞智/兼容 XInput 手柄控制工具。它不修改现有视觉、定位或 `t265_map` 代码，真实 `/dev/ttyS1` 只由本程序打开。

## 安装和启动

```bash
sudo apt install python3-evdev python3-serial python3-tk
sudo usermod -aG input,dialout "$USER"
# 重新登录后执行
./gamepad_teleop/install_desktop_launcher.sh
```

桌面点击 `F407-手柄控制模式` 后会打开按键说明窗口并自动连接手柄和 F407。也可以从终端运行：

```bash
./gamepad_teleop/run_gamepad_control.sh
```

如果自动识别了错误的输入设备，可以显式指定：

```bash
./gamepad_teleop/run_gamepad_control.sh --device /dev/input/eventN
```

## 手柄功能

必须持续按住 RB 才会使能运动和舵机；程序启动时还要求摇杆先回中。LB 将速度限制为 35%。

| 输入 | 功能 |
|---|---|
| 右摇杆上/下 | 前进/后退 |
| 右摇杆左/右 | 车体左移/右移 |
| 左摇杆左/右 | 原地逆时针/顺时针旋转 |
| 左摇杆上/下 | 摄像头舵机连续控制 |
| 十字键左/右 | 两侧夹爪闭合/张开 |
| 十字键上/下 | 大舵机抬起/放下 |
| RB | 总使能，松开立即停车 |
| LB | 精细速度 |
| START 或 M1 | 启动 T265 建图界面 |
| SELECT 或 M2 | 退出 T265 建图界面 |
| A/B/X/Y | 预留扩展 |

## T265 PTY 转发

按 START/M1 后，本程序继续独占真实 UART，并创建伪终端，把 F407 返回的 ODOM/状态字节镜像给 `t265_map`。`t265_map` 写回伪终端的运动命令会被丢弃，所以建图界面不会与手柄争抢运动控制；手柄仍可连续遥控小车。按 SELECT/M2 或界面按钮会向 T265 进程发送 SIGINT 并关闭 PTY。

默认启动命令为：

```text
t265_map/run_t265_map.sh --enable-motion --uart {pty}
```

如需覆盖命令，可设置 `GAMEPAD_T265_COMMAND`，其中 `{pty}` 会替换为实际伪串口路径。

## 协议和安全

每秒发送50帧 `A3 B3 19 ... C3` 连续遥控帧，字段布局与 F407 `Gamepad.c` 对齐。正常退出会发送三帧未使能零输入和一帧 STOP；手柄拔出、程序卡顿或 UART 断开时，F407 自己的 150 ms 看门狗负责停车。

第一次必须架空三个车轮，逐轴核对正负方向，再低速落地测试。
