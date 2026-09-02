# 摄像头参数测试

这个目录用于单独观察摄像头参数对画面的影响，不会调用或修改传统视觉识别程序。

## 启动

在板端执行：

```bash
cd ~/RDK_X5/traditional_rescue_vision/camera_parameter_test
python3 camera_parameter_test.py
```

默认测试的是正式程序使用的模式：`MJPG 640x480@350`。

测试更高分辨率：

```bash
python3 camera_parameter_test.py --format MJPG --width 1280 --height 720 --fps 180
python3 camera_parameter_test.py --format MJPG --width 1280 --height 1024 --fps 180
```

测试 YUYV：

```bash
python3 camera_parameter_test.py --format YUYV --width 640 --height 480 --fps 30
```

启动时也可以设置参数：

```bash
python3 camera_parameter_test.py \
  --auto-exposure 1 \
  --exposure 15 \
  --auto-focus 0 \
  --focus 264 \
  --sharpness 4
```

## 窗口按键

| 按键 | 调整内容 |
|---|---|
| `1`~`8` | 选择曝光、焦距、白平衡、锐度、亮度、对比度、饱和度、伽马 |
| `[` / `-` | 减小当前选中的参数 |
| `]` / `=` | 增大当前选中的参数 |
| `e/f/w/k/b/c/v/g` | 直接减小对应参数（兼容旧按键） |
| `p` | 终端打印实际参数和画质指标 |
| `s` | 保存当前原始帧到 `captures/` |
| `h` | 在终端显示按键帮助 |
| `q` / `Esc` | 退出 |

画面中的 `source`、终端中的 `V4L2实际` 是摄像头实际协商出的参数。画面中的 `capture` 是采集线程实际解码帧率，`window` 是窗口刷新帧率；窗口刷新慢不代表摄像头协商成了低帧率。`sharpness` 是中心区域 Laplacian 方差，只用于比较同一场景下不同参数的相对清晰度，不是绝对画质等级。
