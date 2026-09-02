# RDK X5 智能救援传统视觉系统

本项目使用颜色、轮廓、几何尺寸和多帧跟踪识别：

- `green_supply`：绿色普通物资；
- `danger_cyan`：浅蓝危险目标，使用更宽松阈值和更快确认；
- `injured_orange`：橙色伤员；
- `core_black`：黑色核心物资，额外检查局部对比度及多边形特征；
- `safe_red`、`safe_blue`：红/蓝安全区；
- `entrance_purple`：紫色安全区入口。

处理链路：

```text
MJPEG 1280x1024@180
  -> 压缩帧单帧泄漏队列（只保留最新帧）
  -> RDK X5 JPU按60Hz解码JPEG为NV12
  -> NV12转BGR
  -> 最新BGR帧
  -> HSV/Lab分割
  -> 候选轮廓
  -> 颜色/形状/尺寸评分
  -> 地面单应坐标
  -> 多帧确认
```

默认阈值只是安全起点，必须用实际比赛物资、相机和照明重新标定。

## 文件说明

```text
run_editor.py                 可视化阈值编辑器
run_detector.py               比赛用无界面识别程序
calibrate_ground.py           图像到地面毫米坐标标定
config/rescue_vision.json     全部现场参数
config/homography.txt         标定后生成
runtime_result.json           运行时识别结果
rescue_vision/                识别核心模块
tests/smoke_test.py           合成图像测试
```

## 1. 部署

把整个目录复制到板端，例如：

```bash
scp -r traditional_rescue_vision root@RDK_IP:~/RDK_X5/
ssh root@RDK_IP
cd ~/RDK_X5/traditional_rescue_vision
bash deploy_rdkx5.sh
```

如果板端已有OpenCV、GStreamer和`python3-gi`，可以跳过安装，只运行：

```bash
python3 -c 'import cv2,numpy,gi; gi.require_version("Gst","1.0"); from gi.repository import Gst; print("OK")'
```

## 2. 固定摄像头参数

```bash
cd ~/RDK_X5/traditional_rescue_vision
bash set_camera_controls.sh /dev/video0
```

通过环境变量临时修改：

```bash
EXPOSURE=15 WHITE_BALANCE=4600 FOCUS=220 bash set_camera_controls.sh /dev/video0
```

每次改变曝光、白平衡、焦距或补光后，都应重新检查阈值。

## 3. 运行阈值编辑器

### 推荐：SSH Web编辑器

Web编辑器不使用RDK桌面的X11窗口，界面直接显示在调试电脑浏览器中。先在电脑新建带端口转发的SSH连接：

```bash
ssh -L 8080:127.0.0.1:8080 root@RDK_IP
```

然后在板端运行：

```bash
cd /home/sunrise/RDK_X5/traditional_rescue_vision
python3 web_editor.py --device /dev/video0 --decoder jpu --decode-fps 60
```

在电脑浏览器打开：

```text
http://127.0.0.1:8080
```

如果SSH已经连接，可以在电脑另开终端只建立隧道：

```bash
ssh -N -L 8080:127.0.0.1:8080 root@RDK_IP
```

Web编辑器提供：

- 宽滑条和可直接输入的数值框；
- 当前组号、显示名称、内部配置键；
- 组号和显示名称均可修改（内部配置键保持稳定，避免破坏导航配置）；
- 鼠标框选目标后自动估算HSV、Lab和形状范围；
- 每个类别可建立多组参考阈值，例如“正视、侧视、斜视、远距离”；新增参考会复制当前参数，再对新角度框选自动取值；
- 调试界面只运行当前选中的参考，便于看清该参考是否有效；正式运行时同类别所有参考按“任一通过”合并，并对重复命中去重；
- 明确显示颜色、形状、毫米尺寸和多帧规则是否生效；
- 黑色核心物资专用的局部对比度和多边形顶点参数；
- 框选典型物资后给出可直接写入的具体曝光值；180 FPS下根据帧周期自动限制在4～50（约0.4～5.0ms）；
- 框住哑光灰卡后扫描2800～6500 K，选择B/G/R最均衡的具体色温并锁定；
- 框住600～800 mm处带边缘的物资后，对0～1023焦距粗扫、精扫并锁定最清晰值；
- 白黑掩膜只显示通过面积、形状和评分的最终有效目标，面积不合格的小色块保持黑色；
- 识别采用640x512全图粗筛，再在1280x1024原图的候选ROI内精判；既减少全帧运算，又保留远处目标的原始像素；
- 网页预览直接读取摄像头最新BGR帧，与识别线程相互独立；显示前缩小为640x512、JPEG质量72、默认10 FPS；
- 浏览器等待上一张JPEG加载完成后再请求下一张，不会让SSH下的图片请求相互覆盖；
- 白黑掩膜使用最近邻缩放，边缘不会因为预览缩放产生灰色像素。

框选自动取值只用于生成起点，必须用目标的不同距离、朝向、阴影和赛场光照复核。参数点击“应用”后只在内存生效，点击“保存全部配置”才写入JSON。

多参考建议每类保留2～4组真正互补的参数。可以按“正视、侧视、斜视/顶视、远距离”采样；不要把每一帧都建成一个参考，因为每增加一组参考都会增加一次该类别的分割和轮廓计算。各参考的面积和形状范围应分别收紧，不要只用多个非常宽的颜色范围。

普通物资参数保存后，先按`Ctrl-C`关闭Web编辑器释放摄像头，再运行无Web的本地显示与UART程序：

```bash
export DISPLAY=:0
python3 run_normal_supply_uart.py --device /dev/video0 \
  --decoder jpu --decode-fps 60 --window-mode fullscreen --uart /dev/ttyS1
```

该程序直接复用Web编辑器保存的完整识别配置和原有检测算法，只启用`green_supply`，不启动HTTP服务。全屏显示使用等比例完整缩小，不裁剪摄像头画面；当前1024×768桌面会把1280×1024图像缩成960×768并在左右补黑边。按`F`切换全屏/窗口，按`Q`或`Esc`退出。

### 备用：RDK桌面OpenCV编辑器

本地显示器：

```bash
export DISPLAY=:0
python3 run_editor.py --device /dev/video0
```

SSH使用时必须启用X11转发，例如`ssh -X`，或在MobaXterm中启用X Server。

快捷键：

| 按键 | 功能 |
|---|---|
| `[` / `]` | 上一个/下一个类别 |
| `1` / `2` / `3` | HSV、Lab、最终融合掩膜 |
| `F` | 切换 AND、OR、仅HSV、仅Lab |
| 空格 | 冻结/恢复画面 |
| 鼠标左键 | 读取像素BGR、HSV、Lab |
| `S` | 保存参数到JSON |
| `L` | 从JSON重新加载 |
| `C` | 保存当前现场原图 |
| `Q` / `Esc` | 退出 |

界面左上为识别结果，右上为白目标/黑背景掩膜，左下为掩膜后的目标图，右下为运行状态。

推荐每类调试顺序：

1. 冻结包含目标的画面；
2. 点击目标中心与明暗边缘，记录HSV/Lab；
3. 先调HSV，再调Lab；
4. 根据误检情况选择AND或OR；
5. 调开闭运算，确保目标连通但相邻目标不粘连；
6. 调`Area min/max`排除噪声和大背景；
7. 在不同距离、姿态和光照下重复验证；
8. 按`S`保存。

编辑器只识别当前选中的类别，因此能把算力用于实时调参，不代表正式运行只识别一种目标。

## 4. 地面坐标标定

配置中的地面坐标以机器人为原点：X向右为正，Y向前为正，单位毫米。默认点为3列×3行：

```text
X = -300, 0, 300 mm
Y =  400, 800, 1200 mm
```

把九个标志点准确放到这些位置，然后：

```bash
python3 calibrate_ground.py --device /dev/video0
```

按空格冻结画面，按照终端给出的顺序点击九个点，按`S`保存。建议重投影RMSE小于15mm；过大时重置并重新点击。

标定后不要改变相机高度、俯仰角、焦距或分辨率，否则单应矩阵失效。

当前默认分辨率已改为1280x1024。旧分辨率的标定文件不会被加载；新标定会同时生成`homography.txt`和记录1280x1024的`homography.txt.meta.json`。必须在新分辨率下重新标定一次。

## 5. 比赛运行

```bash
python3 run_detector.py --device /dev/video0
```

默认参数来自`config/rescue_vision.json`：1280x1024 MJPEG@180、JPU只取最新帧按60Hz解码。检测调度会受实际解码帧率限制。需要降低负载时，可在规定范围内覆盖：

```bash
python3 run_detector.py --device /dev/video0 \
  --danger-fps 60 --material-fps 60 --zone-fps 20
```

程序默认：

- 危险青色物资默认按120Hz调度，可在60～120Hz配置；
- 其余三类物资默认按90Hz调度，可在60～90Hz配置；
- 红蓝安全区和紫色入口默认按30Hz调度，可在20～30Hz配置；
- 只把多帧确认后的目标写入`runtime_result.json`；
- 输出文件采用临时文件替换，不会被导航程序读到半写入JSON。

结果示例：

```json
{
  "frame_id": 12345,
  "frame_age_ms": 4.2,
  "calibrated": true,
  "tracks": [
    {
      "id": "M7",
      "class": "danger_cyan",
      "confidence": 0.86,
      "state": "CONFIRMED",
      "position": [-125.0, 780.0],
      "coordinate_system": "ground_mm"
    }
  ]
}
```

导航程序必须执行安全策略：`danger_cyan`或无法可靠分类的目标不得进入收容动作。

## 6. 性能判定

编辑器和运行程序会显示实际解码及三级检测帧率。配置中的危险120Hz、普通90Hz、安全区30Hz是调度上限；JPU设置为60Hz时，各检测分支不会超过新解码帧率。`performance`中的两级识别默认先在640x512粗筛，再仅对候选ROI进行1280x1024精判；只有切换到“白黑掩膜”时才生成和放大调试掩膜。如果单次检测长期超过16ms：

1. 收紧`roi_polygon`，去除车身和远处无效画面；
2. 降低安全区识别频率，不降低危险目标识别频率；
3. 只在导航到目标附近时启用精细形状判断；
4. 保持只处理最新帧，不允许排队补处理旧帧。

## 7. 测试

```bash
python3 tests/smoke_test.py
python3 -m py_compile run_editor.py run_detector.py calibrate_ground.py rescue_vision/*.py
```

合成测试通过只说明代码和基本颜色规则正常；现场阈值仍需使用真实材料校准。
