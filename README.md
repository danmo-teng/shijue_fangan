# RDK X5 救援机器人视觉、UART与T265调试方案

本仓库整理了RDK X5端本次救援机器人项目实际使用的代码，包含：

- RDK X5 BPU YOLOv8物资主识别，以及保留的Web传统视觉备选；
- MJPEG 1280×1024@180 FPS采集、最新压缩帧队列、JPU 60 FPS解码；
- 普通物资识别、本地等比例全屏显示和STM32F407 UART上报；
- T265二维轨迹、车头方向和累计行驶距离调试窗口；
- T265与三轮编码器融合定位；
- F407 UART协议、编码器打包和融合位姿参考实现。

## 目录

```text
vision/             Web调参、识别、JPU和UART视觉闭环
t265_trajectory/    T265单机诊断及二维轨迹窗口
t265_map/           T265环境扫描、localization map导入导出和F407运动调试
localization/       T265 + F407三轮编码器融合定位
docs/               给电控端的UART接入说明
```

## 1. 视觉环境部署

```bash
cd vision
bash deploy_rdkx5.sh
```

摄像头默认工作方式：

```text
MJPEG 1280×1024 @ 180 FPS
  -> 始终覆盖保留最新压缩帧
  -> RDK X5 JPU直接输出NV12
  -> VSE硬件缩放320×256并填充为320×320
  -> BPU YOLOv8s物资与红/蓝安全区识别（置信度>=0.50）
  -> 传统视觉仅作为显式回退
```

## 2. Web调参

电脑建立SSH转发：

```bash
ssh -L 8080:127.0.0.1:8080 sunrise@RDK_IP
```

RDK X5运行：

```bash
cd vision
python3 web_editor.py --device auto --decoder jpu --decode-fps 60
```

电脑浏览器打开`http://127.0.0.1:8080`。界面保留原项目的多参考阈值、框选自动取值、形状规则、曝光/白平衡/焦距诊断及原图/掩膜/分类视图。点击“保存全部配置”后按`Ctrl-C`退出。

## 3. 普通物资传统视觉备用识别与UART

```bash
cd vision
export DISPLAY=:0
python3 run_normal_supply_uart.py \
  --device auto --decoder jpu --decode-fps 60 \
  --window-mode fullscreen --uart /dev/ttyS1 --baud 115200
```

该入口保留用于传统视觉和UART单项调试；完整任务默认通过`mission_test/run_mission_test.sh`
使用YOLO识别，并由定位进程统一转发UART帧，避免两个进程争抢`/dev/ttyS1`。

1280×1024画面会等比例完整缩小到桌面，绝不裁剪；1024×768屏幕上显示为960×768，左右各补32像素黑边。按`F`切换全屏，按`Q/Esc`退出。

RDK X5 UART1接线：物理8脚TX连接F407 PD9/USART3 RX，物理10脚RX连接F407 PD8/USART3 TX，两板共地，3.3V TTL，115200 8N1。

## 4. T265二维轨迹

项目使用官方librealsense 2.50。SDK源码和本机构建产物较大，未提交到本仓库；配置时通过`REALSENSE_ROOT`指向包含`third_party/librealsense`和`build-250`的目录：

```bash
cd t265_trajectory
cmake -S . -B build \
  -DREALSENSE_ROOT=/path/to/T265-sdk-root \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2

export DISPLAY=:0
./run_trajectory.sh --min-confidence 2 --scale 150 --csv trajectory.csv
```

轨迹窗口显示二维路线、车头方向、当前位置、置信度和`path`累计距离。按`R`重置原点及距离，按`F`切换全屏，按`+/-`缩放。

## 5. T265与编码器融合

```bash
cd localization
cmake -S . -B build \
  -DREALSENSE_ROOT=/path/to/T265-sdk-root \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

协议与电控侧改动参见：

- `docs/f407_uart_integration_guide.md`
- `localization/docs/uart_protocol.md`
- `localization/firmware/`

## 6. T265环境扫描与地图复用

`t265_map/`是独立的T265建图和验证程序。它使用T265内部mapping/relocalization，导出
`t265_localization.raw`，并在导入前完成`pose_sensor.import_localization_map()`；每次运行会在
`rescue_map/runtime/history/t265_map_builder/`下保留`metadata.json`、T265原始/修正轨迹、
编码器轨迹、F407状态和事件日志。桌面入口由
`t265_map/install_desktop_launcher.sh`安装为“T265-环境扫描建图”。

程序默认只观测，不发运动命令。待F407实现`t265_map/README.md`中定义的`TYPE=0x19`
扫描运动命令和`TYPE=0x1A`状态后，显式使用`./run_t265_map.sh --enable-motion`，即可通过
界面执行原地90°/180°/360°、直行1m、横移1m和“一次转向直线返航”测试。新接口与当前任务
`0x17/0x18`含义分离，不改动现有任务通信。

## 7. 四出发区救援地图

`rescue_map/`提供四个出发区、红/蓝方及融合/仅T265选择，并在桌面窗口显示赛题场地、小车位置、方向、轨迹和行驶距离。点击开始会同时启动完整YOLO/STM32任务；仅T265模式仍保留UART任务通信，只禁止编码器参与定位融合。详见[rescue_map/README.md](rescue_map/README.md)。

板端桌面可双击“智能救援一键启动”；快捷方式可通过`rescue_map/install_desktop_launcher.sh`重新安装。

T265镜头朝上安装后，姿态和位移使用完整四元数及三维轴向投影；不再使用二维
`camera_to_robot_yaw_deg`猜测补偿。实车校准后使用机器人前向=`+T265 Pose X`、上向
=`-T265 Pose Z`，因此左向为`-T265 Pose Y`；注意T265 Pose坐标与图中的IMU子坐标不是同一套轴。

## 8. 连续物资抓取与分区投送

`mission_test/`实现连续寻找、抓取和分区投送：开局强制先完成一次普通物资，之后允许搬运其他物资或伤员；每次投送后F407张爪退出并靠近中心区域继续搜索。RDK不直接下发实时位置坐标，而是在返安全区和靠近中心期间根据融合位姿持续更新“航向+剩余距离”。详见[mission_test/README.md](mission_test/README.md)和[电控UART联调说明](docs/f407_uart_integration_guide.md)。

`TYPE=0x18`任务命令由独立串口线程以100 Hz更新SEQ/CRC，不依赖T265取帧主循环；识别窗口会显示实际转发SEQ、帧龄、累计帧数和错误数。
NAV和RETURN_CENTER另由独立50 Hz规划线程读取融合位姿与STM32状态，持续更新航向和剩余距离；
安全区到达可由地图接触或下位机`DISTANCE_DONE`锁存确认。
返航定位采用编码器平移预测为主、T265低频低权重位置修正；接近围栏检测轮子空转并冻结轮式
增量，避免T265少算路程或编码器空转继续污染地图位置。

## 安全说明

- RDK与F407连接前应断电，确认TX/RX交叉并共地；
- 40Pin为3.3V逻辑，禁止直接连接RS-232电平；
- 纯通信测试阶段应架空车轮或让F407进入只显示、不驱动电机模式；
- `DISTANCE_VALID=0`时电控只能使用目标图像坐标，不能执行距离停车或夹取。
