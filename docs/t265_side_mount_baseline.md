# T265 侧装正式比赛配置（2026-10-15）

正式比赛同步 `c9323aa` 桌面实测方案。历史 `277c4a8` 推导的 `-x/+y`
基线已被本轮实测替代，使用完整四元数投影，不增加平面猜测角度：

```ini
camera_robot_forward_axis = +x
camera_robot_up_axis = -y
```

程序自动按 `left = up × forward` 得到 `left = +z`，并用四元数旋转前向、左向和上向，连续累计相对 yaw，再投影 T265 角速度。出发区坐标和四个初始车头方向不变，因为它们描述的是底盘而不是传感器安装姿态。

正式模板和C++默认值使用：

```ini
camera_offset_forward_m = -0.020
camera_offset_left_m = -0.070
encoder_to_robot_yaw_deg = 180.0
t265_translation_scale_enabled = false
t265_translation_scale = 1.000
encoder_fusion_weight = 0.0
wheel_center_radius_m = 0.0
navigation_distance_compensation_enabled = false
start_heading_offset_deg = 0.0
```

后20mm、右70mm表示相机跟踪原点相对底盘中心的偏置，不改变底盘出发位置。
四个出发区航向仍为135°、45°、225°、315°，出发位置仍由原角点距离选项生成。
不要复制桌面测试的中心起点或额外45°初始航向。

正式默认仅T265，自动传入`--t265-only-output --enable-pose-jumping`。
每个有效T265位姿经安装外参修正后直接作为`pose`输出，不经过EKF预测、
观测混合或600mm创新拒绝门限。置信度为0时沿用最后有效位置，原失效判断保持。
编码器继续接收、解算、绘制和记录，只作参考；轮式轨迹用三轮位移与T265航向，
不参与导航距离补偿，也不向T265注入轮速。比赛仍读取原有`pose`接口。

SDK显式位置跳变设置在地图导入模式也有效。全部设置和启动完成后的实际回读
写入`localization_events.jsonl`的`t265_pose_jumping_final`；正式默认应为1。
地图导入仍是原可选功能，不因迁移自动启用。保留原地图启动握手和比赛串口配对。
测试程序的MOVE_BODY/TURN_REL符号修正不用于正式任务指令。

## 部署

退出旧比赛进程后，在上位机仓库执行：

```bash
git pull --ff-only origin codex/gamepad-teleop
cmake -S localization -B localization/build -DCMAKE_BUILD_TYPE=Release \
  -DREALSENSE_ROOT=/home/sunrise/文档/ChatGPT/T265
cmake --build localization/build --target t265_omni_localizer -j2
```

双击桌面“智能救援完整比赛流程”，或运行`competition_rescue/run_competition_map.sh`。
重新开始会话时自动生成`rescue_map/runtime/localization.conf`，不用改历史日志或手工编辑
旧运行配置。本次上位机迁移不要求重新烧录下位机，也不修改现有任务状态机、速度、
审核、投送计数及协议。

## 必要验证

只检查本次定位配置、正式启动参数、四区位置/航向和侧装前进/左移/逆时针方向，
复用现有检查程序，不新增测试程序或执行无关整套回归。实车正式入口首次运行时，
确认位姿起点、航向及轮式记录与所选出发区一致。
