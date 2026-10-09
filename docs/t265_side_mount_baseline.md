# T265 朝右侧向安装基线

本仓库历史提交 `277c4a8` 的朝右安装曾使用原生 `-Z` 前向、`-X` 左向，再加平面 `-90°` 修正。当前完整四元数投影中不再恢复 `heading_y()` 或 `camera_to_robot_yaw_deg`；两者的等价轴配置为：

```ini
camera_robot_forward_axis = -x
camera_robot_up_axis = +y
```

程序自动按 `left = up × forward` 得到 `left = +z`，并用四元数旋转前向、左向和上向，连续累计相对 yaw，再投影 T265 角速度。出发区坐标和四个初始车头方向不变，因为它们描述的是底盘而不是传感器安装姿态。

侧向首次实车基线使用：

```ini
camera_offset_forward_m = 0.0
camera_offset_left_m = 0.0
t265_translation_scale_enabled = false
t265_translation_scale = 1.000
encoder_fusion_weight = 0.0
wheel_center_radius_m = 0.0
```

正式启动已经是 T265-only，并自动传入 `--ignore-encoders`；编码器仍可保留在 UART/状态链路中，但不进入定位预测或融合。下位机不需要改动。

验证顺序：静止观察、沿车体前向移动、向车体左侧横移、原地逆时针转90°。应分别看到 forward、left、yaw 的单独变化；若前后或左右反向，只调整对应轴的符号，不恢复二维 yaw 补偿。若轴向正确但距离比例有系统误差，再单独开启比例测试。
