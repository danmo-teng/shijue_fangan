# 2026-11-09 提线C++可行性原型

这些是性能分析的源码证据，不在CMake、正式或测试启动路径中加载；没有独立测试入口、设备访问或运动命令。
用户尚未选择迁入运行链。原运行算法仍在`localization/tools/lidar_yaw_debug/static.py`。

`lidar_extract_20261109.cpp`只导出C ABI数值函数，复用现有`lidar_geometry.cpp`候选评分；
8轮/160候选、两次TLS、25mm支持带、断段/分位跨度与32点抽样规则保持。
`lidar_extract_overlay_20261109.py`是已有static模块的临时覆盖片段，保留原NumPy RNG序列，
将当前NumPy所用OpenBLAS的LAPACKE函数地址传入原型，避免更换SVD库掩盖数值差异。
V1还保留原NumPy argsort：V0使用std::sort，在投影并列时改变支持点抽样，合成墙解差0.625～3.90625mm。
保留原排序后13个双墙/离群/并列/跨度/断段/支持边界案例的支持点、观测秩和墙解一致，未改几何阈值。
仍有逐轮NumPy抽样/排序回调，不是已实现完全脱离Python的提线器。

编译原型共享库，不生成新测试程序：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
g++ -std=c++14 -O3 -ffp-contract=off -shared -fPIC \
  docs/reports/prototypes/lidar_extract_20261109.cpp \
  localization/src/lidar_geometry.cpp \
  -o /tmp/rdk_lidar_extract_prototype_20261109.so
```

评估使用已有`test_f407_motion_protocol.py`的`competition_control_rate_fixture`与既有几何检查。
只在`/tmp`创建包覆盖：`__init__.py`扩展到原包路径，`static.py`由原文件加覆盖片段组成；
启动该次检查的`PYTHONPATH`优先指向临时包。安装入口和原文件不变，不把该环境用于实车会话。
具体输入、全部轮次及差异见`../localization_joint_latency_20261109.json`。

V1的139帧/143线支持点完全相同、法向分量差0、跨度差≤2.22e-16m；
中位7.494→3.071ms（2.44倍）、P9511.320→4.975ms（2.28倍）。不要拿有排序差异的V0约3倍代替当前数字。
单/双墙、四个旋转对称分支、围栏重获、动态去畸变相关检查通过。
一次动态检查最初因控制快照新增“省略显示证据”的契约而失败，已修正该比较并复验；
不是修改几何容差使原型通过。没有通过地面精度或最坏延迟验收。

若选择迁移，还需将库加载/ABI整理为正式可选后端，明确当前NumPy/LAPACK兼容范围，
核对边界点/并列投影排序以及同入口实车数据；不能把这个机器上的原型直接称为交付。
