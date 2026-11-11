# 正式预热失败：C++混编产物恢复

2026-11-11；源码基线97235a3cb1dfdcabef8c4ac52399ea4841ea5a46。

## 原因与证据

10:52:32、10:53:26、10:54:52三次预热均由动态定位工作进程报出
`basic_string::_M_create`，随后正式入口结束预热。原始记录位于
`rescue_map/runtime/history/competition_wheel_input/20261111_105232_competition_wheel/`
及同目录的105326、105452会话；各自`lidar_worker_events.jsonl`和`dynamic_events.jsonl`
记录同一错误，未修改或清理。

故障库`localization/build/libmotion_runtime.so`混用了不同头文件布局的对象文件：
`NativeMotionSource`构造函数将字符串指针/长度写在对象+1024/+1032，
同一库的`snapshot()`却从+1040/+1048读取，相差16字节，将其他内存误作字符串长度。
旧库SHA256为`8766299b23bdca61eef23606b7b1da294aa167ff917a961da01c2577347bea21`，
BuildID为`274bd7541aa8a71e74c593d7149d6d431ea480c8`。
这是此前未完成实验构建覆盖正式产物的交付问题，不能由当前Git提交号证明二进制一致。

不接设备、fd=-1、实验模式关闭，也能在创建后首次snapshot重现。
`NativeDynamicFence`构造时读取该状态；异常清理再次snapshot抛出，
因此原Python回退也未执行。不是因为用户没有按3、没有看到围栏或下位机拒绝运动。

三次日志均有T265 pipeline_started；最近一次已有真实Pose且SDK轮速API成功53次、失败0次。
下位机实报`e85c738075451574e4c29ea7da05daf5a8258b6b/dirty1`，NormalRun/TASK、921600。
这些只证明相关通路曾工作，不等于定位精度通过。此次失败无须归因于T265固件上传或SD满。

## 恢复范围

保留工作区全部未提交实验源码、用户ROI、原始日志及用户资料。
从上述已提交HEAD导出独立源码快照，在临时目录干净Release编译，
验证后统一替换正式目录的可执行文件、两份共享库及匹配的原测试二进制。
不将当前未验收实验增量重新编进正式目录，不回退本轮载荷换角/拒绝回执修复。

当前未提交Python的正式默认路径与该HEAD的C ABI兼容：封装签名未变；
新增配置字段由旧库忽略，新增坐标逻辑仅实验开关打开时执行。
未修改比赛算法、安装参数、协议、下位机或新增运行限制。

## 实际恢复与必要检查

已从HEAD干净构建并替换正式三个运行产物及匹配测试程序。
旧产物、原始诊断、源码归档、构建/检查输出及前后指纹保存在
`/tmp/rdk_formal_recovery_97235a3_2l1oraj9/`；本机部署回执为
`localization/build/formal_build_provenance_20261111.json`。
运行产物SHA256：

| 产物 | SHA256 |
| --- | --- |
| t265_omni_localizer | 9450e224dd13eb71638b1afdb819b734a980d3be2b3e27ae2d59be05339fd174 |
| libmotion_runtime.so | b2d2e8d30161a5c99edf594daf62c7b4520c50ee614d0dd054df1d08752d1baa |
| liblidar_geometry.so | 8ba7d1ad9a3dd6fb273192e39c8eef2cb821f2e863958703ebdc7bf9dbca8975 |

- 新库构造/快照的字符串指针与长度均为+1008/+1016，原错位已消除。
- 影子库及部署后正式默认路径均通过create→NativeDynamicFence→snapshot→close，无设备输入。
- 原CTest 2/2通过，0.26秒；C协议测试显式保留assert，避免Release关闭检查。
- 原`test_native_runtime_motion_source_parity`及
  `test_native_runtime_socket_input_and_control_parity`通过，约17.36/0.28秒；
  复用已有离线输入/本地socket检查，未新增测试程序。
- 三个运行ELF和匹配C++测试程序的动态依赖均可解析；部署后字节与验证产物相同。

本轮不启动真实预热、SDK、UART、雷达或电机，软件检查不能称为整链实车预热成功。
