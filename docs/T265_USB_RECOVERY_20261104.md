# T265测试入口的uhubctl恢复实验（历史证据）

2026-11-04，基于c302610增量加入，仅测试路径；用户明确允许当前共享USB组的恢复实验。

最新实测补充：`20261104_070043_interactive`的主机boot_id已变化，uhubctl返回成功且T265重新枚举，
但SDK固件仍上传0字节/15秒超时，没有Pose。这说明现有方法尚未解决实际重启异常；下文正常态成功实验不能替代该结果。
本轮仅分析该记录，未再次重置USB或扩大Hub操作范围。

## 方法与边界

RealSense官方仓库issue #4681讨论了主机启动后T265停在03e7:2150、重新插拔可恢复的问题，
并讨论uhubctl电源循环。它是硬件相关的解决办法，不是SDK保证成功的复位API。
uhubctl官方说明USB3 Hub同时有USB2/USB3虚拟Hub，默认会配对处理；软件端口关闭不一定代表VBUS真正断电。
另一案例#9660使用-R复位整个Hub，本次没有自动套用这个额外动作。

参考来源（完整链接）：

```text
https://github.com/realsenseai/librealsense/issues/4681
https://github.com/realsenseai/librealsense/issues/9660
https://github.com/mvp/uhubctl
https://github.com/mvp/uhubctl/releases/tag/v2.6.0
```

RDK Linux=6.1.83，Ubuntu软件源候选uhubctl=2.4.0。本机从官方v2.6.0源码
352f5878e999c0a9d5a453b34110479b2056d7e7编译安装到/usr/local/sbin/uhubctl；
2.6.0包含Linux 6.x sysfs路径修正。没有修改udev权限、没有自动下载安装逻辑或开机服务。

## 本机拓扑与操作

- T265运行态8087:0b37在2-1的1号口；boot态03e7:2150在配对1-1的1号口。
- Hub为05e3:0626/05e3:0610，均报告ganged power switching。
- USB视觉相机1bcf:28c4在1-1的3号口；F405走独立/dev/ttyS1。
- 实验只指定T265所在口，保留USB2/USB3配对。已明确告知共享供电可能影响视觉相机，并取得用户同意。
- 对本机已识别的Genesys Hub才加-f；其它Hub不强制。没有-R、没有所有端口操作，也不扩大到根Hub重置。

## 21de92f历史实现（已由快速启动替代）

现有t265_f407_motion_map.py新增--t265-usb-recovery；动态地图继承同一参数。
命令行默认关闭；按用户要求，原桌面“雷达-T265动态定位”现已携带此参数，供下次动态定位测试直接使用。
启用时在底盘订阅和唯一SDK进程启动之前运行一次，绝不在运动中自动复位/重试。
小模块t265_usb_recovery.py只负责USB目标发现、操作及记录，不拥有UART或SDK，不是独立测试程序。

按03e7:2150/8087:0b37发现唯一目标，从sysfs取得实际父Hub/端口；目标缺失/多个时不猜测地址。
先查设备占用，不打断已有SDK。命令为uhubctl -N -l 实际Hub -p 实际端口 [-f] -a cycle -d 5。
-N仅跳过可能无响应设备的字符串描述查询，不改变电源操作；保留默认USB2/USB3配对。
若请求失败或中断，尝试给相同端口恢复on，不重复cycle、不扩大操作范围。

命令、返回值、输出、USB前后设备、Linux boot_id写入本会话t265_usb_recovery.json，metadata引用其结果。
文件已加入原模块日志保留名单，沿用最近5次完整会话机制；失败/不完整/保护会话不清理。
USB助手日志只描述其操作阶段，SDK/Pose证据在后续原始日志；不会伪报VBUS已测低、设备内部已彻底清空或冷启动故障已恢复。

## 实际验证

日志：rescue_map/runtime/history/t265_f407_motion_map/20261104_050445_interactive/

- 从正常运行态T265开始，显式执行一次5秒cycle；不是RDK重启后的异常态实验。
- uhubctl实际操作了2-1/1-1的1号口；运行态2-1.1设备3变成boot态1-1.1设备6，随后SDK运行态设备4。
- 视觉相机仍为1-1.3设备4，未观察到其USB地址变化；未测量实际VBUS电压。
- T265固件上传0.346秒，Pose1943条、约199.57Hz，回调队列丢弃0。
- 真实轮速SDK结果516次true、0次拒绝，活动频率约50.17Hz；正常退出、完整会话标记存在。
- 无commands.jsonl或chassis_actions.jsonl，未发送运动/STOP/reset。
- 原始实验命令未加-N；最终仅增加-N描述查询优化，已用本机只读状态查询确认支持。
  失败恢复on分支复用现有测试文件做模拟检查，不额外循环实体USB。
- 必要Python语法、唯一目标/歧义/已有SDK占用、失败cycle仅恢复同口on的检查通过。

结论：当前设备状态下，恢复操作能够使T265重枚举并正常启动Pose/官方轮速输入。
“切换供电、RDK重新启动后出现的异常能否恢复”仍待用户下次实测，不能由上述正常态实验替代。

## 当前入口

当前原动态桌面保留同名参数，但已取消固定5秒uhubctl循环。运行态直接交给唯一SDK进程，boot态用现有工具快速上传固件。
本轮更多复位手段仍未恢复真实故障；当前改动、实测和RDK源码实例见[T265_USB_FAST_START_20261104.md](T265_USB_FAST_START_20261104.md)。
用户已明确断电有效，不把人工拔插作为最终自动恢复方案，不再要求重复拔插证明。
