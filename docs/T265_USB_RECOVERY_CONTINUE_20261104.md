# T265自恢复继续排查：角色切换与引导端点

2026-11-04，实际基线fb0cc2f，用户本轮要求继续尝试修好重启自恢复。
结论：**新增三种尝试仍未恢复设备，真实Pose为0；本轮不是修复成功交付。**
未重复已失败的uhubctl/xHCI/根端口循环，未要求用户再手工拔插。

## 本轮新证据

本机仍在同一主机boot_id（1860094a-05cd-49f2-9935-98360dbd5842），T265为03e7:2150，无SDK进程占用。
USB3角色接口为`/sys/class/usb_role/35100000.usb-role-switch/role`，与底盘UART、USB2闪连控制器分开。
核对RDK官方内核`drd.c`，该接口允许userspace控制；不是猜测寄存器或GPIO。

1. **host→device→host**：实际读回device用时0.319s、恢复host累计0.437s；约4.039s出现已配置设备。
   随后上传官方固件仍0字节，1s传输超时，整个尝试约5.222s。
2. **64字节分块**：发现设备以480Mbps枚举，却报告bcdUSB=1.10、bulk最大包64字节，内核报invalid maxpacket 64。
   在原t265_boot中加入可选字节粒度后试64字节，连第一块也未收到，仍0字节/1s超时。
   这不支持“仅是原256KiB块太大”的解释，不能声称改小块已修好。
3. **USB configuration 0→1**：只对T265解除配置再重新配置，读回1；同样仍0字节上传。

原始记录（保存在RDK，未清理）：
`rescue_map/runtime/history/t265_f407_motion_map/20261104_111607_usb_recovery_continue/`

包括role_switch、各次boot结果、原始descriptor十六进制、endpoint_reconfiguration及result_summary。
实验结束已确认USB3角色host、T265配置1，没有遗留SDK/boot进程。视觉相机已重新枚举。
没有启动电机、底盘UART或雷达，没有更改内核、设备树、未知GPIO、标定或正式比赛流程。

## 本轮代码修改与检查

只增量修改`t265_trajectory/src/t265_boot.cpp`，增加显式诊断参数`--chunk-bytes BYTES`。
原`--chunk-kib`兼容，默认仍256KiB，官方固件文件和USB bulk数据内容保持不变。
帮助信息及BOOT日志明确显示字节数；没有把64字节或失败复位链设成自动启动默认。

必要验证：现有t265_boot目标增量编译通过；实际64字节上传尝试失败且如实记录；
0、超过4MiB、非整数参数均在打开设备前拒绝。没有新增独立测试程序。

## 对自恢复方向的判断

当前证据更指向设备引导/USB协商或固件接收状态，不能仅凭日志确定电气根因。
控制端点还能返回descriptor，并不代表固件接收bulk通道正常；USB重枚举也不等于真正掉电复位。
同类NVIDIA官方论坛记录出现相同的480Mbps/64字节异常，但它不能代替RDK验证。

RDK公开普通X5资料只有接口原理图，X5 Module供电图不能直接当本机V1.0的引脚定义。
本机无已启用USB VBUS regulator；用户已经明确真实断电能恢复，这一事实不再重复要求验证。
目前建议的最小备选：T265 USB 5V独立可控负载开关，由RDK空闲GPIO控制；USB数据保持连接，
具体是否还需隔离数据线回供、放电时间和恢复枚举要由器件/接线及实测决定，不承诺零秒恢复。
只复位T265，不改F405，不与RGB相机共切电；不能用GPIO直接给相机供电。

接线/接口变动须由用户决定。本轮已提出“先给最小硬件方案”或“仅继续软件接口核查”的选择，尚未获得该硬件方案授权。
没有擅自部署开机服务或追加永久重试/故障锁存。下一步不能再把没有实际成功证据的复位命令称为自恢复。

参考来源：

```text
https://github.com/D-Robotics/x5-kernel/blob/main/drivers/usb/dwc3/drd.c
https://github.com/D-Robotics/x5-kernel/blob/main/drivers/usb/phy/phy-sunrise5-usb.c
https://forums.developer.nvidia.com/t/disable-enable-usb-ports-on-orin-nx-programmatically/273196
https://archive.d-robotics.cc/downloads/hardware/rdk_x5/
```
