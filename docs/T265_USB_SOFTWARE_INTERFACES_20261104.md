# T265纯软件恢复：树莓派方法及RDK接口实测

2026-11-04，基于`c0efacc4b2e6f3615c0600b899c98087a20d8bba`。
用户当前决定：**只用纯软件，继续核查接口，可参考树莓派免拔插方法；不准备硬件改线方案。**

软件试验未恢复：5项实际尝试后的官方固件上传均报告`0/9323648`字节、1秒传输超时；
没有进入`8087:0b37`运行态，没有Pose。命令成功、重枚举和驱动重新绑定均不算T265恢复。
运行代码/默认启动未改；没有把这些失败步骤加入自动启动，5秒对照也没有成为默认等待。

## 树莓派资料能支持什么

- RealSense issue 7772：Pi 3B+用户报告`usbreset`可恢复多次运行后失联的相机，问题描述中的失败态是`8087:0b37`。
  这与本机卡在`03e7:2150`、固件传输0字节的现象不同。USB设备复位已在本机试过，没有理由只换命令名称再试。
- RealSense issue 9660：Pi 4B用户报告`uhubctl -l 1-1 -a cycle -d 5 -R`在主机启动后执行一次有效，
  并提醒不能每次启动相机都运行。本轮按本机实际Hub/端口复现5秒版本；此前只有零延时版本，二者分开记录。
- uhubctl项目说明Pi 4B依赖VL805固件和成组供电设计；本机是DWC3＋Genesys `05e3:0610/0626`，
  不能照抄Pi的PCIe路径、EEPROM升级或电源控制结论。Hub返回`off`不证明实测VBUS已降到0V。

资料是方法来源，不是RDK成功证据。出处见文末。

## 本轮实际操作与结果

| 操作 | 实际读回/行为 | 固件结果 |
| --- | --- | --- |
| T265单设备runtime suspend/resume | `control=auto, autosuspend_delay_ms=0`；约0.099s读到suspended，约0.432s恢复active；随后恢复原设置 | 0字节 |
| USB旧枚举顺序 | 临时`old_scheme_first=Y,use_both_schemes=N`，用原boot工具执行单设备reset；随后恢复N/Y | 0字节 |
| Pi方式5秒断开＋Hub reset | 实际对1-1及其配对2-1的全部4端口执行，`-N -S -f -a cycle -d 5 -R`；命令耗时7.757s，Hub报告reset成功，相机重新枚举 | 0字节 |
| 外层DWC3驱动重新绑定 | 仅`dwc3-of-simple/35100000.usb3`；unbind0.252s、bind0.034s，约1.354s读到已配置设备 | 0字节 |
| 只关闭USB3配对端口 | `-e -l 2-1 -p 1 -a off`，USB2的1-1端口1仍报告480Mbps连接；单设备reset后尝试固件，最后恢复USB3端口 | 0字节 |

每项用现有`t265_boot --timeout-ms 1000 --chunk-kib 256`判断实际固件结果，没有SDK双开。
最后一项只证明当前故障态下隔离SuperSpeed路径没有恢复，**不是更换物理USB2口或冷启动USB2的完整试验**。

DWC3源码的外层remove会释放子控制器和时钟；实际设备树没有该节点的`resets`属性，
不能把重新绑定称为已断开T265电源，也不能声称已触发一个不存在的板级reset引脚。

## 引导接口的新证据

直接通过标准USB GET_DESCRIPTOR/GET_STATUS读取设备，而非只看sysfs缓存：

- device descriptor返回18字节，bcdUSB仍为1.10；当前总线速度480Mbps。
- device qualifier返回10字节，bcdUSB为2.00；配置和other-speed配置均返回32字节，bulk端点最大包均64字节。
- bulk OUT 0x01和IN 0x81的GET_STATUS均返回`0000`，没有报告endpoint halt。
- RDK内核`config.c`对高速bulk端点非512字节打印警告，此处没有自动改成512；不能把警告本身当作内核拒绝所有上传的证据。

这进一步缩小到引导设备/USB传输状态，但不能仅凭这些日志唯一判定Boot ROM、PHY或主机控制器哪一侧有错。
没有halt时反复clear_halt无明确依据；SDK的`hardware_reset()`（vendor request 0x10）属于运行态TM2接口，
现有`tm-boot.h`只定义固件bulk上传，没有提供已核验的boot态reset请求。本轮未猜测vendor命令，未触碰标定。

## 原始记录及恢复状态

RDK原始目录：

```text
/home/sunrise/RDK_X5/shijue_fangan/rescue_map/runtime/history/t265_f407_motion_map/20261104_114046_usb_software_interfaces/
```

包括before、runtime_suspend_resume、boot_old_scheme_reset、pi_style_5s_cycle、dwc3_outer_rebind、
ss_companion_disabled_trial、live_control_reads、各次上传结果、kernel_usb_tail及result_summary。
整个过程仍在boot_id `1860094a-05cd-49f2-9935-98360dbd5842`，没有重启主机。

最终已读回：枚举参数N/Y、角色host、T265配置1、电源策略on/2000ms、外层驱动已绑定、
USB2/USB3 Hub所有端口power，视觉相机已重新枚举，USB无进程占用。
没有启动电机、打开底盘UART、清理日志、修改正式比赛/下位机/内核/设备树/接线。

## 当前结论与未完成项

本轮核查和实测完成，**纯软件自恢复仍未解决**。没有可靠依据交付新的自动复位链。
后续只在发现可核验的新接口时继续，例如Boot ROM明确支持的复位协议，或本版X5真实可用的板载USB供电控制；
不重复排列已失败的reset命令，不再把硬件改线方案列为等待用户批准的当前任务。
这里没有证明所有纯软件方法都不可能，也没有把树莓派成功案例说成本机已成功。

## 随后用户物理重插：恢复对照

用户报告已经重插，基于`2b773bf`复核；没有再次软件reset或重启主机。
引导描述符已变为USB2.00、bulk最大包512字节（此前异常态为USB1.10/64字节）。
同一`t265_boot --timeout-ms 1000 --chunk-kib 256`在0.345s内传完9,323,648字节，
并实际枚举为`8087:0b37`；总引导命令耗时1.295s。

现有`t265_debug --wait 5 --duration 5 --print-rate 1 --csv ...`读取原生Pose：
999帧、199.79Hz、帧号缺口0，终端tracker confidence为2；SDK/固件2.50.0/0.2.0.951，
实际序列号944222110255。程序正常退出，最后USB无进程占用。
未开UART、未启动电机或轮速注入；这只确认当前相机可用，不是运动精度或软件自恢复验证。

日志：`rescue_map/runtime/history/t265_f407_motion_map/20261104_114839_usb_replug_check/`，
保存重插后descriptor、boot、sdk_pose_check、原始CSV及result_summary；未清理此前失败记录。

参考源码/第一手记录（完整URL便于复制）：

```text
https://github.com/realsenseai/librealsense/issues/7772
https://github.com/realsenseai/librealsense/issues/9660
https://github.com/mvp/uhubctl/blob/master/README.md
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/core/hub.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/core/config.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/dwc3/dwc3-of-simple.c
https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-boot.h
https://github.com/realsenseai/librealsense/blob/v2.50.0/src/tm2/tm-device.cpp
```
