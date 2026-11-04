# T265 USB启动故障：本机复位实验与RDK案例核查

2026-11-04，基于235b0e8。用户要求快速恢复、不要固定等待，并明确“断电就好了”，
要求查RDK＋T265复位案例；不再要求用户重复手动拔插证明这一点。
**本轮未恢复当前故障设备，没有Pose成功证据，不能称USB问题修好。**

## 本机实际状态与实验

- D-Robotics RDK X5 V1.0，Linux6.1.83，主机boot_id为1860094a-05cd-49f2-9935-98360dbd5842。
- T265卡在03e7:2150；相机组为Genesys 05e3:0610/0626，报告ganged switching。
- 所有实验前检查无SDK/USB设备占用；同组仅T265和空闲DHZJ视觉相机，没有USB存储或网卡。
- 未启动电机、未打开底盘UART、未重启RDK、未修改下位机/设备树/内核/未知GPIO/寄存器。
- 原始证据：`rescue_map/runtime/history/t265_f407_motion_map/20261104_104058_usb_repair/`。

| 实测方法 | 结果 |
| --- | --- |
| 现有t265_boot的libusb单设备复位 | 仍0字节，1.002s传输超时 |
| uhubctl直接Hub请求（-S），仅端口1，cycle延时0 | 重枚举；过早上传NO_DEVICE，枚举完成后再上传仍0字节/1s超时 |
| 同端口零固定延时循环＋-R重关联Hub | 总耗时6.567s；固件仍0字节/1s超时 |
| ganged Hub四个端口一起关闭/打开，固定延时0 | 总耗时6.223s；固件仍0字节/1s超时 |
| 仅USB3的xhci-hcd.2.auto驱动解绑立即重绑 | 解绑重绑0.386s、枚举约3.86s；固件仍0字节/1s超时 |
| 控制器USB2/USB3两个根Hub唯一端口均关闭后立即打开 | 总耗时8.00s；固件仍0字节/1s超时 |

所有电源/重枚举命令返回成功，不能证明VBUS真正降到复位电压。没有电压测量，不能凭sysfs消失宣称已真实断电。
共享相机在部分实验中重新枚举；实验结束已恢复全部相关端口供电和原驱动绑定，未留下后台复位进程。
自然枚举和驱动初始化仍需时间；“不加固定sleep”不等于设备可在零秒内完成启动。

## 专项查到的RDK案例：读源码后再判断

1. **FDogeLover/AUV，RDK X5＋T265项目。** `docs/known_issues.md`问题1明确记录：
   uhubctl在已连接设备上曾成功，但冷启动故障连续两次未恢复，仍未解决；不能称为现成可靠复位方案。
2. **LONEFORME/lingxiao-drone，RDK X5机载项目。** 网站指南描述有“硬件复位自愈”看门狗，
   实际`自研系统/09_部署与运维/watchdog/t265_monitor.py`（本次读取blob6408afa3429caa12063a1f22ef64e78f2d321202）
   只运行lsusb并显示READY/NEED REPLUG/NOT FOUND，没有GPIO供电切换、USB复位或恢复固件的实现。
   因而不能把网站的描述当成已验证硬件复位代码。
3. **RealSense官方仓库#9660。** 有其它板卡通过uhubctl -R恢复的报告，也有不成功报告。
   本机已经按对应方法试过，未恢复，不能将树莓派/Jetson案例冒充RDK成功案例。
4. **Intel T265勘误TM2-4233。** 公开建议保证Hub/控制器先完成枚举，再给T265供电。
   这支持上电顺序方向，但不证明本板的逻辑端口切换已经控制了真实5V。

本轮没有找到可核验的“RDK X5原板、不增加可控供电硬件、可靠且立即恢复该0字节故障”的成功实现。
这不等于断言不存在；当前已查实例和本机实验均不足以保证软件复位成功。
不能照抄其它项目的置信度门控、看门狗、永久停车或模拟Pose逻辑。

## RDK供电控制核查

本机只读设备树/调节器：USB相关PHY节点disabled，没有暴露USB VBUS regulator。
对照D-Robotics x5-kernel的x5-rdk.dtsi与phy-sunrise5-usb.c，确有可选set_vbus/regulator接口，
但“驱动有接口”不代表本机有可控电源接线。公开X5 Module载板有USB3_VBUS_EN电路，
本机是X5 V1.0而非Module，不能照搬引脚或盲写GPIO。
下一步应核实本板USB VBUS使能的实际接法/可控性；若原板无法控制，才讨论外接单路可控USB供电，
由RDK控制T265真实断电后恢复。未擅自要求F405增加功能，也未自动修改系统供电驱动。

## 本次交付的快速启动改动（不是USB故障修复证明）

原桌面入口仍使用同一`--t265-usb-recovery`参数，但`recover_t265()`不再每次强制5秒Hub循环：

- 已是8087:0b37：跳过复位，交给原唯一SDK进程验证实际Pose。
- 03e7:2150：立即用现有t265_boot加载官方固件，256KiB块、每块1s传输超时；无固定延时。
- 上传失败如实记录、快速报错；不重复已实测无效的USB复位，不把枚举当Pose成功。
- 固件工具仍等待必要的运行态枚举，最长边界不是固定等待。未修改正式比赛启动脚本。
- 设备被占用时不重置、不争抢USB；保留原日志格式及当前会话保护。

已有测试文件中的目标发现、运行态跳过复位、boot加载、占用保护、失败不重试检查通过；这些是mock检查。
当前故障态底层上传实际仍失败；快速成功启动和冷启动自动恢复尚未实机通过，不伪报验证结果。

## 本轮显示与安装图

原动态地图：围栏2px→1px、网格改浅；雷达点绿色1px→红色2px并抗锯齿，暂停雷达时仍灰色。
用已有扫描离线渲染检查，未启动真实雷达。桌面模板与实际桌面图标同步说明。

俯视图：`figures/sensor_top_view_20261104.png`与同名SVG。
T265定位中心在右后，雷达在左后；按当前配置绘制，未编造真实车壳轮廓。
用户说明“画面相对常见正装翻转180°”与现有`camera_robot_up_axis=0,-1,0`一致：
`+Z_B=-Y_C`。它与绕车体竖直轴顺时针30°是不同旋转，不再额外叠加180°。
位置、旋转候选与比例均未改。

## 可核对的完整来源链接

```text
https://github.com/FDogeLover/AUV/blob/main/docs/known_issues.md
https://github.com/LONEFORME/lingxiao-drone/blob/main/%E8%87%AA%E7%A0%94%E7%B3%BB%E7%BB%9F/09_%E9%83%A8%E7%BD%B2%E4%B8%8E%E8%BF%90%E7%BB%B4/watchdog/t265_monitor.py
https://github.com/realsenseai/librealsense/issues/9660
https://dev.realsenseai.com/download/16547/
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/arch/arm64/boot/dts/hobot/x5-rdk.dtsi
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/phy/phy-sunrise5-usb.c
https://github.com/realsenseai/librealsense/blob/v2.50.0/doc/t265.md
```
