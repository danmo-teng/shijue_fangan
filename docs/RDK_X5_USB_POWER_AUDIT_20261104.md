# RDK X5板载USB供电与驱动核查

2026-11-04；只读核查基线`11ac60c`，随后复位实验基线`764b2c1`。
用户随后明确批准四路USB一起复位，要求验证启动自恢复。本页以下保留驱动核查，并更新实际实验结果。
**最新结果：板级硬复位及组合方法仍未恢复T265；设备当前处于boot故障态，未实现开机自动恢复。**

## 结论

1. **当前运行系统没有接通可调用的USB VBUS regulator控制链。** 官方存在相应驱动源码，但本机未编译、DT未启用、无电源映射。
2. **板级Hub硬件RESET现已实际测试。** 当前板型对应`34120000.gpio`的line13，当前枚举为`gpiochip4`，旧全局号392。
   GPIO高→低→高实际读回，Hub确实重新枚举；1ms/100ms/5s脉冲均未恢复T265。它不是已验证有效的VBUS开关。
3. 普通X5公开原理图仅有接口页，缺少GL3510及5V开关页；尚不能确定RESET/PPON是否控制两组USB 5V输出。
   不将X5 Module底板的EN接线移植到本板，不凭GPIO空闲或名字相似试拉电平。

## 本机身份及实际连接

- `/sys/class/socinfo/board_id=302`，`board_version=1_B`，`hw_name=x5-rdk`，`soc_name=X5U`。
- `hw_info=x5-rdk_1_B_Micron_8192MB_4266`；运行DT model为`D-Robotics RDK X5 V1.0`。
- `/boot/boot.cmd`将board ID `0x0302`映射到`x5-rdk-v1p0.dtb`，与运行DT吻合。
- 内核6.1.83；当前USB3控制器`35100000.usb`为host，USB2闪连控制器`35300000.usb`为device。
- 核查时T265在`2-1.1`，8087:0b37/5000Mbps；实验后已变为`1-1.1`，03e7:2150/480Mbps故障态。DHZJ相机在`1-1.3`。两者经同一板载GL3510的USB2/USB3逻辑Hub。
  当前组内未见USB存储/网卡。F405使用独立UART，本轮未打开。

## 各层控制究竟做什么

| 层级 | 源码/接口 | 核查结果 |
| --- | --- | --- |
| 外置逻辑Hub端口电源 | `usb/core/hub.c:usb_hub_set_port_power()` | 向Hub发送SET/CLEAR_FEATURE(PORT_POWER)，返回成功只能证明协议处理；能否切5V取决于GL3510的电源输出及板级接线 |
| xHCI根端口 | `usb/host/xhci-hub.c:xhci_set_port_power()` | 改PORTSC的PORT_POWER位，另有ACPI可选路径；未见本板GPIO供电使能动作，不能据此证明插座5V已断开 |
| DWC3角色切换 | `usb/dwc3/core.c`、`drd.c` | 仅在存在`usb2_phy`时调用`otg_set_vbus()`；控制器重置/时钟操作不等于下游设备掉电 |
| Sunrise5专用VBUS | `usb/phy/phy-sunrise5-usb.c` | `devm_regulator_get(dev,"vbus")`，`set_vbus()`调用regulator_enable/disable；它本身不包含本板5V使能GPIO号 |
| extcon角色检测 | `extcon/extcon-usb-gpio.c` | `id`和`vbus`按GPIOD_IN取得；是输入检测，不是供电输出。本机仅配置AON GPIO5/6作为ID |
| 板级Hub RESET | pinctrl＋gpio-dwapb | 独立于上述USB复位命令，DT在GPIO控制器默认状态中配置输出高；目前没有自动复位序列 |

### 为什么源码有VBUS函数，本机仍没有供电开关接口

实际`/proc/config.gz`：`# CONFIG_USB_SUNRISE5_PHY is not set`。
运行DT的`usb3_u2phy`、`usb2phy`均为disabled；全树未发现`vbus-supply`。
`/sys/kernel/debug/regulator/regulator_summary`只有dummy、ADC、BPU、CPU和GPU，未见USB VBUS电源。
本机也没有`sunrise5_usb_phy`绑定实例。

所以“把节点改okay”或“编译这个驱动”还不够：必须先证明板级开关及其EN连接，并提供真实regulator。
不能用dummy regulator返回成功当成切电，也不能把Hub RESET冒充`vbus-supply`。

## 新找到的Hub RESET引脚证据

安装文件`/boot/hobot/x5-rdk-v1p0.dts`和官方同名文件均明确：

```text
LSIO_UART4_TX  LSIO_PINMUX_3 BIT_OFFSET26 MUX_ALT2 &pconf_output_high
&ls_gpio0_porta: pinctrl-0 = <&lsio_gpio0_13>; // USB HUB RESET high
```

运行DT也引用该组，phandle解析指向`pconf-output-high`；实际pinmux：

```text
pin 13 (lsio_uart4_tx): gpiochip4 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_13
```

通过只读`GPIO_GET_CHIPINFO_IOCTL`和`GPIO_GET_LINEINFO_IOCTL`核对：
`gpiochip4`标签34120000.gpio、32线，line13 flags=0x2（输出）、consumer为空。
只读核查阶段没有申请GPIO、设置方向或读取/写入电平；随后获准实验已执行电平操作，见下节。
旧全局号来自本次base379+13=392，不能跨板型硬编码。

驱动调用链：`pinctrl/hobot/common.c`的`PIN_CONFIG_OUTPUT` → `horizon_gpio_set_direction()` →
`gpiod_direction_output()`，底层为已绑定的`gpio-dwapb`；当前默认配置不构成复位脉冲。

还核对了一个容易误读的输出：pinconf-pins显示`pin output (0 level)`，但该版common.c的GET分支
返回的是`gpiod_get_direction()`，并非实际输出电平。因此本轮没有据此断言RESET被拉低，也没有“纠正”电平。

其他板型不同：旧`x5-rdk.dts`使用DSP GPIO22，Module使用LSIO GPIO12；不能互抄编号。

## 电路证据边界及下一步

官方普通X5公开文件`RDK_X5_IO_CONN_PUBLIC_V1.1.pdf`只有一页，列出
`USB3_VBUS_HOST12_5V`和`USB3_VBUS_HOST34_5V`，没有呈现其开关EN如何连接。
官方下载目录及本机相关资料路径中未找到普通X5的完整Hub/电源页。
Module完整底板图属于另一硬件版本，不能填补这一缺口。

用户已批准四路复位；下列实验影响四路USB及同组视觉相机，不涉及F405或改线。
只实施有接口/源码依据的方法；没有猜测其他GPIO、写未知vendor请求、改Hub EEPROM或盲目启用未映射的regulator。

## 获准后的实际复位实验

原始会话：`rescue_map/runtime/history/t265_f407_motion_map/20261104_130809_usb_hub_gpio_recovery/`。

| 新方法 | 实际结果 |
| --- | --- |
| GPIO13低脉冲100ms | 读回1→0→1，实测低100.267ms；两个Hub及视觉相机重枚举。T265从健康runtime变成USB1.10/64字节端点的boot故障态，固件0字节 |
| GPIO13低脉冲5s | 低/高读回成功，实测低5.004s；固件仍0字节 |
| GPIO13保持低，重绑定外层DWC3，等根Hub就绪，再释放板级Hub | 释放前确认根Hub存在、角色host，实际低266.566ms；固件仍0字节 |
| 解绑两个Hub类驱动，保持Hub本身枚举，对全部下游端口off，5秒后on并恢复驱动 | 两次间隔读回均保持全端口off，排除本次等待期间内核自动重新设置power位的解释；之后仍0字节 |
| GPIO13低脉冲1ms | 实际低1.132ms，固件仍0字节；不是只试了较长脉冲 |
| 清除两个bulk端点状态/数据toggle | claim及两个clear_halt均返回0；随后上传仍0字节 |
| 直接用官方SDK原生tm_boot查询 | 原有t265_debug，SDK2.50.0报告两次Error booting T265，列出0个T200设备；不是只有自写loader失败 |

所有六次现有loader探测均报告0/9,323,648字节、1秒传输超时；真实Pose为0。
前三种不同脉冲不是新增默认等待；上游GL3510驱动补丁提到200μs复位，这里用1ms作较短软件脉冲对照，
100ms/5s作持续复位对照，并分别记录实际耗时，不称其中任一值为本板标定最优值。

结束时GPIO已恢复高并释放handle、仍为输出；Hub类驱动和DWC3已绑定，角色host，全部端口power，
视觉相机重新枚举、USB无进程占用。**恢复这些设置没有恢复T265本身。**
没有启动电机/UART、没有清理日志、没有主机重启；当前已具备与此前相同表征的故障态，恢复方法全失败，未进入开机回归。

此前单设备reset、uhubctl单口/全口/根端口/Hub reset、角色切换、控制器重绑定、挂起唤醒、
旧枚举顺序、configuration 0→1、小块上传及USB3隔离的原始失败记录仍保留，未把相同方法无理由重复执行。
这覆盖了目前查明且适用于本板的复位途径，不代表穷尽所有未来可能的内核/电路方案。

**未安装自动复位链。** 将这些已失败步骤串进启动会增加启动耗时，还可能把健康设备变成故障态。
“忘记插拔也确保启动”仍未达成；下一缺项是普通X5 1_B的Hub/5V开关EN接线或其他确证可用的复位接口。
未测VBUS波形，不能以port off读回或固件丢失单独证明实际掉电，也没有宣称所有纯软件办法在物理上均不可能。

## 留存与来源

只读原始记录（未触发清理）：

```text
/home/sunrise/RDK_X5/shijue_fangan/rescue_map/runtime/history/t265_f407_motion_map/20261104_124002_rdk_usb_power_audit/
```

包括运行DT、GPIO/pinmux、regulator、内核配置、board_identity、reset_gpio_line_info及result_summary。
获准实验记录另存130809会话；本轮仅更新证据及当前状态，没有运行代码/驱动修改，没有把失败动作变成自动启动功能。

官方源码版本及完整URL：

```text
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/arch/arm64/boot/dts/hobot/x5-rdk-v1p0.dts
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/pinctrl/hobot/common.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/phy/phy-sunrise5-usb.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/dwc3/core.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/dwc3/drd.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/core/hub.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/usb/host/xhci-hub.c
https://github.com/D-Robotics/x5-kernel/blob/b2edb450c6eaef1c7277885d49f74efb6088b634/drivers/extcon/extcon-usb-gpio.c
https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/hardware
https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_IO_CONN_PUBLIC_V1.1.pdf
https://lkml.rescloud.iu.edu/2310.1/02406.html
```
