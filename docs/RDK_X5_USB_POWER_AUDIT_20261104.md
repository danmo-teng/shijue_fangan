# RDK X5板载USB供电与驱动核查

2026-11-04，基线`11ac60c`；本轮用户要求追查供电控制能力及驱动。
全程只读硬件，没有复位当前健康T265，没有启动SDK/UART/电机，没有清理日志。

## 结论

1. **当前运行系统没有接通可调用的USB VBUS regulator控制链。** 官方存在相应驱动源码，但本机未编译、DT未启用、无电源映射。
2. **找到一个此前未试过的板级Hub硬件RESET引脚。** 当前板型对应`34120000.gpio`的line13，当前枚举为`gpiochip4`，旧全局号392。
   这是Hub RESET候选，不是已证明的VBUS开关。不能把之前USB协议层Hub reset的失败直接当成该引脚已试过。
3. 普通X5公开原理图仅有接口页，缺少GL3510及5V开关页；尚不能确定RESET/PPON是否控制两组USB 5V输出。
   不将X5 Module底板的EN接线移植到本板，不凭GPIO空闲或名字相似试拉电平。

## 本机身份及实际连接

- `/sys/class/socinfo/board_id=302`，`board_version=1_B`，`hw_name=x5-rdk`，`soc_name=X5U`。
- `hw_info=x5-rdk_1_B_Micron_8192MB_4266`；运行DT model为`D-Robotics RDK X5 V1.0`。
- `/boot/boot.cmd`将board ID `0x0302`映射到`x5-rdk-v1p0.dtb`，与运行DT吻合。
- 内核6.1.83；当前USB3控制器`35100000.usb`为host，USB2闪连控制器`35300000.usb`为device。
- 当前T265在`2-1.1`，8087:0b37/5000Mbps；DHZJ相机在`1-1.3`。两者经同一板载GL3510的USB2/USB3逻辑Hub。
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
**没有申请GPIO、设置方向或读取/写入电平。** 旧全局号来自本次base379+13=392，不能跨板型硬编码。

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

下一项有区别、有代码依据的小实验是**板级Hub RESET脉冲**；优先在再次出现boot态故障时做对照。
它将影响四路USB及同组视觉相机，不能称为只复位T265；不涉及F405或改线，但当前健康相机会被主动中断。
这次仅核查，没有实施或加入启动默认；实际脉冲参数/恢复流程应在决定做该实验时具体确认。
即使恢复了枚举，还需确认原固件真实上传和Pose；VBUS是否掉电仍需电路或测量证据，不能以命令返回代替。

因此本轮可以确定“当前驱动供电链未启用”及“存在尚未验证的Hub RESET候选”，
还不能确定“标准X5硬件绝无软件切电能力”，也不能声称自恢复已经实现。

## 留存与来源

只读原始记录（未触发清理）：

```text
/home/sunrise/RDK_X5/shijue_fangan/rescue_map/runtime/history/t265_f407_motion_map/20261104_124002_rdk_usb_power_audit/
```

包括运行DT、GPIO/pinmux、regulator、内核配置、board_identity、reset_gpio_line_info及result_summary。
本轮仅更新核查文档及当前状态，没有运行代码或驱动修改。

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
```
