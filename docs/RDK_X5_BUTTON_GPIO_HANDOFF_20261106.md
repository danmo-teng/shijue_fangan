# RDK X5 四按键接线与开机配置交接

更新：2026-11-06。对象是智能救援小车的 RDK X5 上位机；本文记录当前实际接线、按键实测和开机配置，供后续上位机 Codex 接手。按键业务功能尚未分配，本文不授权启动小车或修改下位机。

## 当前硬件及接线

设备系统上报 `board_id=0x302`、型号 `RDK X5 V1.0`，系统另报 `board_version=1_B`。使用 40 针排针的 **BOARD 物理针号**，不是 BCM 编号，也不是 Linux GPIO 编号：

| 按键板端 | RDK 物理脚 | 此次 Linux GPIO | BCM | 说明 |
| --- | ---: | ---: | ---: | --- |
| Vcc | 23 | 395 | 11 | GPIO 输出高，暂时为按键板供电；原为 SPI1 SCLK |
| GND | 25 | — | — | 公共地 |
| 按键 1 | 27 | 355 | 0 | GPIO 输入；可复用 I2C0 SDA / PWM |
| 按键 2 | 29 | 399 | 5 | GPIO 输入；可复用 SPI2 SCLK |
| 按键 3 | 31 | 400 | 6 | GPIO 输入；可复用 SPI2 SSN |
| 按键 4 | 33 | 357 | 13 | GPIO 输入；可复用 I2C1 SDA / PWM |

Linux GPIO 数字是**本机当前编号**。启动脚本按 GPIO 控制器的 label 和偏移计算编号；其他系统或内核版本应重新核对，不要把表中数值当作跨平台固定编号。

当前 UART1（物理 8、10）和 UART6（物理 16、36）在使用，不能因按键项目改变它们。变更接线前应关闭设备并断开所有可能的供电，包括 OTG；不要带电插拔信号线。

**供电待改进：**23 是信号 GPIO，不是专用电源脚；按键板灯曾变暗。软件显示 `out 1` 只代表输出配置和逻辑状态，不能证明带载时实际电压稳定。用户计划之后改接合适的 3.3 V 电源；改线前需断电，完成后重新核实。当前未测出 23 脚的带载电压，也没有证明供电不稳是之前漏检的原因。

## 本次实测

四个键都检测到 `1 → 0 → 1`：松开为高、按下为低。用户发现 1 号键帽太低，第一次没有压到开关；补按后物理 27 脚、GPIO355 和芯片原始输入同步发生变化。27 脚的复用寄存器当时是 GPIO 的 `MUX_ALT1`。这是本次通路可用的证据，不代表之前其他接法中的每个针脚都已通过测试。

后续程序应按低电平有效解释按键。可结合实际按键抖动采用约 20～50 ms 软件去抖；各键对应的业务行为、启动小车的操作方式由用户后续确定，不要自行指定或自动启动电机。

## 已配置的开机行为

1. 已按官方 `srpi-config` 修改设备树同样使用的 `fdtput` 方式，在 `/boot/hobot/x5-rdk-v1p0.dtb` 关闭 `spi1`、`i2c0`、`pwm3`；`pwm2`、`spi2`、`pwm0`、`pwm1`、`i2c1` 本来已关闭。`serial1`、`serial6` 保持 `okay`。这会在**下次启动**取消上述总线或 PWM 功能；如未来要重新使用，须先重新规划复用和接线。
2. 已安装并启用 `rdk-x5-button-gpio.service`。该一次性服务在系统启动时调用 `/usr/local/sbin/rdk-x5-button-gpio-init`，导出所需 GPIO，将物理 23 设为输出高，27、29、31、33 设为输入。当前配置已是目标状态时脚本不重复改写方向或电平。服务结束后 GPIO 保持导出，无常驻轮询和自动关断计时。
3. 实际运行中服务已成功执行，状态为 `enabled`、`active (exited)`；23 当前 `out 1`，四个输入当前均为 `in 1`。运行时 I2C0、SPI1、PWM3 已在此前手动解绑。

**没有重启验证。**设备树文件已检查，服务在当前启动周期执行成功，但下次冷启动后仍需核对各引脚和按键；不可将其写成“重启实测通过”。原设备树备份：`/var/backups/x5-rdk-v1p0.before-button-gpio-20261106T013043.dtb`。系统或设备树升级可能覆盖当前修改，应重新核查。

## 给后续上位机 Codex

- 先读仓库 `AGENTS.md`、`docs/PROJECT_STATUS.md`、`docs/PROJECT_TODO.md` 并核对实际代码，不将此硬件交接直接当作新的比赛行为、协议或运行授权。
- 系统服务负责初始化 GPIO，不负责监听按键、去抖、设置功能或启动任何业务。只监听已有输入时，可以读取其 sysfs `value`；如使用官方 `Hobot.GPIO`，采用 `GPIO.BOARD` 并只配置 27、29、31、33 为输入，避免重新配置物理 23。
- 当前实现使用 **sysfs GPIO**。不要同时用 `libgpiod` 排他申请已由 sysfs 导出的同一条线，除非先统一所有权和启动方案。`Hobot.GPIO.cleanup()` 会取消导出已配置的脚；不要无差别清理或取消导出 23，否则按键板供电会受影响。一次性 systemd 服务不会持续监视并自动恢复被其他程序改动的引脚。
- 按键 1～4 的业务映射、短按或长按行为、与现有 GUI/任务入口的关系，均需以用户之后的具体要求为准；不得凭本文自动改协议、下位机或启动小车。

在 RDK 上可检查：

```sh
systemctl is-enabled rdk-x5-button-gpio.service
systemctl status rdk-x5-button-gpio.service --no-pager
for node in spi1 i2c0 pwm3 spi2 pwm0 pwm1 pwm2 i2c1 serial1 serial6; do
    printf '%s: ' "$node"
    fdtget -t s /boot/hobot/x5-rdk-v1p0.dtb "$node" status
done
for n in 395 355 399 400 357; do
    printf 'gpio%s: ' "$n"
    cat /sys/class/gpio/gpio$n/direction /sys/class/gpio/gpio$n/value | tr '\n' ' '
    printf '\n'
done
```

上面 `fdtget` 显示的是**下次启动使用的文件**，不代表当前运行设备树已被修改；sysfs 表示当前运行状态。如果有人释放或更改了这些脚，可以在确认按键接线及没有其他占用后手动运行 `sudo systemctl restart rdk-x5-button-gpio.service`，然后复核。恢复设备树备份会同时撤销备份后对该文件作过的其他修改，操作前必须先对照。

官方参考：[40PIN 复用和定义](https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/40pin_define)、[GPIO 应用](https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/gpio)、[板卡编号与设备树](https://github.com/D-Robotics/rdk_doc/blob/main/docs/07_Advanced_development/02_linux_development/driver_development_x5/uboot_kernel_config.md)。
