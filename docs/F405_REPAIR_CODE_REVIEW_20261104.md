# F405 1.3修复的代码行为审查

这是适配前的审查记录。后续RDK 1.3测试兼容已实施且实际读回新固件，当前结果见[f405_v13_test_adapter_20261104.md](f405_v13_test_adapter_20261104.md)，不要沿用本文历史“尚未适配/未烧录”作为当前状态。

2026-11-04。输入来自用户指定的/home/sunrise/下载/transfer.txt，其中只有下位机回执链接。
审查对象：下位机4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96；对比11eeacf，含6个提交。
实现最后修改在0a2dbf7f523cba577c6ccacc3db5fae31b8dae48，4ac1d3d为后续文档提交。
RDK审查前HEAD=dd61e49。本轮只审查/必要离线检查，没有修改上下位机运行代码、烧录或发送运动。
回执中的“用户确认”不自动转成RDK新的实施授权或长期约束，本轮核查的是代码实际做了什么。

## 结论

运行时年龄误停和附加时间诊断确实已按回传落实到源码；共享UART驱动的现有主机回归在RDK复跑通过。
不能称整条链已可用：新固件未烧录，RDK还拒绝1.3及部分合法恢复数据，Location另有恢复后大角度差被舍弃的遗留行为。

## 已落实的具体行为

| 条件 | 当前源码行为 | 依据 |
|---|---|---|
| 从未初始化过合法反馈 | ready=false，不能伪造初始零测量；晚到合法帧可完成初始化 | imu_uart.c:IMU_Init、receive_frame、lose_stream |
| 单纯时间流逝、没有新包 | 不因150ms年龄清ready，不因300ms设置heading_uncertain；源计数不增长 | imu_uart.c:IMU_Update、receive_frame |
| 显式ORE/FE/NE/PE | lose_stream令ready=ever_ready、recovering=true；已有yaw/rate保留；随后3个合法角度样本清恢复标志 | imu_uart.c:lose_stream、IMU_UART_IRQHandler |
| RECOVERING期间已有初始化反馈 | Location/Debug/motor/Task原ready判断不会单因该恢复标志而终止；原控制目标和轮速闭环继续运行 | driver新语义与各层判断链；Task/Location本次未改 |
| 持续静默 | 使用最后合法值，没有新失联暂停/取消阈值；原动作完成、动作超时和人工STOP仍有效 | 不能将其描述为只允许短暂静默，也不是冻结一次PWM；连续业务未统一加有限超时 |
| 07轮速度 | 正dt、合法计数变化、无数值溢出时可VALID；未知时间界、EXTI_PENDING、epoch未知不单独清VALID | DebugTelemetry.c:WheelSample |
| 08 gyro | 源值存在且可表示即可VALID，RECOVERING/年龄/ready不再决定数据VALID；无新源不发新08 | DebugTelemetry.c:gyro_flags、publish_gyro |
| 0x1F | 布局及CCW方向未改；UART下已有合法可表示yaw/rate即可VALID；READY和RECOVERING可同时置位，age仍如实饱和 | Robot.c:IMU report |

持续无反馈可能漏整圈；保留旧反馈不能恢复未收到的角度，不能称为精度保证。
保留的MOVE/TURN等原动作超时仍可报告FAULT，此次并非删除所有FAULT来源。

源码已删除后轮逐沿计时、IMU逐字节附加取时、时钟寄存器/回退巡检、8ms队列过期、100ms TX年龄巡检、
若干100ms gyro源年龄退出、重复RAM指纹和高频时间统计。TIM5基本计数、实际速度区间、整帧源ID保留。
UART仍由主循环唯一启动DMA；先复制到持久缓冲区再发送，活动队列项不覆盖；显式DMA错误仍走abort/硬件排空。
未知开销没有伪报为已优化多少毫秒：本轮未测板端IRQ负载。

## RDK当前不兼容，已用现有函数离线复现

输入为明确标注的1.3协议合成样本，只调用当前Python解析/转换函数；没有创建UART客户端、打开SDK或注入速度。

| 检查 | RDK当前结果 | 应按1.3区分的语义 |
|---|---|---|
| 身份protocol=1.3，实际实现SHA=0a2dbf7 | 拒绝：supported 1.0/1.1 | 审查后添加版本/实现映射，不能只改expected文字 |
| 07 flags=0x81 | measurement_valid=true | 正常对照 |
| 07 flags=0x85（新增EXTI_PENDING描述位） | measurement_valid=false | 下位机1.3不再由此清VALID，不能沿用旧无效掩码 |
| 08 rate flags=0x343 | wire_valid=true | 正常对照 |
| 08 rate flags=0x347（新增RECOVERING） | wire_valid=false | 新合法源在恢复中仍是有效测量，不应仅因恢复标志舍弃 |
| 0x1F status=0x03、age=1 | wire_valid=true | 正常对照 |
| 0x1F status=0x07、age=1 | wire_valid=false | 1.3允许VALID＋READY＋RECOVERING |
| 0x1F status=0x03、age=255 | wire_valid=false | 年龄长不单独否定已有合法源值；仍不能伪造新样本 |

具体位置：
- localization/tools/f405_test_client.py:245，版本白名单/提交映射仍到1.1；capabilities和metadata合同指向也需同步。
- localization/tools/f405_timed_measurements.py:81、113，轮与gyro无效位掩码仍是旧语义；epoch描述与数据可用性须分开说明。
- localization/tools/f407_motion_protocol.py:216，parse_imu硬要求status==0x03且age<150。
- localization/tools/lidar_t265_motion_map.py:111，主yaw仍丢弃RECOVERING标志样本。

这是上位机需要做的最小1.3适配，不应再要求下位机改回旧时间资格语义。
新合同09/0B查询不支持（7F/status2），capabilities=0x3DF；不能要求质量页、假报标定或将配置存在当SDK接受。
该适配尚未实施；固定安装参数、单位、F405运动学、TURN转换及唯一UART/SDK所有权不应顺手改变。

## 下位机仍需明确的一处行为：恢复后的累计yaw差

Main/Src/Location.c:185–191未随本次修改：先更新previous_imu_yaw，再把绝对值大于APP_LOCATION_MAX_YAW_STEP_MDEG（30000）的增量置0；
同函数末尾仍将location.valid=true。

源码推演例子：已建立参考后，IMU从0°恢复到40°，Location丢弃40°；下一帧41°只累计1°。
该段角度不会在后续正常帧中自动补回。0x1F/08可已报告40°，Location本地航向却还在0°附近。
这不会直接引起同一次ODOM3，却可能使继续执行的动作使用偏差航向。此条件尚无本次板上复现，不将其写为原现场FAULT的已证根因。

建议接收方只核对此处：明确的归零/参考变更沿现有路径处理；合法累计yaw恢复差不能被当成归零静默丢失。
不建议加新年龄门或简单扩大阈值，也不在RDK窗口代改下位机。若选择保留某种行为，须说明对MOVE/TURN/正式Task的实际影响。
这项是局部遗留检查，不扩大成新诊断、协议或故障框架。

## 实际检查、版本与未完成项

- 对6个提交的实际diff及当前IMU/Location/Debug/motor/Task/UART/遥测路径作只读审查；Task业务、PID/速度和构建预设未在该diff中改变。
- 复用下位机现有tools/test_imu_uart.c及其mock头文件，在RDK临时目录编译执行：PASS。
  检查包括RX错误/长间隔保留ready、不伪造样本、恢复、回绕、晚初始化。它是模拟USART的共享驱动主机测试，不是F405硬件或电机验证。
- 上表RDK兼容性检查实际执行；没有新增仓库测试程序，没有运行全套测试。
- 下位机文档报告NormalDebugTimed和NormalDebugTask构建通过，RDK本轮未重跑这两个固件构建。
- 下位机规则已在AGENTS.md持久化，GitHub规则提交ca87bfa可核对；不能再称其尚未写入。
- 下位机回执明确未烧录。构建身份为0a2dbf7/source_dirty=1（其说明差异仅Task.c空白）；这不是板端actual。
  板端actual目前仍只有旧会话11eeacf/dirty1记录；本轮未查询设备。
- NormalDebugTask仍无07/08，默认115200；正式比赛定位接口仍是下一项独立适配，不能称本次修复同时接通比赛遥测。

下一小步：RDK测试路径适配已审查1.3；接收方核对上述Location恢复角度处理并回传；之后根据明确烧录/实际身份做必要短实测。
左横移清障仍为用户希望保留的效率候选，圆弧/斜坡误推仅为待实测问题，不作为直接否决依据；与本次误停修复分开。

源码及回传（完整URL，方便复制）：

```text
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96/docs/F405_REPAIR_20261104.md
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96/Main/Src/imu_uart.c
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96/Main/Src/Location.c#L185
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96/Main/Src/DebugTelemetry.c
```
