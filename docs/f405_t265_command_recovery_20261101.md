# F405/T265测试命令恢复：1.1兼容与日志写入

本轮基线c4fdc330f52aab44162f966b15de0313e16afb6d，已跟踪工作区最初干净。
只修改测试客户端、日志、现有地图及原回归脚本；正式比赛、定位外参、比例、pose jumping和下位机不改。
用户随后要求只做必要检查，本轮未扩展精度测试或自动运动。

## 故障及源代码审查

上一轮已读取的20261101_164404_interactive身份实际为11eeacf/source_dirty=1、协议1.1、
F405/NormalDebug/MOTIONDEBUG/230400。旧客户端只允许协议1.0和旧提交，
GUI因此在动作提交前返回。该会话没有MOVE/TURN下发，保存的0x1A均为IDLE/fault=0。
另外，日志队列约24.8秒后溢出1698条；前一次16:27会话溢出6659条，形成另一处普通动作阻塞。
本轮开始复核时整个T265历史目录已为空，以上是上一轮原始日志审查结果，不能声称本轮再次核验原文件。
本轮没有清理、恢复、改写那些历史会话。

已只读审查下位机GitHub仓库：

- 512038d2cfe83653d1c3036ad003fc23117df344：时钟异常检查、采样诊断、UART DMA所有权/恢复与质量查询。
- 3f61b7ac38bcad0e2e43e13688e548892b974965：1.1完整合同、JSON、向量及模拟验证资料。
- 11eeacfda71f419834ff78401faf1a0d35a3831e：专用NormalDebugTimed启动/烧录配置。

[1.1合同](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/11eeacfda71f419834ff78401faf1a0d35a3831e/docs/F405_TIMED_CONTRACT_20261101.md)
明确保持旧0x15/0x1F/0x19/0x1A、控制符号、旧payload与序号定义；增加0B/0C质量页及clock-fault有效性。
这些提交可按已审查版本适配；不是允许任意未来版本，也不是对source_dirty二进制作签名认证。

## 修改文件与接口

- `localization/tools/f405_test_client.py`：提交与协议版本配对检查；1.1要求质量查询能力位；
  actual保持原值，document/expected选择对应合同。1.1的epoch_valid另外要求clock_flags.bit0，
  未知漂移/采集误差不因此合格。失败前先安排下次查询，日志不可用时不继续生成每次poll的重试记录。
- `localization/tools/async_debug_log.py`：4096条原始队列容量不变；批量序列化并按文件写入，
  100ms刷新；独立有界最新快照槽按文件合并，最多10Hz原子替换。原始历史时间与记录不合并/降采样。
  返回队列峰值、原始写入数、快照合并/写入数和丢弃统计。真实丢弃仍明确标记不完整，不绕过日志检查。
- `localization/tools/t265_f407_motion_map.py`：身份/未决动作导致按钮请求提前拒绝时，
  保存requested_test、参数、具体reason和uart_motion_write_attempted=false。
  界面显示“身份已确认”，避免把身份通过说成全部动作或精度通过。
  原sensor-check延长到最多60秒，复用地图渲染但不打开可点击窗口，不处理运动按键。
- `localization/tools/f405_connection_check.py`：结果额外保留关闭后的实际身份/链路快照。
- `localization/tools/test_f407_motion_protocol.py`：在既有脚本内检查1.1版本配对、未知版本/提交拒绝、
  clock-fault时间无资格、按钮拒绝事件、日志失败退避，以及批量原始日志无损/最新快照合并。

没有实现新的0B/0C周期查询。本轮解决普通测试命令资格与日志，未假装补齐MCU漂移、后轮EXTI和IMU源延迟界。
无新控制TYPE/opcode、无外部位姿加权融合、无永久STOP锁存或自动动作重放。

## 必要验证结果

原Python协议/客户端回归通过。C++、外参和比赛代码未改，本轮没有重复C++构建或实车精度测试。
一次真实持续检查复用原入口：

```bash
python3 localization/tools/t265_f407_motion_map.py --wheel-input off --sensor-check-seconds 45 --baud 230400 --chassis-profile timed --no-log-prune
```

RDK日志：`rescue_map/runtime/history/t265_f407_motion_map/20261101_165719_interactive/`。

- actual=11eeacfda71f419834ff78401faf1a0d35a3831e、source_dirty=1、协议1.1、NormalDebug、230400；身份ready=true。
- 同一SDK进程，45秒、64次既有地图渲染；不含真实窗口事件处理，也不能据此保证任意负载下的实时性。
- 23次身份请求均有written和匹配reply；查询超时、记录到的混合CRC/解析错误均为0。
- 结束前队列峰387/4096，丢弃0，error=null，历史写入23023条（关闭前快照，随后仍有正常收尾数据）；
  快照写入288次、合并486次。关闭后没有logging_events错误文件，会话complete=true。
- 8794个不同SDK帧，约199.57Hz，回调队列丢弃0。采样连续性不代表精度。
- 没有commands/chassis_actions运动文件，只有身份查询；MOVE/TURN/STOP/reset/订阅均为0。
- wheel_input_requested=false；时间/SDK资格仍false，无SDK速度接受样本。不是轮速B组，也没有验证电机动作。

接口已能认可当前实际固件，日志持续检查未复现原溢出；下一次由用户通过原桌面入口主动触发动作，
再核对发送与匹配回执。轮速注入仍等待可靠时间界，T265原生视觉退化和独立地面精度验证仍未完成。
