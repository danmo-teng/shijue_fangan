# RDK X5 <-> F407 定位协议

电气和外层协议与 `F407-Rescue-Robot` 一致：USART3，115200 8N1，固定 15 字节，递增序号，Modbus CRC-16。RDK X5 的 `/dev/ttyS1` TX（40Pin 物理 8 脚）接 F407 PD9/RX，RDK X5 RX（物理 10 脚）接 F407 PD8/TX，必须共地且两端都是 3.3 V TTL。

```text
索引:  0  1   2    3    4  5    6  7    8  9   10   11     12     13   14
数据: A3 B3  15   SEQ  M1_BE   M2_BE   M3_BE   DT  STATUS CRC_LO CRC_HI C3
```

- `M1/M2/M3`：三路 `EncoderStatus.position` 的低 16 位，原始编码器符号，大端。主机用 16 位模减法处理回绕，再应用配置中的 `encoder_sign_m1..m3`。
- `DT`：底层编码器采样周期，单位 ms，当前应为 10。
- `STATUS bit0..2`：M1/M2/M3 有效；正常帧为 `0x07`。
- `STATUS bit3`：计数器或软件累计值刚被清零，主机只重建基线，不融合该帧。
- `STATUS bit4`：编码器故障，主机拒绝轮式更新。
- CRC 覆盖 `TYPE,SEQ,P0..P7` 共 10 字节，初值 `FFFF`，多项式 `A001`，低字节先发。

发送低 16 位累计位置而不是单帧增量，这样 UART 丢一两帧时不会丢失行驶距离。`SEQ` 的差值用于累加 `DT`；只要两个成功帧之间单轮增量未超过 32767 计数，回绕差值仍唯一。

F407 端可复用 `firmware/f407_odom_protocol.c` 打包，建议每 10 ms 在主循环/发送队列中插入一帧。不要在 TIM6 ISR 中阻塞发送；现有视觉 ACK 与里程计帧共用 USART3 TX 时，必须经过同一个非阻塞 TX 队列。

## RDK -> F407：`TYPE=0x16` 融合场地位姿

当前任务默认关闭该帧（`--tx-rate 0`）。下面只保留旧版兼容格式；显式设置非零发送频率时才回传：

```text
索引:  0  1   2    3    4  5   6  7   8  9    10      11       12     13   14
数据: A3 B3  16   SEQ   X_BE   Y_BE   YAW_BE STATUS CONF_SIG CRC_LO CRC_HI C3
```

- `X/Y`：`int16`，场地坐标 mm；中心为原点，+X 向图纸右，+Y 向上。
- `YAW`：`uint16`，单位 0.01°，范围 `0..35999`，从 +X 逆时针增加。
- `STATUS bit0 VALID`：位姿可用。F407 只能在该位为1且帧未超时时执行位置闭环。
- `bit1 T265_GOOD`：tracker confidence 至少为2。
- `bit2 WHEEL_ACTIVE`：本时刻编码器正在参与融合。
- `bit3 OBSTACLE_GATE`：处于起步/角落减速带门控区，编码器被禁用。
- `bit4 ODOM_FRESH`：RDK 收到的 F407 里程计未超时。
- `bit5 INSIDE_FIELD`：融合中心在±1.5 m 场地边界内。
- `bit6 T265_UPDATE_REJECTED`：本次 T265 更新因跃变门限被拒绝。
- `CONF_SIG bit0..1`：tracker confidence；`bit2..3`：mapper confidence；`bit4..7`：位置 1σ 的厘米整数，15表示≥15 cm。

F407 接收端必须设置独立看门狗，建议 150 ms。重复 `SEQ` 不得刷新时间戳；超时、CRC 错误或 `VALID=0` 立即停止使用上位机位置。`firmware/f407_fused_pose.[ch]` 提供载荷解码和新鲜度判断。

位姿失效造成的停车是可恢复等待，不应等同永久故障：位姿恢复后，如果任务NAV方向仍在
250 ms新鲜窗口内可直接继续，否则等待新的NAV。任务方向帧的分级超时、临时STOP与ABORT
语义见根目录`docs/f407_uart_integration_guide.md`。

当前 F407 `Vision_ParseBytes()` 已经是 USART3 的唯一公共字节流解析器，不要再并行启动第二个字节定界状态机。应在现有解析器中把 `0x16` 加入合法 TYPE，CRC 通过后将 `P0..P7` 交给 `F407_FusedPoseDecodePayload()`。

## 共享串口任务扩展

普通物资任务增加`TYPE=0x17` STM32状态和`TYPE=0x18` RDK任务命令，字段定义见仓库根目录`docs/f407_uart_integration_guide.md`。定位程序使用：

```bash
./run_localization.sh \
  --uart /dev/ttyS1 \
  --command-file ../rescue_map/runtime/uart_command.bin \
  --stm-status ../rescue_map/runtime/stm32_status.json
```

命令文件必须恰好15字节，只允许`TYPE=0x11/0x12/0x18`且CRC正确；相同内容只转发一次。STM32状态JSON由合法`TYPE=0x17`生成。这样视觉任务与定位融合不会同时打开同一个UART。
