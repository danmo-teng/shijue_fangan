# 发给电控负责人的 RDK X5 ↔ STM32F407 UART 联调说明

## 1. 本阶段目标

使用F407现有的`USART3`与RDK X5进行全双工通信：

- RDK向F407发送普通物资的图像中心，供底盘方向及摄像头舵机闭环使用；
- F407以100 Hz向RDK回传三路编码器累计计数，供T265与轮式里程计调试；
- 当前视觉端不根据目标面积发送停车信号，`NEAR`始终为0；
- 通信中断必须保留安全看门狗，但通信调试阶段可只显示数据、不驱动电机。

## 2. 接线与串口参数

```text
RDK X5 40Pin 物理8脚  UART1_TX  --->  F407 PD9  USART3_RX
RDK X5 40Pin 物理10脚 UART1_RX  <---  F407 PD8  USART3_TX
RDK X5 GND                         ---  F407 GND
```

- 两端均为3.3 V TTL，禁止连接RS-232电平；
- 必须共地，TX和RX交叉连接；
- 串口参数固定为`115200 baud, 8 data bits, no parity, 1 stop bit`；
- RDK设备节点为`/dev/ttyS1`。

## 3. 公共15字节帧

除原有4字节配置ACK外，双向业务消息统一使用固定15字节帧：

```text
索引:  0  1   2    3   4  5  6  7  8  9 10 11   12     13    14
数据: A3 B3 TYPE  SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

- 帧头：`A3 B3`；帧尾：`C3`；
- `TYPE`高4位为协议版本1，低4位为消息编号；
- `SEQ`为0～255循环递增序号；
- `P0..P7`是固定8字节载荷；
- CRC覆盖`TYPE、SEQ、P0..P7`共10字节；
- CRC算法为CRC-16/Modbus：初值`0xFFFF`、多项式`0xA001`、低字节先发送；
- CRC错误、非法TYPE、错误帧尾不得更新控制数据或看门狗时间。

解析器必须支持任意拆包、粘包、噪声后重新寻找`A3 B3`，不能假设一次DMA回调就是一整帧。

## 4. RDK → F407：普通物资视觉报告 `TYPE=0x12`

RDK每30～50 ms发送一帧：

| 载荷 | 内容 | 字节序 |
|---|---|---|
| `P0 P1` | 目标中心X，范围0～639 | 大端 |
| `P2 P3` | 目标中心Y，范围0～479 | 大端 |
| `P4 P5` | 可选前向距离；当前阶段填0 | 大端 |
| `P6` | 四类数量，每类2 bit | 位打包 |
| `P7` | 识别状态 | 位标志 |

普通物资单目标识别成功时：

```text
P6 = 0x01                         普通物资数量为1
P7 = 0x09                         FOUND | CLASS_VALID
P7 bit1 NEAR = 0                  当前阶段始终不发面积停车信号
P7 bit6 DISTANCE_VALID = 0        当前阶段不提供距离
```

没有可靠目标或目标丢失时，`P0..P7`全部填0。`FOUND=1`时坐标必须在640×480范围内。当前阶段`P4=P5=0`，F407只能使用X/Y，不得执行距离减速、距离停车或夹取。以后完成距离标定后置`P7 bit6 DISTANCE_VALID=1`，此时距离必须为`1..65535 mm`。视觉报告重复`SEQ`不能刷新时间戳，超过250 ms未收到新合法报告时停止使用旧目标。

已知校验帧：目标`(320,240)`、距离无效、1个普通物资、序号`0x10`：

```text
A3 B3 12 10 01 40 00 F0 00 00 01 09 1C 13 C3
```

电控代码需要新增`distance_valid`字段，并修改`vision_save_report()`：清除`DISTANCE_VALID`时只允许距离为0，但仍接受`FOUND=1`；置位时才要求距离非零。`Crab_Object()`只有在`distance_valid=true`时才允许执行距离分段减速、距离停车或夹取。当前阶段建议固定低速前进，只用X居中和Y调整舵机；首次通信测试应只在LCD显示快照，不驱动电机。

## 5. F407 → RDK：三路编码器 `TYPE=0x15`

F407当前上游代码的PD8只发送赛前配置ACK，还需要新增编码器上报。建议每次10 ms编码器采样完成后生成一帧，即100 Hz：

```text
索引:  0  1   2    3    4  5    6  7    8  9   10   11     12     13   14
数据: A3 B3  15   SEQ  M1_BE   M2_BE   M3_BE   DT  STATUS CRC_LO CRC_HI C3
```

- `M1/M2/M3`：`EncoderStatus.position`累计计数的低16位，保持编码器原始符号，大端发送；
- RDK通过16位模减法处理回绕，因此不要发送单帧增量；
- `DT`：编码器采样周期，当前填`10`，单位ms；
- `STATUS bit0..2`：M1/M2/M3有效，正常填`0x07`；
- `STATUS bit3`：累计计数刚复位，本帧只建立新基线；
- `STATUS bit4`：编码器故障，RDK拒绝融合本帧；
- `SEQ`每次采样加1，`0xFF`后回到`0x00`。

已知校验帧：`SEQ=0`、`M1=0x1234`、`M2=0x5678`、`M3=0x9ABC`、`DT=10`、`STATUS=0x07`：

```text
A3 B3 15 00 12 34 56 78 9A BC 0A 07 90 F6 C3
```

可直接移植本工程提供的`f407_odom_protocol.h/.c`。采样任务中的调用结构应类似：

```c
static uint8_t odom_sequence;

void Odom_Publish10ms(void)
{
    EncoderStatus encoder[3];
    uint8_t frame[F407_ODOM_FRAME_SIZE];
    F407OdomPayload payload;

    Encoder_GetAll(encoder);
    payload.position[0] = (uint16_t)encoder[0].position;
    payload.position[1] = (uint16_t)encoder[1].position;
    payload.position[2] = (uint16_t)encoder[2].position;
    payload.sample_period_ms = 10U;
    payload.status = F407_ODOM_M1_VALID |
                     F407_ODOM_M2_VALID |
                     F407_ODOM_M3_VALID;
    payload.sequence = odom_sequence++;
    F407_OdomBuildFrame(&payload, frame);
    USART3_TxEnqueue(frame, sizeof(frame));
}
```

`USART3_TxEnqueue()`必须把15字节复制到静态环形队列，使用DMA或中断逐帧发送，不能保存指向上述栈数组的指针。

## 6. USART3发送侧必须统一排队

原有4字节配置ACK与新增15字节编码器帧共用PD8 TX，二者必须进入同一个非阻塞发送队列：

- 禁止在TIM6中断中阻塞调用`HAL_UART_Transmit()`；
- 禁止ACK和编码器分别调用两个互不协调的`HAL_UART_Transmit_IT/DMA()`；
- 一个完整帧发送结束后才能启动下一帧，不能发生字节交错；
- 队列满时优先保留配置ACK并丢弃最旧编码器帧，同时累计丢帧计数；
- 100 Hz编码器帧只占约15 kbit/s，115200 baud带宽足够。

推荐流程：TIM6完成编码器采样后只设置发布标志，由主循环或PendSV读取同一周期的三路原子快照、打包并入队。

## 7. 后续可选：RDK → F407融合位姿 `TYPE=0x16`

视觉闭环和编码器联调完成后，再接收T265融合场地位姿：

```text
A3 B3 16 SEQ X_BE Y_BE YAW_BE STATUS CONF_SIG CRC_LO CRC_HI C3
```

- `X/Y`：有符号`int16`场地坐标，单位mm；
- `YAW`：`0..35999`，单位0.01°；
- `STATUS bit0`为位姿有效，`bit1`为T265质量良好；
- F407使用独立150 ms看门狗，重复序号不刷新；
- 需要把现有`Vision_ParseBytes()`合法TYPE范围加入`0x16`，CRC通过后调用`F407_FusedPoseDecodePayload()`；
- `TYPE=0x15`是F407发出的编码器帧，不需要加入F407接收解析器。

## 8. 电控侧交付与验收清单

1. 示波器或逻辑分析仪确认PD8输出为115200 8N1、3.3 V TTL。
2. F407上电后以100 Hz连续发送合法`TYPE=0x15`，静止时计数不变化、转轮时对应通道变化。
3. 三轮分别正转，记录计数方向，符号修正在RDK配置中统一完成。
4. 注入上述两条已知帧，CRC结果必须完全一致。
5. 验证DMA任意分段、粘连两帧、CRC错误和丢1字节后接收解析器能够恢复。
6. ACK与编码器同时发送时不得发生帧字节交错。
7. RDK停止发送`TYPE=0x12`超过250 ms后，F407不得继续沿用旧视觉坐标。
8. 通信测试模式确认无误后，再允许视觉快照进入底盘和舵机闭环。
