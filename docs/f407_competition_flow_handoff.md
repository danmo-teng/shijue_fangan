# 完整智能救援流程的 F407 配合说明

本文是上位机新分支的接口说明，不是F407补丁。F407仓库不得由上位机仓库自动修改或生成补丁。

上位机仍通过定位进程的原子`uart_command.bin`发送固定15字节`TYPE=0x18`帧：

```text
A3 B3 18 SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

`P0`为命令，`P1`为`VALID/RED_SIDE/DRIVE_STRAIGHT/USE_FINAL_HEADING/DISTANCE_VALID`等标志；普通导航继续使用`P2/P3=剩余距离mm`、`P6/P7=绝对航向0.01°`。

## 新命令

| 命令 | 值 | `P2/P3` | `P4/P5` | `P6/P7` |
|---|---:|---|---|---|
| `APPROACH_TARGET` | `0x09` | 目标图像X | 目标图像Y | 0 |
| `HOLD` | `0x0A` | 0 | 0 | 0 |
| `YIELD_BACKOFF` | `0x0B` | 有符号后退距离mm | 0 | 0 |
| `ESCAPE_MANEUVER` | `0x0C` | 有符号旋转角度deg | 有符号横移距离mm | 0 |
| `RELEASE_LEFT` | `0x0D` | 0 | 0 | 0 |
| `RELEASE_RIGHT` | `0x0E` | 0 | 0 | 0 |
| `RELEASE_BOTH` | `0x0F` | 0 | 0 | 0 |
| `DISPERSE_PILE` | `0x10` | 0 | 0 | 0 |
| `CHANGE_LANE` | `0x11` | 有符号横移距离mm | 有符号前进距离mm | 0 |
| `CARGO_AUDIT` | `0x12` | 左爪类别码 | 右爪类别码 | 审核信息 |

`CARGO_AUDIT`类别码：`0=空`、`1=普通`、`2=核心`、`3=伤员`、`4=危险`、`5=未知`、`6=普通+核心混合`。

审核帧中，`CARGO_AUDIT`使用字节级紧凑格式：`P2=左爪类别码`、`P3=右爪类别码`、`P4=左爪数量bits0～1/右爪数量bits2～3`、`P5=审核标志`、`P6=audit_id`、`P7=总数量`。

审核标志为：

- `P5 bit0`临时藏堆，bit1危险存在，bit2未知存在，bit3伤员混装，bit4审核稳定，bit5目的地为伤员区；
- `P2/P3`表示当前左右爪的主要类别，混合普通+核心使用类别码6；
- 下位机只有收到稳定且合法的审核后，才允许接受后续导航或投送命令。

现有命令`GRAB_CONFIRMED=0x02`、`NAVIGATE_WAYPOINT=0x03`、`ENTER_SAFE_ZONE=0x05`、`TASK_COMPLETE=0x06`、`ABORT=0x07`、`RETURN_CENTER=0x08`继续保留。

`HOLD`与旧的`STOP`语义不同：`HOLD`是可恢复停车，不能锁死比赛任务；`ABORT`才是故障后锁存停车。

## 任务硬联锁

F407应在执行层再次拒绝下列情况：

- 首件普通物资不是恰好1件绿色；
- 普通/核心批次超过3件；
- 伤员与其他物资混装，或伤员数量不是1件；
- 危险或未知目标进入安全区投送动作；
- `RELEASE_LEFT/RIGHT`指定的爪子没有对应物资或动作条件不满足；
- 命令帧失联、序号/CRC非法、电机故障、IMU/编码器故障。

下位机只负责实时执行和硬联锁，不需要保存全场目标列表。上位机已经在`competition_detections.jsonl`保存所有识别结果，并通过稳定的track ID决定当前批次。

## 停滞退让与脱困

当上位机在`APPROACH/NAVIGATE/RETURN_CENTER`等应当移动的阶段检测到T265位姿在约1.5秒内位移小于40mm时：

1. 上位机反复发送`YIELD_BACKOFF`，下位机后退默认250mm并保持当前货物约束；
2. 退让结束仍没有位移，上位机发送`ESCAPE_MANEUVER`，默认原地旋转90°并横移160mm；
3. 旋转方向下一次取反，最多执行2次；仍无位移则`ABORT/HOLD`停车并等待人工复位。

下位机需要在`TYPE=0x17`状态的`mode`字段报告动作完成，建议增加以下模式号：

```text
30 YIELD_DONE
31 ESCAPE_DONE
32 RELEASE_LEFT_DONE
33 RELEASE_RIGHT_DONE
34 RELEASE_BOTH_DONE
35 DISPERSE_DONE
36 LANE_DONE
```

退让和脱困期间必须有运动看门狗；如果电流、轮速、碰撞或机构故障已经明确，直接停车，不应继续旋转。

## 验收顺序

1. 架空验证新命令解析、CRC、重复SEQ幂等和`HOLD/ABORT`区别；
2. 空载低速验证`YIELD_BACKOFF`、正负旋转和横移方向；
3. 单物资验证左右爪释放；
4. 普通+核心两件、伤员单件、危险混入四种审核组合验证硬联锁；
5. 最后再做中心拥挤物资、对手避让和完整比赛流程。
