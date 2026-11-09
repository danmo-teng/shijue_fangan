# 2026-11-09 延迟与到角深化分析

入口：[完整分析](ANALYSIS.md)。本轮只离线分析，MotionDebug是受控实验依据、两场正式比赛是联合负载参考。

- turn_angles.csv：全部34转动，含32 DONE、2 FAULT；应用角度逆时针正，单位分别标列名。缺值留空，不补造SDK或地面真值。
- angle_summary.json：终点/过冲/累积角误差、gyro速率积分与累计yaw差。
- motion_timings.json：11个运动会话，all为动作包络或整个Pose会话，motion为真实已发动作时间并集；详见完整分析。
- competition_timings.json：两场真实开赛区间SDK/Pose及雷达统计；control.measured=false明确正式逐包实发未测。
- pose_consumer_stalls.json：投影前后到CSV时戳的作用域、SDK捕获/回调间隔及典型主线程停顿。
- pose_stall_all_output_alignment.json：九个有Pose会话119个>100ms处理间隙，均与上一帧20Hz输出相符，使用CSV的elapsed_s与stdout三位小数对照。
- source_manifest.json / provenance.json：已读原始证据SHA256、实际/参考版本、代码指纹。原始文件在RDK保留，未清理或覆盖。
- 三图：分层延迟、到角/累积/速率、同一Pose的回调与处理时间。
- validation.json：34转动/97动作配对、119处输出对照、77份选取原文件哈希及8个相关源码指纹核对结果；只做离线必要检查。

方法：原始0x07/08按boot/reference及各自ID去重；轮ID连接首个生产control packet与被动接收端。yaw/rate分别使用自己的MCU采样时间；SDK角度由原始四元数和当前安装轴展开。停稳端点窗口为前−0.20～−0.02s、后+0.15～+0.35s，下个动作前50ms截断；没有人工独立角度真值。API仅主机返回，DONE仅固件回执。

上一份联合分析的控制表曾混淆映射采样端点与host UART RX，本轮已在原报告更正并列出两种作用域，不能把两张表当性能先后对比。

RDK下载目录的本轮派生分析包（不覆盖旧两场轨迹/联合分析ZIP）：

```text
/home/sunrise/下载/RDK_MotionDebug延迟到角_20261109_深化.zip
```
