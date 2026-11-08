# 两端最新修改要求

更新：2026-11-08。只维护当前相关请求和回执；旧请求按Git历史取证，不自动恢复任务。
发布、读取、镜像要求均不等于审核通过或实施授权。接收方先审实际代码，向人说明影响，
可以接受、缩小、替代或拒绝。人保留知情、拒绝和选择权；已明确批准范围不重复审批。
协作约束见本仓库AGENTS.md和docs/CHANGE_WORKFLOW.md，不增加时间门、握手或故障锁存。

## 固定位置及启用状态

```text
RDK仓库：danmo-teng/shijue_fangan，分支codex/gamepad-teleop
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md

F405仓库：kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot，分支main
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md
```

RDK入口随本轮提交发布。当前GitHub连接对F405仓库只有读权限，对端同名文件尚未建立。
用户已选择由F405同学一次性建立入口，此后双方AI直接读取对端GitHub最新相关要求，
同步到自己仓库并回传审核/实现状态，不再逐次请用户转交。尚不能称两端入口都已落地。
这不是常驻执行服务；每次相关跨端工作先核对源文件当前提交和请求ID，避免拿缓存或旧版本执行。

## RDK → F405：UART-100HZ-01

**目标已由用户批准：这轮直接修改921600＋真实100Hz编码器通路。**
具体实现仍由F405端核对当前代码、说明最小diff和影响；新增重要取舍由人决定。
状态：RDK适配已提交6bfed9e；F405方案审查/代码提交/烧录/真实频率均尚未回传。

### 问题与证据

已读F405 main的6302bf2e643c034f5cca694c519be8245dcf5dd2及后续HEAD（核对时无后续提交）。
该提交不是板端actual证明，保留其舵机/导航等既有修改。

- CMakeLists.txt目前把APP_NORMALDEBUG_UPPER_BAUD设为230400，MotionDebug/NormalRun共用。
  Boards/F405/Src/board_f405.c按该宏设置物理UART4；句柄名huart3不能据名字改错物理串口。
- Robot.c原10ms调度已调用Encoder_Sample10ms；encoder_f405.c已有真实累计计数、区间增量和采样起止。
- DebugTelemetry.c的WheelSample把两个10ms区间合并，WHEEL_PERIOD_US=20000，因此07只有50Hz。
  GYRO_PERIOD_US=10000，08有新源时最高100Hz；Uart.c已有唯一DMA发送者和待发同类遥测合并。
- 07/08各95字节，100Hz＋100Hz合计19000B/s，921600/8N1容量92160B/s，占20.62%，
  这里只算07/08，仍须加既有状态等；单帧线传输约4.12ms→1.03ms。不是端到端实测延迟。

### 最小修改

1. 仅把RDK↔F405这一路上位机UART4改为921600、8N1，MotionDebug与NormalRun两个现有构建都一致。
   不改主IMU串口及其230400、调试文本串口115200、雷达230400、引脚映射或外设资源。
2. 复用原10ms编码器采样，每个真实相邻采样区间生成一次新的07记录；取消两段汇总20ms才发布。
   WHEEL_PERIOD_US及CONFIG_REPLY的wheel_period_us如实为10000。速度分母使用实际t1−t0，
   不硬写10ms、不重发旧样本冒充100Hz；保留累计计数、真实增量、measurement_id和原有效位语义。
3. 保持协议1.3、D4 4D封包、TYPE及全部字段布局不变。07仍为车中心实测速度，前/左正、单位µm/s；
   沿用新F405实际运动学、原轮序/计数符号。08仍主gyro逆时针正、yaw毫度和角速度毫度/秒。
   没有新gyro源时不伪造新source_id；主gyro发布上限维持100Hz。
4. CONFIG_REPLY保留原字段：uart_baud=实际921600、wheel_period_us=10000、gyro_period_us=10000，
   required_baud与本profile实际设置一致；身份02仍自报真实Git、dirty、board/mode/service。
   未知drift/source uncertainty继续未知，不改成已标定或新鲜资格通过。
5. 复用原UART RX/TX DMA、唯一发送者、短快照和同类待发遥测合并。采样结束后走原主循环尽快发布，
   不新增ISR、逐沿计时、质量巡检、统计框架、周期查询、握手或年龄门；不积压后集中补发旧速度。
   若现有发送调度确有具体问题，先说明实际证据及最小修改，不顺手重写驱动或扩大队列。
6. 不改电机PID/速度/距离目标、任务配对、合爪审核、人工STOP、mode51、比赛状态机和舵机成果。
   遥测配置/异常不接成新增全局停车、FAULT、永久锁存或等待资格。

两端影响：烧录后RDK必须同时用921600；旧230400主机无法与新固件通信。
RDK本轮已适配C++串口、共用测试客户端及正式/测试入口；保留显式230400选项用于尚未升级的板。
旧测试客户端的Git SHA白名单已移除，按现有board/mode/service/协议布局核对，actual原样记录。
不需要为了新固件SHA再改一次上位机名单；expected不得冒充actual。

### 必要检查与回执

复用现有两个构建和已有入口，只做本次必要编译/调用链核对；**不要求新增测试代码或独立测试程序。**
使用现有遥测回执/日志区分“配置声明100Hz”和“不同measurement_id、真实相邻约10ms区间的实收”。
未实测就写未实测；API接受、DONE和配置回执不证明地面精度或固定端到端延迟。
修改后commit/push，回传源码SHA、必要检查、实际构建/烧录模式及板端身份是否亲自读回。
有分歧先提出，不把这份任务单视为对新增方案的批准。代码完成后RDK再核对当前diff。

当前RDK定位算法未改：控制/显示仍存在20Hz快照链，这由下一步拆分处理；本任务不谎称已实现100Hz控制。

## F405 → RDK：回执

待对端固定入口建立及UART-100HZ-01回传。当前没有新固件实现、烧录或实收100Hz证明。

## 对端一次性建立入口的可复制说明

```text
请先核对你自己的AGENTS、当前状态和Git，保留已有修改。用户已批准本轮921600＋真实100Hz编码器目标。
请直接读取上位机最新要求，而不是按旧聊天恢复任务：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md

一次性在本仓库建立docs/CROSS_REPO_REQUESTS.md，记录这次请求ID、上位机源提交、相关要求及你的审核/实现回执；
以后有相关跨端工作由AI自行读取对端GitHub最新入口，更新本仓库同名文件，不再逐次要求用户转交。
本仓库AGENTS加入固定入口指针：只读与当次目标有关的新要求；发布/读取不代表审核通过或实施授权；
先说明最小改法、影响和代价，接收方可缩小/替代/拒绝，人保留知情、拒绝、选择权。
当前已批准范围不重复审批；超范围的重要改法仍先问人，不得让两端AI互相授权。
原禁止擅增新鲜度、握手、资格等待、停车/FAULT和无效测试的约束继续有效。
先按实际代码审查UART-100HZ-01，再实施已批准范围，必要检查后commit/push；
把审核结果、实现SHA、未完成项、实际构建和烧录/读回状态写回固定入口。不要把要求、代码、烧录混为完成。
```
