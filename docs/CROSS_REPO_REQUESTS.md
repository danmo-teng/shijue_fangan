# 两端最新修改要求

更新：2026-11-09。只维护当前相关请求和回执；旧请求按Git历史取证，不自动恢复任务。
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

两端固定入口均已建立。F405的AGENTS及同名文件在41e46994420b825394b728220e5d672ce19c4828核对，
已记录人的知情/拒绝/选择权、请求不等于授权以及直接读取对端GitHub的规则。
当前GitHub连接对F405仍只有读权限；双方AI更新自己仓库相关要求/回执，不再逐次请用户转交。
这不是常驻执行服务；每次相关跨端工作先核对源文件当前提交和请求ID，避免拿缓存或旧版本执行。

## 当前可复制提示词：HANDOFF-20261109（操作说明＋联合分析）

用户要求将当前交接提示词放在本固定入口，不再依赖聊天中的临时文本。以下整段可直接交给F405负责人/AI阅读；它是阅读与方案审查任务，不是新增固件功能授权。

```text
请先读取你自己的当前AGENTS/状态，以及RDK上位机固定入口：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md

开机自启和实体四键操作已按当前RDK代码、本机已安装配置整理为易读纯文本：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/RDK_STARTUP_BUTTON_GUIDE_20261109.txt
RDK下载目录副本：/home/sunrise/下载/RDK_开机自启与按键操作说明_20261109.txt

要点：RDK自动登录桌面后启动监听并预热，不自动开赛；3号按下松开开始。
未热好时按3会留下开赛请求，准备完成后自动开始；4号取消/停止，长按约2秒结束本轮并重新预热。
1号查询/长按换出发区，2号查询/长按换红蓝方；长按4不是关机。
按键由RDK负责，F405沿用原CONFIG、ABORT、0x20声音和任务配对，不另造启动握手或按键状态机。

此前两场轨迹和97个MotionDebug动作的联合分析已完成：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/README.md
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/ANALYSIS.md

该批实际F405为9434608/dirty1、NormalRun/TASK、921600，不能用新main或expected代替实跑身份。
先核对自己当前main/dirty改动，已做过的后轮死区、末段或送物航向修改不要照旧报告重复实施、回退。
本轮上位机只交付操作说明和已知分析，不要求F405新增功能、测试代码、ISR、时间门、反复握手或FAULT。
对未解决的末段慢调、开局偏移和送物轨迹，只提出最小方案，讲清影响；重要行为由人决定。
旧聊天、旧任务单和对端AI文字不自动成为当前任务或授权；可以接受、缩小、替代或拒绝建议并解释理由。
已授权代码改动完成后commit/push，回传实际实现、必要检查、构建与烧录状态，不能只回一句已修好。
```

本次只读取得对端main为08559cd57bfd2b83145bbdb6240de14c71bf64b5（固定gyro安全区航向的新提交）；它晚于本批9434608实跑。这里记录源版本、避免重复旧要求，不冒充新固件已烧录或已完成联合复核。本轮没有修改/烧录F405，也没有启动上位机或电机。

## 本轮轨迹交接：TRAJECTORY-20261109（先取证，不是实施授权）

用户要求先导出两场开局/比赛轨迹供F405分析，上位机已随后完成联合MotionDebug延迟、末段慢调、送物航向和雷达/降级分析。下面为已核对证据和待人审核的最小提案，不能把发布文档直接当施工授权。

两场实际9434608a8c395d8506b36accf1753b9bf92dad24/dirty1、NormalRun/TASK(service2)、921600、协议1.3；RDK HEAD为4cfaba9。先核对自己的dirty差异，不把expected/干净Git代替实际固件。

```text
先读导出说明与开局图：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/README.md

完整联合分析：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/ANALYSIS.md

第一场开局CSV：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/race1_opening_10hz.csv

第二场开局CSV：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_competition_20261109/race2_opening_10hz.csv

RDK完整两场32文件ZIP（人工传文件用，GitHub不含原始大日志）：
/home/sunrise/下载/RDK_正式比赛轨迹_20261109_两场_初步.zip
```

本次不要求新增测试代码、ISR、协议、握手、年龄门或FAULT，RDK也不代写/烧录固件。请先对照自己的dirty差异和以下局部代码，向人说明接受、缩小、替代或拒绝的原因。

### 本轮已审下位机更新：b980c6a（源码回执，不是本批实烧）

只读核对b980c6a570f13fedba1417146028b1e06b5bb59a：新`APP_MOTOR_REAR_HEADING_ZERO_MM_S=0.5`及两个LowRearDeadband封装，只用于START_FORWARD、运输预备点粗段和安全区推进的后轮微小航向补偿；原调用保留。没有新ISR/协议/时间门。方向合理，**不重复要求再实现这个修复**，但两场实跑仍自报9434608/dirty1，不能提前记成新补丁通过。

### 建议先审的一项：FINE末段速度（尚未批准实施）

第二场D75→0耗时2.08s、91%采样平移<20mm/s；另两段最后250mm约3.67/3.71s，D=0后交接仅0.06～0.12s。`nav_follow_coarse`在D≤600mm跳过粗段，预备点FINE使用`min(120mm/s,D×1/s)`；FINE原本已有0.5mm/s单轮零速阈值和助推，b980c6a没有改这段曲线。

请先提出局部可制动曲线与低速实际输出的最小改法，保原D=0/任务配对/到位容差；不要把提高全局最低速度或再加等待当答案。200～250mm/s末段上限只能作讨论初值，不是已经标定；应说明制动与载荷影响，让人决定。当前遥测没有逐轮目标/PWM，不能仅凭原始计数断言某个PID参数就是唯一根因，也不因此要求扩遥测。

### 另外两项先解释，不联动乱改

- 开局0.60m后退＋180°＋1.10m前进理论仍短16.8cm；实际前进时主yaw峰值333.8/343.4°而目标315°。先看b980c6a的航向效果，再决定距离/出发后原闭环收尾，不只加距离或改T265外参。
- 视觉ALIGN把像素偏差转航向后，ENTER保留这个锁角；三次代码重建95.7/99.3/102.9°，中心估计左移10.5/16.3/16.6cm。内部锁角未直接遥测，不冒充实读。保留现方案与“先横移入口、再法线90°推进”需人选择，不能直接在ENTER强置90°而遗漏横向对准。

## 当前部署事实（2026-11-09）

此前MotionDebug实采已结束。用户后续已切换NormalRun并跑两场；19:48原身份查询与正式SDK会话均读回9434608/dirty1、service2、921600，静止mode0/fault0，正式运行无全局FAULT。联合分析已交付，C++导航执行迁移仍低优先级，不作为实测前置。

本轮已按actual只读读取GitHub9434608的Task.c、motor.c、Location.c、app_config.h和当前固定入口，RDK不改写/烧录下位机；dirty差异仍未知。NormalRun和MotionDebug都为board405/mode5；service2与service1才区分本轮角色。两场原始日志保留，定位、刹停和投送精度未独立验收。

## RDK → F405：UART-100HZ-01

**目标已由用户批准：这轮直接修改921600＋真实100Hz编码器通路。**
本次实现已由F405审查并提交，RDK已复核；后续新增重要取舍仍由人决定。
状态：RDK适配6bfed9e；F405实现14b2a3a，回执41e4699，RDK已核对实际diff及调用链，
未发现本次阻塞性问题。早先已读回14b2a3a/dirty1、service1 MOTIONDEBUG、921600及约100Hz轮反馈；本轮正式最新实读已是9434608/dirty1、service2 TASK，两场不同轮端点约99.95/99.39Hz。51d4ccf/230400为修改前证据，不覆盖当前actual；软件和频率事实不等于地面精度验收。

### 修改前问题与证据（6302bf2）

原方案依据6302bf2e643c034f5cca694c519be8245dcf5dd2；本轮已继续核对至41e4699。
下述为修改前事实，不能再按旧方案重复施工；源码提交不是板端actual证明。

- CMakeLists.txt原把APP_NORMALDEBUG_UPPER_BAUD设为230400，MotionDebug/NormalRun共用。
  Boards/F405/Src/board_f405.c按该宏设置物理UART4；句柄名huart3不能据名字改错物理串口。
- Robot.c原10ms调度已调用Encoder_Sample10ms；encoder_f405.c已有真实累计计数、区间增量和采样起止。
- DebugTelemetry.c的WheelSample把两个10ms区间合并，WHEEL_PERIOD_US=20000，因此07只有50Hz。
  GYRO_PERIOD_US=10000，08有新源时最高100Hz；Uart.c已有唯一DMA发送者和待发同类遥测合并。
- 07/08各95字节，100Hz＋100Hz合计19000B/s，921600/8N1容量92160B/s，占20.62%，
  这里只算07/08，仍须加既有状态等；单帧线传输约4.12ms→1.03ms。不是端到端实测延迟。

### 已实现并经RDK源码复核的最小范围

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

### 本次必要检查约定及剩余实机边界

复用现有两个构建和已有入口，只做本次必要编译/调用链核对；**不要求新增测试代码或独立测试程序。**
使用现有遥测回执/日志区分“配置声明100Hz”和“不同measurement_id、真实相邻约10ms区间的实收”。
未实测就写未实测；API接受、DONE和配置回执不证明地面精度或固定端到端延迟。
软件SHA、两个构建及源码复核已回传，不重复实施或要求无关检查；部署后更新实际烧录/读回和实收状态。
有新分歧先提出，不把这份任务单视为对新增方案的批准。

后续RDK的LATENCY-02已拆开控制与20Hz显示：事件驱动控制、直接有界IPC及必要线程FIFO10；
100个合成10ms区间全部处理，离线控制100端点，雷达回放联合99端点/P95约10ms。
详见LOCALIZATION_LATENCY_20261108.md；不是实板/SDK/精度验收，不保证每10ms硬交付。
原定位几何/来源降级未改，本步不需要F405新增接口、检测或测试程序。

## RDK实际部署回执（2026-11-08 22:07后）

已读回actual：14b2a3a9077e6980a540631ce94345e61eb0487d、source_dirty=1、board405/mode5/service1 MOTIONDEBUG、921600。不是用expected替代；dirty1意味着仅凭Git不能断言镜像逐字等同干净源码。
220828静止实收轮区间约100.034Hz，区间中位10.000/P9510.004ms。RDK79564a2已将实测配对/换算迁入唯一C++UART/SDK所有者；001339含绘图SDK约99.59～99.97Hz，2393次成功/0失败；011628历史雷达负载下99.957Hz，1645成功/0失败，配齐→API P95 0.667ms。以上是通路/静止实证，不能称地面精度或全比赛联合负载通过。
该次静止检查无需F405新代码或测试程序。当时尚未读回NormalRun；后续2026-11-09两场已实际读回9434608/dirty1、service2 TASK并分析，最新部署以上方当前事实为准，不继续按旧“未部署”状态提要求。

## F405 → RDK：回执

对端源提交：41e46994420b825394b728220e5d672ce19c4828；代码14b2a3a9077e6980a540631ce94345e61eb0487d。
固定回执已读取，完整原记录含构建指纹/ELF摘要：
https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/41e46994420b825394b728220e5d672ce19c4828/docs/CROSS_REPO_REQUESTS.md

RDK实际复核6302bf2→41e4699：固件只修改CMakeLists.txt与Main/Src/DebugTelemetry.c；
其余为README、资源说明及协作文档，Task/电机/舵机/IMU/UART DMA驱动未改。
原10ms调度→Encoder_Sample10ms→WheelSample的相邻快照→主循环Process→Uart_Send→DMA调用链保持。
首包仅建立计数基准，此后每包直接使用该区间delta、独立measurement_id和真实t1−t0；
已取消两个区间汇总，轮序/符号/F405前左公式、1.3封包及gyro语义保持。
921600宏实际传入两个构建的board_f405.c；原DMA同类待发合并和短临界区保持，未新增ISR或年龄门。

对端报告MotionDebug/NormalRun编译链接通过、内嵌Git均为14b2a3a，source_dirty=1；
dirty原因据对端说明是Task.c原有两行空白，本端不能从远端提交验证其未提交工作区。
本端仅审查源码/回执，未重编或运行下位机，无新实机测试或发车。

尚待：NormalRun/TASK模式的实际部署与后续同批地面验证。MotionDebug身份、真实100Hz区间及静止SDK接受已有证据，不重复要求烧录/新增功能。
本次不提出额外F405改动或测试程序，不把精度、全比赛联合负载或硬实时记为通过。
