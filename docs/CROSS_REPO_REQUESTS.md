# 两端最新修改要求

更新：2026-11-10。只维护当前相关请求和回执；旧请求按Git历史取证，不自动恢复任务。
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

## 最新请求：GYRO-FORWARD-REVIEW-20261110（下次维护核查，不预设改固件）

用户本轮要求：下次核查高精度单轴、带TI M0滤波的串口gyro从接收到向RDK转发是否存在延迟问题。用户认为模块波特率已最高；本轮不要求再提高波特率、改滤波或接线。重要方案仍需接收方说明、由人决定。RDK只读重核main=b56bc80007db69a756525b941310e4c4eb60dcc7，固定入口blob仍为54ad68124628fc1f2fc361d82bec4dd342850b61；尚未收到对端本项审核回执。

```text
请求ID：GYRO-FORWARD-REVIEW-20261110
范围：请在下次维护时先核查当前gyro接收、最新源快照及UART4 DMA转发，不预设必须改码。
完整证据与源码位置：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/priority_todo_20261110/GYRO_FORWARDING.md

已有证据：5会话94608个TYPE08发布，源接收→publish P95约0.94ms、最大1.835ms；
这不含publish之后的排队/DMA/主机接收，不能当全部转发延迟。
约28ms相位在F405接收源时刻仍存在，不能把它全部归到后续UART4排队，也未证明全是模块滤波。

请核查：
1. 实物型号/现滤波与输出配置、实际230400接收；AA/BB时戳确实对应各自帧处理，
   已有ORE/校验计数及中断优先级/长关中断段是否提示接收问题。
2. DebugTelemetry中独立yaw/rate源ID、单位、100Hz最新快照及源时刻；不要用整包SEQ当同时采样。
3. Robot主循环服务和Uart唯一DMA起发者之间是否阻塞，未发送07/08合并、
   ACK/任务状态优先和原恢复分支是否造成可避免等待。优先读源码和已有日志/命令，不加测量框架。

回传：已证问题或无需修改、具体文件/代码行为、未知项；有现成实机证据再列actual，不用源码HEAD代替。
若发现要改，先向人说明最小方案、开销及两端影响；重要改法经人决定后实施、必要检查、commit/push。
不新增时标/定时器/ISR工作、协议/遥测、高频计时、新鲜度门、握手、停车/FAULT或额外测试代码。
RDK直收/并接只作后续候选，不会消除模块内部滤波延迟；未核对电气接口与配置发送权前不改线。
```

## 上批审查：PRIORITY-REVIEW-20261110（准备完成，非固件修改命令）

用户要求完成最高优先级欠项的候选和审查，是否采用再讨论；性能优化暂缓。本轮只读F405 main快照b56bc80007db69a756525b941310e4c4eb60dcc7，未修改、编译或烧录固件。已核实STAGE FINE 110～180mm/s、开局限加速/现0x16补中心、红90°蓝270°锁角、部分爪等待已在源码，不再重复下旧任务；本批实跑仍为9434608/dirty1，不用新main替代actual。

```text
最新完整审查（先读这一份）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/priority_todo_20261110/LOWER_AUDIT.md
轮滑、同步对齐及T265基础的离线候选/对照：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/priority_todo_20261110/README.md

源码已做不等于地面效果通过：
1. STAGE新速度先看新版有载刹停，不追加200～250mm/s或改到位容差。
2. 开局补中心有80mm结束范围、800mm轮式预算和原30秒START预算，继续SEARCH不等于严格到中心。
3. 固定法线已实现，但没有视觉横移入口；是否补横向对准由人结合新画面决定。
4. S1退出55↔88度按现公式每侧约108ms，是命令角估算，不冒充实际舵机到位。
5. 35文件调用审查已完成：当前应用没有IMU_ZeroYaw调用，RESET/起点/mode51均重建Location参考。Location仍丢弃超过30度的单步累计yaw；候选只删该截断及无用常量，代价是模块自身异常角也会进入Location，需用户决定后才发布实施要求。当前定角直接读IMU，不能把旧MOTOR5归因于这个Location截断。

本轮没有新的F405实施要求，不加板端测试代码、时标、ISR、遥测、握手、新鲜度门或FAULT。
RDK已修降级首因被覆盖的诊断、纯T265直接输出残留年龄门及原地图简洁页；来源策略/正式流程不变，离线对齐/选源候选不要求另一端照搬。
下面旧轨迹交接中的“先改FINE/开局/视觉锁角”已由此源码回执更新，不能重复执行。
```

## 最新补算：SENSOR-SYNC-20261109（六对同步，只读证据）

用户指出上一轮主要交付处理耗时，未完整计算传感器间同步。本轮已补齐原始波形/墙面几何的六对相位；没有修改F405或发布新的固件实施要求。

```text
四路同步报告：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/sensor_sync_20261109/ANALYSIS.md
每动作各时间口径的编码器/T265/gyro结果：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/sensor_sync_20261109/motion_pair_lags.csv
原始雷达点拟合的全部结果（含异常/小角和正式转动）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/sensor_sync_20261109/lidar_pair_lags.csv

符号：A→B为B的运动特征相对A的延迟，正数更晚；不是外部标定的硬件绝对时延。
共同14次正常90/180转动，估计源时刻中位：
编码器→gyro +29ms；编码器→T265 -2ms；gyro→T265 -28ms；
编码器→原始雷达 +5.4ms；gyro→原始雷达 -27.9ms；T265→原始雷达 +2.3ms。
雷达用原始光束/墙面拟合、中心自由运动，不拿已采用gyro的定位航向自证零延迟。
27个有SDK的MotionDebug转动及两场70个活动区间复核：T265相对gyro源相位约-28～-29ms；
按主机原始RX则约-23～-24ms，二者不能混用。小于几毫秒只宜说接近同步。
平移F/A的全波形RX匹配中位+30/+6.5ms，A方向间-29～+34ms，不支持固定平移延迟补偿。
第二场雷达包收齐相对日志点时刻P95 10.36ms/最大276.86ms，另有接收/映射长尾，不等同提线CPU。

约28ms gyro历史回移只是待人讨论的候选，不是本次要求F405改时间戳、滤波器或控制器。
不据此增加计时/ISR/年龄拒绝/握手/FAULT/测试代码，也不改gyro正方向、外参、比例或轮臂。
本批actual仍为MotionDebug ac61480/dirty1、正式9434608/dirty1；源码不能代替dirty差异或已烧录证明。
```

本轮只读重核F405固定入口，文件blob仍为54ad68124628fc1f2fc361d82bec4dd342850b61；它的既有要求不自动恢复为本轮实施授权。原始日志未改或清理。本地分析图表/当前状态提交推送后由对方按固定入口读取，不宣称其已读、已审核或已部署。

## 此前补充：MOTION-TIMING-20261109（处理耗时和到角）

本轮依用户要求继续以MotionDebug作受控响应/到角依据、两场正式作联合负载参考。完整数据和表图已提交本仓库以下固定路径；没有修改或烧录F405，没有新增运动/测试代码/ISR/时间门或握手要求。

```text
深化分析：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_timing_angles_20261109/ANALYSIS.md
全部34个转动（ID、SEQ、请求、实际yaw/SDK、过冲、起止、FAULT）：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_timing_angles_20261109/turn_angles.csv

MotionDebug实际ac61480/dirty1、service1；正式实际9434608/dirty1、service2，不用新main代替实跑。
32转动DONE按主yaw终点基本±1°，没有独立地面真值；4次+90再4次-90残余主yaw+3.312°、SDK+3.518°。
+10°@30耗4.404秒，曾过冲3.848°；两个+360分别停356.715/358.995°并报MOTOR5，内部失败子因未记录，不能说已查明。
请先对照自己当前MotionDebug主行程速率包络、低速输出/助推与修角收敛，只提出局部最小方案及影响，由人决定。
不要据此放宽容差、删除FAULT或新增诊断/遥测/ISR/测试代码，不要回退/重复正式已更新的开局、FINE和安全区锁角。

上位机另查出测试Pose消费0.75～2.47秒长尾，119次>100ms处理间隙均紧接20Hz输出；正式Pose消费P95约2～3ms。
旧报告把映射采样端点到接收的时间误标UART RX，已更正：192842真实UART RX→被动接收P95为4.09ms，非6.44ms。
正常轮RX→SDK API P95约10ms，其中等gyro中位约5ms；没有证据说明SDK正常注入本身积压几秒。
正式雷达参考→结果P95约102/130ms；实际正式0x16逐包送达延迟未记录，不能用被动接收端宣称已验收。
```

本轮已读取F405当前固定入口（文件blob54ad68124628fc1f2fc361d82bec4dd342850b61），其中已记录b980c6a后轮纠偏、37b4c00开局/末段及08559cd固定gyro安全区航向的源码更新。这里不把这些更新写成本批已烧录；旧分析只作证据，不恢复为新实施授权。原规则、人类知情/拒绝/选择权保持，RDK窗口对固件只读。

## 此前交接：HANDOFF-20261109（操作说明＋联合分析；最新修改状态见上文）

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

后续延迟分层、控制时间标签更正、Pose消费长尾及34转动到角深化：
https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/analysis/motion_timing_angles_20261109/ANALYSIS.md

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

### 旧提案的当前状态（11-10源码回执已替代实施建议）

旧9434608日志的75mm爬行、开局距离/偏航及视觉斜推证据继续保留，但F405已实现37b4c00末段/开局、08559cd固定法线和571d331等待缩短。不要重复发旧改法或把它们当新固件已实测。当前数值、剩余入口/预算风险及最短验收范围见最上方PRIORITY-REVIEW-20261110和LOWER_AUDIT.md；没有新的板端实施授权。

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
