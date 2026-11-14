# 开局进攻先正对对面安全区，再放货绕行返程

本轮基于f4e74dd，用户审核路线图后批准修正进攻朝向。16:40正式实跑为蓝方/1区/进攻，上位f4e74dd，UART actual63d75a8/dirty1；业务日志为rescue_map/runtime/history/20000101_080054_competition/，传感器为competition_wheel_input/20261114_164011_competition_wheel/。原第38行记录松爪yaw315.709°，旧左移目标Y=1.117523m，证明不能把该朝向当正确返程基准；坐标是日志估计/命令目标，不是地面独立测量。

进攻抵达藏点先锁本次NAV D0，以原成对DONE结束移动，再进入INITIAL_ATTACK_ALIGN，保持本次已审核载荷和闭爪，转正朝对面安全区：红270°、蓝90°。本次成对转向DONE/mode25才发原RELEASE_BOTH，原释放完成才进入INITIAL_ATTACK_RETURN；旧NAV/转向/释放回包不能跨动作推进。防守开局及普通投送返程保持。

释放完成后冻结实际车中心P，θ固定为上述己方对应的对面朝向，实际release_yaw_deg单独保留为证据，不再用它定义左右。以f=(cosθ,sinθ)、l=(-sinθ,cosθ)计：A=P−0.10f；B=A+0.50l；C=B+((R−B)·f)f；最后R红(0,+.30)/蓝(0,-.30)m。四段仍按5cm到点锁D0/原成对DONE推进，同边/异边都按己方决定R。原返程点、标称10/50cm及普通RETURN不变，不以DONE宣称地面精度。

同一0x18/opcode0x14增加step14 ATTACK_FACE_OPPONENT：A=B=0，U红27000/蓝9000，P1红0xE9/蓝0xE1。原step0/4方向固定，不能借改它们的语义实现。下位内部复用现有连续转向控制及原到角窗口，完成后mode25/COMPLETED；入口仅本任务已完成initial-stash NAV、闭爪及原合法initial-stash审核，保留审核以接受接下来的原释放。无新TYPE/opcode、握手、计时或ISR任务。

原step13 EMPTY_BODY_MOVE的A/B/U布局及执行保持，U现由上位固定为红27000/蓝9000；同段H/D刷新不换action，换段新action，四段后原step12恢复搜索。不把D0重解释成转向，避免改变搜索轮换的结束语义。机器合同见contracts/f405_empty_body_move_20261114.json；新请求MATCH-ATTACK-FACING-20261114。

扫障左边距本轮56→59cm；红S(-0.56,+0.91)/蓝(+0.56,-0.91)m，后边距94cm、各轴±3cm及扫障路线保持。机器合同中的目标值同步更新。

修改文件：attack_return.py、state_machine.py、run_competition_rescue.py、obstacle_route.py、protocol.py及原有三份检查文件；当前摘要/TODO/固定请求和机器合同同步。没有写人工ROI、模型、原始日志或归档定位工作，没有开启设备/电机。

**当前部署依赖：** F405 main及16:40实跑actual为63d75a8/dirty1，已有step13/SEARCH入口和前轮近爪配套；新step14尚待接收方审核、实现及用户安排烧录。审查Task.c blob a9f65ebf0427ae0e19da1d9803def4af0d299f2b，dirty差异未知，不等同板内完整字节。本窗口只读下位、不烧录、不重启入口或发车；不能用旧固件声称新转正流程已验证。

7项必要上位检查通过：8种起点/己方及任意原yaw的固定对面坐标系、转正后才松爪、错/旧DONE不推进、四段/SEARCH交接、原step0..13协议及成对CRC/同段action保持。无实车动作；转正带载效果、转向后地面点位及机械放货仍待联合验证。
