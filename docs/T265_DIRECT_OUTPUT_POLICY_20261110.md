# 纯T265输出：关闭轮速不恢复年龄门

2026-11-10，TIME-01B增量收尾。按当前已确定的数据采用政策修正遗漏，不改变定位来源选择。

`localization/src/main.cpp`的直接0x16输出原先只在存在官方轮反馈socket时绕过250ms Pose年龄判断。纯T265没有该socket，即使明确使用`--t265-only-output`，仍可能因年龄清除VALID。现在纯T265及官方输入路径均不据年龄清VALID；短路求值也不在这两条路径额外读取处理时钟。未收到/未接受Pose、实际confidence及其他状态语义保持，未把无数据编成有效定位。

没有修改SDK注入、输出坐标、安装参数、比例、pose jumping、正式动态来源仲裁或下位机协议。显式旧融合支路仍保留原行为，不能据本项声称全仓已删除所有年龄相关逻辑；合爪业务例外不变。

必要验证：

- `cmake --build localization/build --target t265_omni_localizer -j2`成功；构建产物SHA256为`99ae162ad1266728c93f8db9bd55869199b897f77bd5b6b5e6923c121e5d6f07`。
- 既有`competition_rescue/tests/test_runner.py::test_formal_t265_mount_is_independent_of_wheel_input`通过，覆盖四起点纯T265/官方输入入口及现安装配置，不改变原默认链。
- `git diff --check`通过。没有为本条件新增独立测试程序或运行无关整套检查。

本轮没有打开SDK、UART或电机，没有实际读取0x16，因此这是代码修复和必要构建验证，不是设备送达或地面精度验收。原始日志与用户资料未改、未清理。提交及推送见本文件所在Git提交。
