# 测试日志容量显示与64GB TF迁移

2026-11-04，基于79ae68c增量实现。仅测试界面/日志管理；未改正式比赛、F405、定位算法或外参。本轮没有清理真实日志、启动硬件或制作/写入系统镜像。

## 现在如何使用

- 重新打开已有T265地图或雷达＋T265动态地图，底部显示日志所在文件系统的可用/总容量，每秒后台读取一次。2026-11-08已扩容，实读约59GiB总量/33GiB可用；这是日志所在文件系统容量。低于10%或1GiB告警，低于1GiB红色提示；不停车、不拒发、不停注入。
- 点击「日志管理」→勾选旧会话→「删除所选…」→核对名称、数量和容量后再次确认。可翻页；列表只在打开、刷新和删除完成后读取，容量查询不逐帧遍历日志。删除任务在后台执行，STOP按钮和S/空格仍有效；面板打开时其他动作键不穿透到地图，返回后恢复正常操作。
- 桌面「测试日志管理」使用同一程序的`--logs-only`模式，不打开USB、T265 SDK、底盘或雷达，也不新建运行会话；T265启动失败时仍能查看容量和清理。该模式ESC/Q退出。

```bash
python3 /home/sunrise/RDK_X5/shijue_fangan/localization/tools/t265_f407_motion_map.py --logs-only --fullscreen
```

只识别本模块`YYYYMMDD_HHMMSS_interactive`会话。当前会话、明确保护、不完整、符号链接、含子目录或未知/用户文件的会话锁定；不会递归删除。已选择目录在删除前重新核对归属、保护清单和文件快照，内容有变即跳过。保护清单损坏时停止删除操作，运动链不受影响。

当前政策：不再自动只留5次，也不因容量不足自动删除。新式会话须有完整完成标记；旧式无SDK会话ID才兼容正常终结事件。分析完成、确认无价值后手动勾选删除；本轮未清理真实日志。桌面新增「比赛定位日志管理」，复用同一个日志面板，根目录和owner限定为competition_wheel_input，不启动任何传感器。

实现文件：

- `localization/tools/motion_log_storage.py`：共用会话归属/保护/删除规则、单后台任务及缓存面板。
- `localization/tools/t265_f407_motion_map.py`：现有T265入口、完整会话边界、仅日志模式。
- `localization/tools/lidar_t265_motion_map.py`：复用相同面板与按键处理。
- `localization/tools/t265-log-manager.desktop`、`install_desktop_launcher.sh`：桌面入口及安装。
- `localization/tools/test_f407_motion_protocol.py`：在现有检查入口中用临时目录验证删除边界，不新造测试程序。

必要检查：临时目录的保护/当前/未完成/用户文件/链接/选择后内容变化/最近5次保留；手动二次确认、STOP、容量读取失败；仅日志模式不启动传感器、不生成会话；现有动态地图1024×768渲染与直接相关检查均通过。真实目录只读预览，未点击删除，未进行实车验证。

## 迁移推荐：关机后在电脑克隆整张旧卡

以下是扩容前记录的迁移操作参考，2026-11-08本机根分区已扩容；不应重复格式化/克隆。RDK系统3.5.0，原旧卡`/dev/mmcblk1`约29.5 GiB，含256 MiB启动分区和ext4根分区。**建议完整镜像迁移到64GB卡，保留原环境，不重刷空白系统后只拉Git。** Git不含已安装SDK、模型、标定、桌面设置及未跟踪用户资料。

1. 准备64GB新TF卡、读卡器、电脑至少35 GiB空闲空间。保存工作、结束测试，正常关机，断电后取出旧TF卡；不要带电拔TF卡。
2. 用电脑将**整个旧卡设备**读成`.img`，保存到电脑硬盘。Windows可用[Win32 Disk Imager](https://sourceforge.net/p/win32diskimager/wiki/Home/)的Read；不要选Write、不要格式化旧卡，Windows提示格式化Linux分区时取消。Linux可用[GNOME Disks](https://wiki.gnome.org/Apps/Disks)的Create Disk Image。
3. 拔旧卡，插新64GB卡，把刚才的镜像写到**新卡**并校验。Windows可用同工具Write，或官方文档列出的[Rufus / RDK Studio本地镜像](https://d-robotics.github.io/rdk_x_doc/en/Quick_start/system-burn/burn-sd-card/)；Linux可用GNOME Disks的Restore Disk Image。写入会覆盖新卡全部内容；旧卡保留作回退。
4. 新卡装回同一RDK，开机后执行：

```bash
sudo srpi-config
```

选择`Advanced Options`→`Expand Filesystem`→按提示完成/重启。官方明确指出，复制已初始化系统后可能不会自动扩容，应使用此项：[RDK扩容说明](https://developer.d-robotics.cc/rdk_x_doc/System_configuration/srpi-config)。

5. 验证容量和项目：

```bash
lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINTS
df -h /
git -C /home/sunrise/RDK_X5/shijue_fangan status --short
git -C /home/sunrise/RDK_X5/shijue_fangan rev-parse HEAD
```

64GB标称容量在Linux约59.6 GiB，根文件系统略少；若仍约29 GiB，说明尚未扩容。核对SDK、模型/标定、桌面入口、`vision/456`、`雷达与融合`和保留日志都在，再用「测试日志管理」确认剩余空间。新卡启动及环境确认前保留旧卡，不同时把两张克隆卡接入RDK运行。

## 可选：在RDK上用官方备份工具制作镜像

本机已有`/usr/bin/rdk-backup`。如果有外接存储，可先停止测试和其他写盘程序，再用它把系统打包到外接盘，随后按上面第3–5步写入新TF卡。[官方命令说明](https://d-robotics.github.io/rdk_x_doc/en/Appendix/rdk-command-manual/cmd_rdk-backup/)

下面路径只是示例，需替换为实际外接盘挂载点，路径不要含空格。先用`findmnt`确认目标确在外接设备，不能是当前`mmcblk1`根分区。外接盘建议至少40 GiB空闲，使用支持大文件的文件系统（例如ext4/exFAT，不能FAT32）。

```bash
findmnt -T /media/sunrise/RDK_BACKUP
df -h /media/sunrise/RDK_BACKUP
sudo rdk-backup /media/sunrise/RDK_BACKUP
```

镜像输出为该目录的`rdk-时间.img`。本机脚本会联网安装依赖；它按已用根分区空间约1.2倍创建镜像，复制启动和根分区。核对脚本实际排除`/media`、`/dev`、`/proc`、`/run`、`/sys`、`/tmp`、`/lost+found`及备份工作目录，启动分区单独复制；本项目`/home/sunrise/...`在复制范围。若重要资料实际位于被排除目录或外接盘，需要单独保留。不能把备份目录设为`/`或用户主目录。

**备份优先写外接盘，不默认往当前SD的/mnt写整个系统镜像。** 本轮未执行备份、刷卡或扩容。C++事件日志已改有界后台写盘，打开失败/写满不抛到控制链；地图状态文件写失败只警告。缺目录和/dev/full必要检查通过；完整会话标记仍只在必要日志保存完成后生成，不把日志不完整说成完整。
