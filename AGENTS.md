# 主机工程约定

用户所说的“主机”“主机代码”均指远程 Linux 的 `/home/user/robocup_basketball_ws`。
通过 SSH 别名 `robocup-linux` 连接。主机代码默认只在这里检查、修改；不以 Windows 副本判断远程实际逻辑，不自动同步 Windows 副本。
保留其他任务和用户未提交的场地、界面及配置改动。软件测试使用模拟器或独立 ROS 域的 PTY，不因编译或测试启动真实机器人运动。
