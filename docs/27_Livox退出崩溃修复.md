# Livox 退出崩溃修复（2026-10-13）

现场日志：驱动收到 SIGINT 后，以 -11 退出；内核指向系统 libspdlog.so.1.9.2。
SDK 自带 spdlog 1.3.1，导出的同名 C++ 符号被系统 1.9.2 替换，对象布局不一致。
离线仅加载 ROS 日志库和 SDK、初始化并清理日志即可复现 `free(): invalid size`，
GDB 栈包含系统版 `spdlog::logger::_M_dispose` 和 SDK 的 shared_ptr 析构。
因此不是雷达距离测量或底盘移动访问越界；触发点是日志对象释放。

依据：[Livox SDK2 官方仓库同类问题 #44](https://github.com/Livox-SDK/Livox-SDK2/issues/44)。

修复工具 `tools/livox_shutdown/patch_shutdown.py`：

- SDK 移除内置 spdlog 头文件目录，统一链接系统 `spdlog::spdlog`，包括 fmt 编译配置。
- SDK 使用独立的 `livox_sdk` 日志名称，退出只清理自身 logger，不再 `drop_all()` 清空 ROS 日志。
- 驱动退出先设置退出标志、唤醒点云/IMU 两个信号量，再等待线程退出。
- 停止驱动点云处理和观察回调后，再销毁 SDK；在节点/动态库仍存在时完成清理。
- 每个外部源码第一次修改前备份为 `.before_rb_shutdown`；保留现有网络配置和 PTP 补丁。

修复自动重启：`lidar_session.py` 不再用源时间戳有效/两路同步条件判断驱动是否断流。
只有收到过两路数据后，任一路实际停止送达超过 `--driver-silence-seconds`（默认2秒），
或链路断开/进程退出，才重启驱动。源时间与连续性检查仍用于定位接入、估计器重建；
PTP 初始化过程中持续有报文时，不反复重启驱动，也不把无效定位当成正常定位。
首帧前没有启动超时。运动与机构参数没有改动。

安装（先停止雷达相关程序）：

```bash
bash tools/install_livox_shutdown.sh
source /opt/ros/humble/setup.bash
cd /home/user/robocup_basketball_ws
colcon build --packages-select rb_bringup
```

SDK 原库备份：`/usr/local/lib/robocup-backups/livox_shutdown/`。
回退时停止雷达节点、恢复上述原库以及外部源码 `.before_rb_shutdown` 文件，重新构建驱动，
并同步回退本仓库会话管理器，再运行 `sudo ldconfig`。

验收边界：SDK/ROS 日志离线退出、无真实雷达的驱动 SIGINT 退出、回归测试与编译分别验证。
真实雷达长时间连续输出、FAST-LIO 定位和实车动作仍须重新接通设备后确认。
