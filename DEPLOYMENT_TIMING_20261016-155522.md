# 主机、底盘、雷达时序部署记录

2026-10-16，已按用户要求将本地时序修改部署到user-NUC12的/home/user/robocup_basketball_ws。

本记录更新此前LOCAL_CHASSIS_POWER_RECOVERY.md和LOCAL_LIDAR_COLD_START.md所述“仅本地、未上传”的状态；这两份说明保留其编写时的验证范围。

## 已完成

- 逐项比较原主机快照、当前主机文件和本地目标，48个文件无冲突，已上传并逐一校验SHA256。文本使用LF。
- 保留297个本次范围外的已跟踪文件，包括主机后来改过的路线点位、field_editor及localization配置。没有切换提交或提交/push；主机HEAD仍为2d88f20a2dcb23a2260f1b9d6513eaf5af09b5a1。
- 备份完整已跟踪源码、部署前Git差异和两个包已安装的产物，再更新源码。
- 在Linux主机编译并安装rb_chassis、rb_bringup到现有install目录。新底盘执行文件来自build_timing_update_20261016_155522；已核对链接了新Can::transport_generation，而非旧CAN发送线程实现。
- 已安装lidar_startup.py、lidar_health.py、lidar_session.py；真实ROS launch参数渲染及执行文件共享库依赖检查通过。
- 主机既有硬件PTP服务及外部Livox时间转换补丁检查通过，本次无需重新安装或重编译外部雷达工作空间。
- 361项Python检查、3项C++ CTest全部通过，含此前受Windows环境限制的6项检查。四个shell脚本bash -n检查通过。
- 安装期间暂时停止前端服务，完成后恢复。前端8090/api/boot/state响应正常，状态waiting；整车启动服务inactive，未启动真实底盘/雷达ROS节点或执行任务。

## 运行方式与限制

后续自动启动会加载新版install。主机与雷达一起上电后等待链路、静态IP和PTP及有效数据，定位稳定后就绪；DJI板和四路电调可随后一起上电，任务需等待底盘握手和定位同时就绪。

任务/底盘通信的无超时停车设置保留。雷达管理器的恢复等待仅用于重试失效传感器进程。已结束的路线不会在恢复后自动继续。

本次没有烧录DJI板；底盘握手仍要求板端具备之前修改的配套固件。当前雷达网口没有carrier，无法在此次部署中验证真实PTP锁定、冷启动、CAN断电恢复和电机扭矩；测试均未驱动机器人。

## 备份与日志

主机备份：/home/user/robocup_basketball_ws/backups/timing_update_20261016-155522

- source_before.tar.gz：部署前源码及原有未提交修改。
- install_before.tar.gz：部署前rb_chassis/rb_bringup已安装文件（保存了符号链接指向的实际内容）。
- tracked_before.json、manifest.json、deployment_verification.json：原文件与目标校验值及部署核对结果。
- git_status_before.txt、git_diff_before.patch、head_before.txt：部署前Git状态。
- build.log、pytest.log、ctest.log、launch_args.log、services_after.json：构建、检查、参数和恢复后的服务状态。

暂存目录：/home/user/robocup_timing_stage_20261016-155522

今后如需回退，应以该备份为依据按部署范围恢复，保留部署后产生的独立改动。
