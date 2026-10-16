# 本地底盘独立断电恢复修改

工作区：E:/BasketballCode/robocup_basketball_ws-host-20261016-135923
基线：2026-10-16 14:00主机源码快照，提交e1cfd20，含采集时的三个未提交源码修改。
本次只修改本地目录，没有上传、修改Linux主机，没有启动机器人或烧录DJI板。

## 目标供电方式

主机与雷达一起上电并保持运行；DJI控制板与四路3508电调作为底盘一路一起断电/上电。
USB-CAN适配器持续在线。本次不覆盖拔插USB适配器导致can0重建的情况。

## 行为

1. boot.sh在底盘节点启动前统一配置CAN：1Mbps，restart-ms=100，txqueuelen=16，pfifo_head_drop limit=16。
2. 主机Can不再建立发送线程或应用队列。每次只尝试一次非阻塞send，失败立即返回，不保存报文等待重发。
3. 实际发送错误或CAN错误/重启帧更新传输代次；底盘丢弃故障前应答、旧握手及运动目标，重新查询。
4. 板子重新启动后，需要配套固件重新报告四路电调反馈、速度模式及使能，并完成查询/确认。
5. /chassis/ready发布握手状态。新任务同时等待定位与底盘就绪，再发送新的零速解锁运动。
6. 活动路线收到ready=false时按明确故障结束，不在恢复后自动续跑旧动作。可重新点击同一任务或其他回合。
7. 用户选择的无超时策略保留：速度指令和ready状态不因通信静默时间自动失效。100ms仅为bus-off恢复/握手轮询参数。
8. CAN接收poll仅用于本地退出唤醒；对端断电时析构不依赖发送特殊帧。dry_run退出也不发送CAN。

## 验证

- 280项Python检查通过：导航/定位/路线、晚就绪等待、取消等待、明确故障不自动续跑、CAN配置及权限失败处理。
- 编译实际Can实现并替换OS调用：20000次失败发送，无应用队列重放；错误代次、错误帧、总线重启后的接收、关闭流程通过。
- 编译实际Chassis/OmniChassis实现并替换ROS/motor_lib边界：旧速度无时间失效、实际故障清旧目标、重新握手、新零速解锁、故障前旧应答拒绝、显式停车及dry_run退出通过。
- BoardLink完成500轮模拟故障/握手重连，错误ID/长度/EFF/RTR及旧nonce不能确认。
- 当前主机协议与E:/Dji中实际板端协议函数的模拟互通通过。
- 两个shell脚本bash -n通过；修改的Python源码语法检查通过。
- Can原有private成员声明保持不变，新增运行状态不改变已安装motor_lib依赖的Can布局。

Windows本地尚未完成Linux ROS2整包编译，没有实机电源/CAN/3508扭矩验证。
内核/USB适配器内部队列无法由本地模拟证明已清空；本实现限制主机积压，并依赖配套板端握手拒绝未确认速度。
16帧队列与4096字节socket缓冲需要结合真实PCAN USB调度验证；如果正常工作也报告ENOBUFS，应依据实测调整，不能直接扩大到1000帧。

## 审查与后续

LOCAL_CHASSIS_POWER_RECOVERY.patch是相对下载快照的本地差异；LOCAL_CHASSIS_POWER_FILES.txt列出变更文件。
原始下载归档位于本目录同级，仍保留未修改快照。
local_validation/run_native_checks.ps1可重跑Windows模拟检查；Linux CTest新增can_transport目标，使用真实Linux头文件加系统调用替代。
部署时需同时更新并编译rb_chassis及本次任务脚本，确保/chassis/ready存在；板端需已烧录配套握手固件。
boot.sh配置CAN需要ip/tc权限或sudo -n。不要在路线运行中手动重新配置CAN。
