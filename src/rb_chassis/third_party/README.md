# 本工程的 bupt_can

主机使用本目录编译出的rb_can，替代系统旧版libbupt_can.a。

- send_can和send_can_with_respond都只调用一次send(MSG_DONTWAIT | MSG_NOSIGNAL)。
- 不创建发送线程，不保留应用发送队列；失败报文立即丢弃，后续只提交当前控制周期的新指令。
- send_can_with_respond返回的是socket提交成功与否，不表示DJI板确认收到。
- socket发送缓冲限制为4096字节（Linux可能按内核规则调整实际分配）。
- 实际I/O失败、CAN错误帧或总线重新启动会推进transport_generation，供底盘废弃旧握手及运动目标。
- 不按消息年龄判定失效，不增加通信超时停车。
- 接收线程使用100ms poll检查本地退出标记；析构不用发送特殊帧，因此对端断电也可退出。
- 保留报文EFF/RTR标志，错误帧不交给普通电机消息回调。

Can的原有成员顺序和类型均保留，新增状态放在实现文件的对象状态表中，以兼容已安装motor_lib的Can布局。
新增transport_generation()不改变对象布局。

tools/configure_chassis_can.py在底盘节点启动前统一配置：
1Mbps、restart-ms=100、txqueuelen=16及pfifo_head_drop limit=16。
队列满时丢最旧报文，避免原1000帧队列积压；该配置需要ip/tc及相应权限。
控制板/电调重上电时，配套板端固件必须拒绝未握手的速度指令。

这是DJI板和电调单独断电、USB-CAN适配器持续在线的恢复流程。
USB-CAN设备被拔插并重建接口时，现有socket不会自动重绑定，需要另行重启底盘节点。
