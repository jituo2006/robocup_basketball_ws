# 出错继续执行的投篮串口流程（2026-10-17）

按现场要求，自动投篮依次尝试：`vesc_stop → vesc_home → vesc_shoot2 3 → vesc_stop → vesc_reset → vesc_stop`。普通拒绝、无回复、异常结果、完成等待超时仅记录该步骤，继续后面的命令和路线；不再增加静止状态查询，不再等待home=1。

确认规则保持50 ms重发、最多20次/1秒，收到确认停止重发；已接受的发射与复位分别最多等5秒完成。最后停止会清除home，流程结束时home=0不是主机阻塞条件。`round_release_completed`仅表示全部步骤已尝试，确认结果仍单独保留。

## 软件检查

- **325项全部通过**，包含独立ROS域198/199的真实rb_launcher+PTY测试；日志：`test_artifacts/serial_continue_20261017/tests.log`。
- 原样重放实板的`RESULT FINISHED run=1 reason=shaft encoder invalid/stale timing_speed=INVALID`，确认仍尝试停止、复位、末尾停止。
- 覆盖逐条无回复/ERR、全部命令均错误、缺失完成结果/报告尾部、复位异常、home通知缺失、三回合出错后继续后续路线、旧五命令配置自动补末尾停止、用户取消仍可退出。
- 原传球配置及现场其他JSON字段与本次备份一致；未改动定位配置。
- AST/LF和`git diff --check`通过。

本次只改远程主机Python源码、配置、测试和说明。未操作实机、未重新烧录、未重启真实服务。结束旧任务进程后重新启动比赛任务加载新逻辑，不需重编译。未自动提交或推送。

修改前备份：`backups/serial_continue_20261017_011007/`。此前`serial_ack_actions_validation.md`记录的是上一版，当前行为以本记录和`serial_ack_actions.md`为准。
