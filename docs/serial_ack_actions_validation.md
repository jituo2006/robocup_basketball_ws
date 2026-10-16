# 自动机构串口逻辑验证记录（2026-10-17）

工程：`/home/user/robocup_basketball_ws`。软件检查使用模拟MCU、假时钟以及独立ROS域198/199的PTY，未打开真实串口、未发送实机动作、未重启实际机器人服务。未修改下位机固件，未烧录。

## 结果

- 最后一次直接相关回归：**311项全部通过**，含两项独立ROS域PTY。日志`test_artifacts/serial_actions_20261016/affected_final.log`。
- 新增协议及真实串口节点PTY用例：**58项通过**（56项假时钟/模拟事件，2项真实rb_launcher+ROS+PTY，含拆包）。
- 14个相关测试文件回归：**417项通过、20项已有测试失败**。当前失败集合已逐项与修改前备份比较，全部可在备份中复现；本次未修改场地、路线和速度参数来迎合旧断言。
- 其中3项涉及旧的场地坐标/搜索条件，11项涉及已有速度/连续行驶变化，6项为旧的投篮入口测试临时目录未复制`transit_motion.py`。后者在真实工程目录中依赖文件存在，与实机入口缺失文件不同。
- 新增协议测试、已更新的捡球/投篮分发、计时回位、镜像入口测试均通过。原传球出球配置与备份逐字段一致，JSON中除shoot命令/固定等待数组之外的字段均未改变。
- Python AST解析、LF行尾检查及`git diff --check`通过。

## 关键覆盖

50 ms确认重发、20次上限、1秒窗口；ACK前DONE拒绝；ACK/DONE连包；上升DONE后1秒保持；MOVING/HOMING等待和HOLD幂等完成；命令错误/无回复跳过；独立15秒DONE等待；取消不再启动下降；投篮各命令确认重试；停止后双ID新鲜静止与零指令；home=0拒绝发射；编码器异常/INVALID结果不继续复位；5秒发射/复位/home等待；重复/旧run拒绝；home通知丢失后的待机查询；断线、连接代次改变、事件丢失、其他控制源插入；服务不可用、无回复发布器、在途服务超时不排队后续动作。

## 日志与备份

- `test_artifacts/serial_actions_20261016/protocol_tests.log`：新增58项。
- `test_artifacts/serial_actions_20261016/regression_final.log`：417通过/20已有失败。
- 同目录`baseline_tests.log`、`baseline_speed_tests.log`、`baseline_shoot_console.log`：修改前备份复现3+11+6项。
- `backups/serial_actions_20261016_233416/`：修改前文件、Git状态、HEAD及差异；新增测试后仍保留备份。

无需重编译C++。终止旧任务并重新启动比赛任务才能加载新的Python模块和配置。实机捡球、投篮及定时回程结果均未验收；home=1是当前主板的定时假定，未增加机械底部检测。本记录对应实现和软件验证阶段；提交和推送情况以后续仓库记录为准。
