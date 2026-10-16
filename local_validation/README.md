# Windows本地模拟验证

include是Linux socket/CAN类型的最小替代头；controller_include是ROS/motor_lib边界替代。
它们仅用于本目录的Windows测试，不得加入生产库的include目录。
生产Linux CTest使用系统Linux头文件。

依赖：g++ (C++17)、Python及pytest/PyYAML。仓库根目录可运行run_native_checks.ps1；
Python测试使用PYTHONPATH=src/rb_mission;src/rb_localization;tools;src/rb_tests/test。
run_protocol_tests.py可以额外传入主机源码根目录和配套固件根目录。

本地模拟不能验证USB驱动、真实内核队列、终端电阻、电源或3508扭矩。
