# 主机与雷达同时冷启动：本地修改

工作目录：E:/BasketballCode/robocup_basketball_ws-host-20261016-135923

基线为2026-10-16 14:00主机源码快照e1cfd20（含采集时三个未提交源码修改）。本次继续保留本地底盘独立上电恢复修改，只补充雷达启动与恢复；没有连接、上传或修改Linux主机，没有启动真实ROS节点、驱动、车辆路线或烧录固件。

## 上电及就绪顺序

1. 主机、MID360可以同时供电。现有前端自动启动监控持续等待雷达网口链路及ping成功，再启动boot.sh。底盘可保持断电。
2. 在既有PTP安装完成的前提下，boot.sh启动PTP服务。lidar_session等待指定网口carrier=1、主机静态地址192.168.1.110出现、ptp4l与phc2sys都active，才启动Livox。任一条件晚出现时每秒重查，不因累计等待时间退出。
3. 点云和IMU均有有效真实源时间、连续稳定1秒，才启动FAST-LIO。新里程计连续稳定1秒，才发布/lidar/session的ready=true。服务active不作为PTP锁定证据。
4. 相机、定位、底盘等节点与雷达管理器并行运行。定位接入有效雷达会话；底盘按独立握手逻辑就绪。两项就绪后再通过前端选择任务。

上电顺序的软件约束由条件等待替代人工固定延时。物理供电压降、网线/交换机、雷达本体故障或错误网络配置仍不能由软件消除。

## 失效恢复

- Livox启动后60秒一直缺少任一路有效输入：回收旧FAST-LIO（若存在），重启Livox，重新等待数据。
- 已有数据后持续10秒断流，或30秒无有效源时间进展：同样重启雷达驱动。重复旧帧、未同步时间不能维持有效进展。
- 输入有效但FAST-LIO启动30秒没有有效里程计，或里程计停滞10秒：只重建FAST-LIO会话。
- 进程退出或启动OSError也可恢复。失败重试间隔1、2、4、8、15秒，之后保持15秒；完全就绪后复位。
- 短暂数据延迟只撤销ready并等待新数据，不频繁杀进程。明确异常、长时间失效和重新启动都会撤销旧会话；新输出稳定后才能重新接入。
- 主机wall clock相对monotonic跳变超过0.5秒时，撤销旧会话、回收FAST-LIO、清理旧数据，保留Livox进程等待PTP重新收敛；新时间段数据稳定后再建立估计器。
- /lidar/session保留generation/ready/cutoff，新增phase/reason用于说明当前等待或恢复原因。

这些等待时间用于回收失效传感器进程，没有恢复用户取消的任务/底盘通信超时停车。不会自动执行路线，也不会自动继续已结束的任务。

## 部署前提与范围

必须已有正确的雷达网卡静态IP/autoconnect、雷达配置中的目的IP，以及硬件PTP安装。仓库外的/home/user/lidar_360/ws_livox和fast_prop_ws没有在此次源码快照内；本次未对它们做修改。

tools/install_ptp.sh及其外部Livox时间转换补丁必须实际安装并编译生效，步骤见docs/26_PTP时间同步.md。没有/etc/robocup/livox_ptp_utc_offset安装标记时，auto模式保留旧安装行为，boot.sh仍执行旧NTP检查；本次不会偷偷安装系统服务或改网卡配置。

新增lidar_startup.py已加入rb_bringup的CMake安装列表。以后部署需重新编译并安装rb_bringup，不能只替换源文件而沿用旧install。既有底盘修改的部署前提见LOCAL_CHASSIS_POWER_RECOVERY.md。

launch可配置lidar_interface、lidar_host_ip、lidar_ptp_mode；默认enp114s0、192.168.1.110、auto。修改参数不会自动修改网卡或Livox外部配置，三者需一致。

## 本地验证

- 352项Python检查通过：任务、定位、路线、底盘等待、前端自动启动，以及雷达健康度、生产管理器主循环和启动参数传递。
- 其中53项雷达生命周期/启动检查覆盖：网口、静态IP、PTP晚就绪；主机与雷达同时启动；雷达先于主机启动；12秒慢出数据；仅有点云；无有效源时间；驱动存活但无数据；长时间断流/重复旧帧；FAST-LIO存活但无输出；启动异常；驱动退出；系统时间前后跳变；持续失败的有界重试；等待期间退出回收。
- 冷启动测试执行实际lidar_session.main，替换ROS、系统调用与进程边界并推进逻辑时间，不以固定脚本直接模拟预期结果。启动探测器另外用临时网口文件和systemctl/ip返回值验证，包括“systemctl返回0但只有一个服务active”的情况。
- launch检查执行实际generate_launch_description（ROS launch边界替代），再由真实bash验证参数逐项传递；未启动ROS。
- boot.sh、run_ptp4l.sh、install_ptp.sh的bash -n检查通过。
- 6项旧检查未纳入此Windows结果：3项/proc模拟需要Windows无权创建的符号链接，2项Linux shell PATH夹具，1项Linux默认C++动态链接运行检查。未改写这些旧测试以伪造通过；这6项及真实ROS launch渲染、Linux整包构建须在Linux环境执行。

Windows本地不具备Linux ROS2整包构建环境。以上不是实机冷启动、真实PTP锁定或雷达定位精度的验证，也未证明电源硬件能稳定同时带载。

现场至少验证多次同时冷启动、无互联网冷启动、雷达延迟上电、底盘单独循环断电；观察/lidar/session阶段、新源时间戳及唯一Livox/FAST-LIO进程，再在定位和底盘均就绪后人工选择路线。

LOCAL_LIDAR_COLD_START.patch仅包含雷达部分相对原下载源码的差异，不重复打包之前底盘修改；文件清单见LOCAL_LIDAR_COLD_START_FILES.txt。原始下载归档和基线校验文件保持不变。
