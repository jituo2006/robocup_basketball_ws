# MID360 硬件 PTP

Linux 的 NTP 校时与雷达同步是两件事。这里使用专用雷达网口 `enp114s0`：主机作为 IEEE1588v2 UDP/IPv4 PTP 主时钟，MID360 自动跟随。网卡必须支持硬件收发时间戳；安装脚本用 `ethtool -T` 检查，不静默回退软件时间戳。只服务雷达有线网络，不使用 Wi-Fi。

## 一次安装

停止 Livox 驱动后，使用普通工作区用户运行（需要免密 sudo）：

```bash
cd /home/user/robocup_basketball_ws
bash tools/install_ptp.sh
```

脚本安装 linuxptp/ethtool、给外部 Livox 驱动应用可重复补丁并编译，再安装并启用两个 systemd 服务。不会启动 ROS、底盘或路线，不更改 NetworkManager/IP。雷达断电或拔线时服务等待链路和 IPv4，重新连接后恢复。

- `robocup-ptp4l@enp114s0.service`：硬件时间戳、域 0、E2E、只做主时钟，Sync 周期 125 ms。
- `robocup-phc2sys@enp114s0.service`：系统 UTC → 网卡 PHC/TAI；使用 `-w` 从 ptp4l 读取 UTC 偏移。只调整网卡时钟，不与 NTP 争用系统时钟。
- `/etc/robocup/ptp4l.conf` 与 `/etc/robocup/livox_ptp_utc_offset` 使用同一 TAI−UTC 偏移（安装参数 `--utc-offset`，当前为 37 秒）。以后闰秒偏移改变时，停止 ROS 后重新安装并重启驱动。

## ROS 时间基准

上游 Livox 驱动直接复制 PTP 时间戳，未同步时则代用主机收包时间。我们的补丁在启用配置后只接受 `time_type=1` 的数据，并同时给点云和 IMU 减去 TAI−UTC 偏移。保留逐点相对时间，不改雷达/IMU 的相对采样间隔。

未锁定 PTP 时不把收包时间伪装为测量时间；管理器沿用既有数据就绪逻辑，待真实输入连续稳定后启动 FAST-LIO。运行中失去同步会断流并撤销旧定位会话；已经退出的路线需重新启动。FAST-LIO 保持 `time_sync_en: false`、`time_offset_lidar_to_imu: 0.0`，不叠加软件补偿。

`boot.sh` 在 PTP 已安装时检查服务，取消依赖互联网 NTP 的 45 秒启动阻塞。没有互联网时传感器仍可跟随本机时钟；系统绝对时间未必精确。后续系统时钟跳变仍由会话管理检测并重建估计器。

## 主机与雷达同时上电

软件按条件推进，不要求人为间隔几秒：

1. 主机、雷达一起上电；网口连接后由现有 NetworkManager 自动设置 `192.168.1.110/24`。手动点击桌面“传球控制”后，前端后台检查持续等待雷达能响应，再准备整车软件；前端不再开机或登录自启动。
2. `lidar_session.py` 再检查 `enp114s0` 链路、主机地址和两个 PTP 服务。服务尚未运行、地址尚未分配时持续等待，每秒重查；不会把服务 active 当成同步成功。
3. 启动 Livox，等点云和 IMU 的真实源时间有效、连续稳定 1 秒，再启动 FAST-LIO；新里程计连续稳定 1 秒后才发布会话 ready，定位节点才接入。
4. 底盘 DJI 板与四个电调可以随后一起上电。雷达定位、底盘握手都就绪后，再按前端任务按钮。

驱动存活但启动后 60 秒仍缺少任一有效输入，会先回收估计器、再重启驱动。运行中连续 10 秒无数据，或连续 30 秒没有有效源时间进展，同样进入恢复。FAST-LIO 输入有效但启动 30 秒仍无有效输出，或有效输出持续停滞 10 秒，只重启 FAST-LIO。进程启动失败也继续重试，重试间隔按 1、2、4、8、15 秒增长，成功就绪后复位。上述时间控制传感器进程恢复，不新增任务或底盘命令的超时停车。

短暂数据延迟撤销当前 ready，等新数据恢复；长时间故障、进程退出、源时间倒退、位置异常或主机时钟跳变会重建会话。主机时钟跳变时保留驱动，让 PTP 重新收敛，丢弃旧时间段数据并重新建立 FAST-LIO 会话。不会自动执行比赛路线；已退出的任务仍需重新点击。

`/lidar/session` 的 JSON 额外提供 `phase`、`reason`，可以看到正在等待网络、PTP、输入、里程计或重试。launch 参数 `lidar_interface`、`lidar_host_ip`、`lidar_ptp_mode` 默认分别为 `enp114s0`、`192.168.1.110`、`auto`。`auto` 根据 PTP 安装标记决定是否检查服务；`on` 强制检查；`off` 仅供旧安装联调，不会配置或关闭 PTP。

这套顺序依赖本节的一次安装已完成：静态 IP/autoconnect、硬件 PTP 服务、外部 Livox 驱动补丁和重编译。没有 PTP 安装标记时，仍走旧版 NTP 启动检查；只复制本仓库不会自动安装外部驱动或系统服务。上电流程的软件重放已覆盖晚出数据和进程卡住；实际供电稳定性及同时冷启动仍需在机器上验证。

## 验证

上电并启动 Livox 后（不需要启动车辆路线）：

```bash
sudo python3 tools/check_ptp.py
journalctl -u robocup-ptp4l@enp114s0 -u robocup-phc2sys@enp114s0 -n 30
```

检测脚本通过 AF_PACKET 被动读取雷达网口，不绑定 56301/56401，不抢驱动数据。分别检查点云、IMU 的真实设备时间类型和转为 UTC 后的收包时间差。服务 active 只表示进程运行，不能证明雷达已同步；网口断开时返回“尚未验证”。收包时间差包含网络/调度延迟，不能当作纳秒级伺服同步精度。

需要现场验证首次锁定、连续运行、断电/插拔恢复、ROS 源时间戳和行驶表现；代码测试不能替代这些验证。

## 回退

停止 ROS 后：

```bash
sudo systemctl disable --now robocup-phc2sys@enp114s0 robocup-ptp4l@enp114s0
sudo mv /etc/robocup/livox_ptp_utc_offset /etc/robocup/livox_ptp_utc_offset.disabled
```

重启驱动后，缺少激活配置会恢复上游时间戳行为。补丁前源码保存在外部驱动的 `src/comm/pub_handler.cpp.before_robocup_ptp`，如需彻底还原可以恢复该文件、删除新增头文件，再重新编译。原雷达网络配置、其他自定义源码和硬件参数不在补丁范围。

依据：[Livox 同步说明](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)、[MID360 时间戳协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)、[linuxptp 的 UTC/TAI 与 phc2sys](https://www.linuxptp.org/documentation/phc2sys/)。
