# 传球控制：桌面手动启动与设备状态检查

2026-10-16，已部署至user-NUC12的/home/user/robocup_basketball_ws。

## 使用

开机及登录后不再自动运行六按键前端。桌面/home/user/桌面/robocup-pass-console.desktop显示为“传球控制”，已设置执行权限及metadata::trusted=true。点击后启动原8090后端并打开Chrome独立窗口；随后按雷达连接状态准备整车程序。雷达和底盘均就绪时，再选择A/B场地的回合1/2/3。

目前前端服务已停止（inactive）且禁用自启动（disabled），整车服务inactive/static，等待用户手动打开。旧桌面登录自启动已设置Hidden=true和X-GNOME-Autostart-enabled=false。原安装命令install_pass_console_autostart.sh也改为转入手动安装器，避免日后重新开启自启动。

PTP独立系统服务的开机启动保持原配置。手动打开前端后，后台的雷达连接检测仍会准备整车；不会自动选择或执行回合。关闭浏览器不会强行结束任务；服务保持到关机或手动停止。

## 状态与执行检查

- 新增只读ROS状态监测，订阅/lidar/session、/localization/ok、/chassis/ready、/chassis/status，不发布运动或使能指令。
- 雷达需要有效新会话和定位；底盘需要真实运行模式、四路电调反馈/速度模式/使能握手、最近匹配的板端状态应答。can0存在或UP并不等于底盘正常。
- 页面顶部显示正常/未就绪和具体原因；初始、异常、状态过期、网页连接中断时六按钮禁用。
- POST /api/pass/run在设备未就绪时返回409和原因；在任务交接完毕、子进程真正启动前再检查一次。设备正常时沿用原六种脚本映射及任务交接逻辑。
- 新增GET /api/hardware/state；GET /api/pass/state带hardware字段。所有前端响应禁用缓存。
- 设备状态2秒新鲜度、板端诊断1秒应答新鲜度仅用于新回合的准入，不改变/chassis/ready原无超时语义，不使运动指令过期，不因网页失联调用停车或取消正在执行的任务。已有明确硬件故障处理保持原逻辑。

## 验证与部署

- 更新21个文件，SHA256逐项核对；312个范围外源码文件校验未变，保留现场路线及点位配置。未提交或push，主机HEAD保持2d88f20。
- 已重新编译并安装rb_chassis，确认安装产物包含/chassis/status。
- 404项Python回归检查、3项C++ CTest全部通过。覆盖六种任务API拒绝/放行、状态过期和恢复、模拟模式、缺少应答、任务交接期间故障、原任务/底盘无超时策略。
- 在独立ROS_DOMAIN_ID=177、ROS_LOCALHOST_ONLY=1验证真实消息订阅及新任务准入；只执行模拟Python命令，不调用机器人路线。
- Node执行实际网页脚本，验证初始/异常/过期/断线禁用、六种正常选择及后端拒绝原因显示。
- 临时HOME配合替代systemctl/gio执行真实安装器，验证重复安装及旧兼容入口均保持手动启动。
- 在浏览器检查桌面与390px窄屏布局；模拟异常时六按钮禁用，模拟正常时全部开放，并完成一次只打印fixture信息的B2演示。截图ui_not_ready.jpg、ui_ready.jpg均为模拟设备状态。
- 短暂手动启动真实8090服务进行只读核对：雷达网口未连接、底盘等待程序建立通信，can_run=false，没有状态监测错误；核对后关闭服务。未向生产回合接口提交启动请求，没有启动真实底盘/雷达节点或行驶。

本次未做实机上电、扭矩或路线验证，未烧录DJI板。物理设备仍需现场确认。当前实际状态记录在live_frontend_state.json。

## 备份

修改前源码、底盘安装产物及桌面/服务配置：
/home/user/robocup_basketball_ws/backups/console_desktop_20261016-160807

安装器另存原服务及入口：
/home/user/robocup_basketball_ws/backups/console-desktop-20261016-162326-7iw1

构建、404项回归、CTest、安装与最终校验日志均保存在上述第一份备份目录；暂存位于/home/user/robocup_console_stage_20261016-160807。新配置和原备份均已保留，未重置其他源码。
