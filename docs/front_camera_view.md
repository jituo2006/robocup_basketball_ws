# 在Windows同页查看远程前后摄像头

本工具只订阅主机当前的前后图像和各自识别结果，不打开USB设备、不重复运行模型、不发布底盘或机构命令。现有相机/识别节点需要已启动。

## 当前电脑直接使用

双击Windows的 `E:\BasketballCode\查看前置摄像头.cmd`，或运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "E:\BasketballCode\查看前置摄像头.ps1"
```

打开 `http://127.0.0.1:18082`，同页显示前摄和后摄，宽窗口并排、窄窗口上下排列；各自有识别框开关、实时状态和结果表。原Windows启动脚本继续可用。脚本通过SSH别名`robocup-linux`连接Linux，检查/启动只读查看服务，再建立SSH端口转发并打开默认浏览器。无需在Windows安装ROS、OpenCV或模型。

这里使用本地转发是因为本机代理直接访问`http://100.82.36.53:8082`时返回502；绕过代理的直连已验证200。网络断开后重新运行启动脚本即可；窗口显示的SSH PID可用`Stop-Process -Id PID`关闭转发。

仓库里对应的Windows文件是`tools/open_front_camera.cmd`和`tools/open_front_camera.ps1`，下载到同一目录，双击CMD即可。默认主机别名需已配置SSH密钥，与本机现有连接相同。可通过`-HostAlias`、`-LocalPort`、`-RemotePort`覆盖；`-NoBrowser`只建立连接，不自动开浏览器。

## 在Linux主机直接运行

```bash
cd /home/user/robocup_basketball_ws
bash tools/view_front_camera.sh
# 可选：bash tools/view_front_camera.sh --port 8082 --fps 6 --width 960 --quality 75
```

默认ROS_DOMAIN_ID=2，与比赛节点一致。默认地址`0.0.0.0:8082`，可用`--host 127.0.0.1`只监听本机。该脚本不会启动比赛、摄像头或识别节点；未收到数据时网页显示等待提示。

- 前摄图像：`/camera/image_raw/compressed`，识别：`/perception/detections`。
- 后摄图像：`/camera_rear/image_raw/compressed`，识别：`/perception/detections_rear`。
- 前摄可用`--image-topic`、`--detections-topic`替换；后摄可用`--rear-image-topic`、`--rear-detections-topic`替换。
- `/`双摄网页；`/status`含`cameras.front`和`cameras.rear`的实时状态，同时保留顶层前摄字段兼容原启动脚本。
- `/frame.jpg?camera=front`前摄叠框、`/frame.jpg?camera=rear`后摄叠框，追加`&overlay=0`关闭该路识别框。不指定camera时仍返回前摄。
- `/status?camera=rear`将顶层字段切换到后摄；未知camera返回404。

网页显示类别、置信度、估计距离和方位角；支持关闭识别框。按时间戳匹配对应推理图像，避免把旧框画到新画面；识别/图像超过1秒未更新会显示状态并清除旧目标。单目距离仅为估计，方位角右正左负。

每路默认每秒最多编码6次、最长边按图像宽度限制为960，保持固定长度缓存。不会新增推理负载；对图像订阅使用BEST_EFFORT/depth=1兼容当前相机QoS。

## 验证

17项测试通过：模拟JPEG/检测数据、双摄图像/识别隔离、单路过期不影响另一路、HTTP路由与叠框开关、无数据占位、时间戳匹配、无效框和NaN，以及独立ROS域202的双路端到端HTTP查看。

日志：`test_artifacts/front_camera_view/dual_tests.log`。还验证了真实前后摄像头和各自识别节点输出，以及Windows通过SSH转发访问`/status`返回200；只启动查看服务，没有运行机器人动作。

当前后台查看服务日志：`test_artifacts/front_camera_view/server.log`。本工具是单次启动，未增加开机启动项或计划任务；主机重启后重新运行Windows启动脚本即可。

双摄改动前备份：`backups/dual_camera_20261017_014118/`。只重启查看服务，原摄像头、感知及机器人控制节点保持运行。
