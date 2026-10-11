# YOLO篮球与排球画框

控制台已有的前、后摄MJPEG视频流默认叠加篮球/排球检测框、中文类别和置信度。
篮球为橙色，排球为青色。框使用现有YOLO检测的归一化图像坐标，不额外运行模型。
雷达候选、其他类别、低置信度、非法框、过期检测以及与图像时间不符的检测不画框。
相机断流超过1.5秒后不继续发送旧画面。关闭画框后直接透传原JPEG字节。
同一帧的绘制结果在多个浏览器之间复用，避免重复解码编码。

后端开关，无需增加前端控件：

```bash
# 查看状态
curl http://127.0.0.1:8090/api/camera_overlay
# 运行时关闭两路画框，前端连接无需重建
curl -X POST http://127.0.0.1:8090/api/camera_overlay \
  -H 'Content-Type: application/json' -d '{"front":false,"rear":false}'
# 运行时开启两路
curl -X POST http://127.0.0.1:8090/api/camera_overlay \
  -H 'Content-Type: application/json' -d '{"front":true,"rear":true}'
# 启动时关闭
bash tools/console.sh --no-ball-boxes
```

持久默认值修改 `tools/console_overlay.yaml`；可独立设置两路开关、置信度阈值、
检测时限、图像与检测源时间差上限和JPEG质量。HTTP更新仅作用于当前控制台进程。
这些开关不停止感知节点，不影响任务节点订阅的检测结果。
