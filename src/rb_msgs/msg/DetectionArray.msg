std_msgs/Header header
uint32 frame_id_seq

# 相机内参（来自标定；未标定时为 0）
float32 fx
float32 fy
float32 cx_cam
float32 cy_cam
uint32 img_width
uint32 img_height

Detection[] detections
