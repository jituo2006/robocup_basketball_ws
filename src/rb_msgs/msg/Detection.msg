# 单个视觉检测目标（归一化图像坐标 + 方位角 + 粗略距离）
builtin_interfaces/Time stamp

# 类别标签。约定取值：
#   ball_basketball / ball_volleyball / ball_unknown   —— 球
#   hoop          —— 迷你篮筐（含篮圈中心）
#   pass_rack     —— 传球训练架（rack_ring 为其顶部圆环）
#   rack_ring     —— 传球架顶部圆环（50 分目标）
#   pillar        —— 定位柱（规则提供的视觉基准）
#   obstacle      —— 干扰球
string label

float32 confidence          # 0..1

# 归一化 bbox（0..1，左上角为原点）
float32 x_min
float32 y_min
float32 x_max
float32 y_max

float32 cx                  # 中心 x（归一化）
float32 cy                  # 中心 y（归一化）
float32 width               # 归一化宽
float32 height              # 归一化高

# 像素坐标（便于直接画图/复算）
float32 px
float32 py
float32 bbox_px_w
float32 bbox_px_h

# 几何估计
float32 bearing_rad         # 水平方位角，相对相机光轴，左正右负（由标定决定）
float32 distance_m          # 距离估计，未知填 NaN
float32 diameter_m          # 目标物理直径（用于单目测距，未知填 NaN）
