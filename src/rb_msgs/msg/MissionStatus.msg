std_msgs/Header header
string mission              # IDLE / PASS / SHOOT
string phase                # 细分阶段名
string detail               # 人类可读说明
float32 elapsed_s           # 当前任务已用时
int32   score_estimate      # -1=未知；服务动作计数不代表出球/命中/官方得分
