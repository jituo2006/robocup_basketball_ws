std_msgs/Header header

# 任务阶段（与 rb_mission 的状态机一致）
string phase

bool has_ball               # 是否持球
uint8 ball_count            # 持球数量（规则：单次持球 >2 颗违规）
string ball_type            # ball_basketball / ball_volleyball / none

bool in_pass_zone           # 是否在传球区内（决定传球得分 10 分 / 5 分）
bool in_shoot_zone_outside  # 是否在投篮边界线外（决定投篮 10 分 / 5 分）

bool estop                  # 急停/自主拍停是否触发
bool localization_ok        # 定位是否可用
bool launcher_ok            # 发射机构串口是否正常
bool perception_ok          # 视觉是否正常
