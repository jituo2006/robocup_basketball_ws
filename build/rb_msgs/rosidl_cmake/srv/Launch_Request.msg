# 发射机构动作。
#
# 协议来自 ROBOCON 2025 ballrobot 的 Message::FrameControl：
#   DRIBBLE=0x80  DOUBLE_DRIBBLE=0x81  SHOOT=0x40  FAST_SHOOT=0x41
#   KEEP_LOOP=0x42  PAWL=0x20  SLIDEWAY=0x10  RESPONSE=0x60
#
# 注意：PAWL（夹爪）和 SLIDEWAY（滑轨）在**下位机侧是开关语义**，
#       每发一次就翻转一次状态，所以这里只有 TOGGLE 动作。
uint8 action
uint16 speed        # 转速 rpm；0 表示沿用节点默认值
uint16 angle        # 角度；0 表示沿用节点默认值
