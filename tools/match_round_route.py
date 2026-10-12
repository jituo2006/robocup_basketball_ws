"""A 区出发的三个回合实验；复用固定路线的 ROS/后摄/吸球实现。

drive: 空场走搜索轨迹、动作点和回位，不取球、不发送机构指令。
collect: 寻一颗目标球、转身对正吸球、退回通道、到动作点停车。
cycle: 放球联调或配置后正式出球两次并回位；第一回合先处理预装球。
所有场地坐标通过启动位姿建立相对变换，不改定位原点。
"""
import json
import math
import time
from pathlib import Path
from visual_target_alignment import align_pillar

CONFIG = Path(__file__).with_name('match_rounds.json')
TASK_LABELS = {'pass': 'ball_basketball', 'shoot': 'ball_volleyball'}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class StartFrame:
    """名义 A 起点朝 +X，对应当前真实位姿；只变换规划，不重置定位。"""
    def __init__(self, pose, start):
        self.pose, self.start = tuple(pose), tuple(start)

    def point(self, p):
        dx, dy = p[0]-self.start[0], p[1]-self.start[1]
        x, y, yaw = self.pose
        return (x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                y+math.sin(yaw)*dx+math.cos(yaw)*dy)

    def yaw(self, yaw):
        return wrap(self.pose[2]+yaw)

    def nominal(self, p):
        dx, dy = p[0]-self.pose[0], p[1]-self.pose[1]
        yaw = self.pose[2]
        return (self.start[0]+math.cos(yaw)*dx+math.sin(yaw)*dy,
                self.start[1]-math.sin(yaw)*dx+math.cos(yaw)*dy)


def load_config(path=CONFIG):
    with open(path, encoding='utf-8') as stream:
        cfg = json.load(stream)
    def number(value, lo, hi):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'路线参数不在范围 [{lo}, {hi}]: {value!r}')
    def point(value, start=False):
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError('路线坐标须为 [x,y]')
        number(value[0], 0, 14)
        number(value[1], 0, 8.5 if start else 7.5)
    point(cfg['start'], start=True)
    for key in ('entry', 'corridor'):
        point(cfg[key])
    number(cfg['speed_mps'], .01, .25)
    number(cfg['start_delay_s'], 5, 15)
    if cfg.get('round1_first_ball', 'preloaded') not in ('preloaded', 'search'):
        raise ValueError('首球方式须为 preloaded/search')
    vision = cfg.get('pillar_alignment', {})
    if vision:
        if not isinstance(vision.get('enabled'), bool):
            raise ValueError('pillar_alignment.enabled 须为布尔值')
        for key, lo, hi in (('confidence', .25, 1), ('tolerance_deg', .2, 5),
                            ('stable_frames', 1, 10), ('timeout_s', .5, 15),
                            ('max_turn_deg', 1, 30), ('max_ang_rad_s', .02, .25),
                            ('kp', .1, 3), ('camera_yaw_offset_rad', -math.pi, math.pi),
                            ('launcher_yaw_offset_deg', -180, 180)):
            number(vision[key], lo, hi)
        if not isinstance(vision['stable_frames'], int):
            raise ValueError('定位柱稳定帧数须为整数')
    m, a, i = cfg['middle'], cfg['arc'], cfg['inner']
    number(m['y'], 3.8, 6)
    number(m['left_x'], .3, 1.5)
    number(m['right_x'], 1.6, 5)
    point(a['center'])
    number(a['radius'], 3.8, 5)
    if not isinstance(a['angles_deg'], list) or len(a['angles_deg']) < 2:
        raise ValueError('弧线至少需要两个角度')
    for angle in a['angles_deg']:
        number(angle, 90, 270)
    for p, _ in search_plan(cfg, 'arc'):
        point(list(p))
    number(i['x'], 7.5, 8.8)
    number(i['top_y'], 3.8, 6.8)
    number(i['bottom_y'], .7, 3.7)
    for area in (m, a, i):
        number(area['scoop_m'], .03, 2)
    if not isinstance(cfg['drop_commands'], list) or not cfg['drop_commands'] or any(
            not isinstance(c, str) or not c.strip() or '\n' in c or '\r' in c for c in cfg['drop_commands']):
        raise ValueError('放球指令须为非空单行字符串列表')
    for task, action in cfg['release'].items():
        if task not in TASK_LABELS:
            raise ValueError('动作类型须为 pass/shoot')
        point(action['point'])
        if action['aim'] is not None:
            point(action['aim'])
            if action['aim'] == action['point']:
                raise ValueError('动作点不能与瞄准目标重合')
        number(action['wait_s'], 0, 30)
        number(action.get('search_heading_deg', 0), -180, 180)
        if not isinstance(action['commands'], list) or any(
                not isinstance(c, str) or not c.strip() or '\n' in c or '\r' in c
                for c in action['commands']):
            raise ValueError('出球指令须为非空单行字符串列表')
    if set(cfg['release']) != set(TASK_LABELS):
        raise ValueError('需要同时配置传球、投篮动作点')
    # 点的合法性只检查规划中心，不代替整机包络的实测。
    p = cfg['release']['pass']['point']
    q = cfg['release']['shoot']['point']
    if math.hypot(p[0]-2.5, p[1]-3.75) >= 2.5:
        raise ValueError('传球动作点须在传球圆内')
    if math.hypot(q[0]-12.425, q[1]-3.75) <= 2:
        raise ValueError('投篮动作点须在投篮边界外')
    return cfg


def validate_mode(cfg, task, mode, release_kind='match', mechanism_enabled=True):
    if task not in TASK_LABELS or mode not in ('drive', 'collect', 'cycle'):
        raise ValueError('任务/实验模式无效')
    if release_kind not in ('drop', 'match'):
        raise ValueError('出球模式须为 drop/match')
    if not isinstance(mechanism_enabled, bool):
        raise ValueError('机构开关须为布尔值')
    if mode == 'cycle' and release_kind == 'match' and mechanism_enabled:
        action = cfg['release'][task]
        if not action['commands'] or action['aim'] is None:
            raise ValueError('双次流程须先在 tools/match_rounds.json 配置真实出球 commands 和 aim；可先用取球实验')


def search_order(round_number, task):
    if round_number == 1:
        return ['middle']
    if round_number == 2:
        return ['middle', 'arc']
    if round_number == 3:
        return ['arc', 'inner'] if task == 'pass' else ['inner', 'arc']
    raise ValueError('只支持第1、2、3回合')


def search_plan(cfg, area, cycle=0):
    """(点, 前摄朝向)。中线始终从规则图右端向左扫；其它区域第二次换向。"""
    if area == 'middle':
        m = cfg[area]
        return [((x, m['y']), -math.pi/2) for x in (m['right_x'], m['left_x'])]
    if area == 'inner':
        i = cfg[area]
        ys = [i['top_y'], i['bottom_y']] if cycle == 0 else [i['bottom_y'], i['top_y']]
        return [((i['x'], y), 0.) for y in ys]
    a = cfg['arc']
    angles = a['angles_deg'] if cycle == 0 else list(reversed(a['angles_deg']))
    return [((a['center'][0]+a['radius']*math.cos(math.radians(v)),
              a['center'][1]+a['radius']*math.sin(math.radians(v))),
             wrap(math.radians(v)+math.pi)) for v in angles]


class RoundRoute:
    def __init__(self, runner, cfg, round_number, task, mode, emit=print, release_kind='match'):
        validate_mode(cfg, task, mode, release_kind,
                      mechanism_enabled=getattr(runner.args, 'route_intake', True))
        self.r, self.cfg = runner, cfg
        self.round, self.task, self.mode = round_number, task, mode
        self.release_kind = release_kind
        self.emit = emit
        self.frame = StartFrame(runner.pose, cfg['start'])
        runner.round_start_frame = self.frame
        runner.keep_intake_after_scoop = True
        self.trail = []
        self.search_origin = None
        self.target_aligned = False
        self.last_error = ''
        if round_number == 1 and release_kind == 'match' and cfg.get('round1_first_ball', 'preloaded') != 'preloaded':
            raise ValueError('规则第1轮首球须预装；场上取首球仅用于放球联调')

    def say(self, message):
        self.emit(f'第{self.round}回合/{self.task}/{self.mode}: {message}')

    def move_world(self, p, remember=False):
        distance = math.hypot(p[0]-self.r.pose[0], p[1]-self.r.pose[1])
        ok = self.r.goto(*p, timeout=max(20., distance/self.r.args.speed+15.),
                         tolerance=self.r.args.step_tol)
        if ok and remember:
            self.trail.append(tuple(p))
        return ok

    def move(self, p, remember=False):
        return self.move_world(self.frame.point(p), remember)

    def turn(self, heading):
        return self.r.rotate_to(self.frame.yaw(heading), timeout=45.)

    def scan(self, target, heading, detect=True):
        """沿一段搜索线保持朝向；中心框触发立即停；区分无球与动作失败。"""
        target, heading = self.frame.point(target), self.frame.yaw(heading)
        distance = math.hypot(target[0]-self.r.pose[0], target[1]-self.r.pose[1])
        deadline = time.monotonic()+max(15., distance/self.r.args.speed+10.)
        while time.monotonic() < deadline:
            self.r.spin(.03)
            if detect and self.r.centered_target_candidates(self.r.dets) is not None:
                self.r.stop()
                self.r.spin(getattr(self.r.args, 'settle_seconds', .3))
                self.search_origin = tuple(self.r.pose)
                self.trail.append(self.search_origin[:2])
                self.say('目标球到画面中心，停车 → 转180° → 后摄横移对正')
                return 'found'
            x, y, yaw = self.r.pose
            dx, dy = target[0]-x, target[1]-y
            remaining = math.hypot(dx, dy)
            if remaining <= self.r.args.step_tol:
                self.r.stop()
                # 到线端后留一点观察时间，避免最后一帧还未到达。
                self.r.spin(.3)
                if detect and self.r.centered_target_candidates(self.r.dets) is not None:
                    self.search_origin = tuple(self.r.pose)
                    self.trail.append(self.search_origin[:2])
                    return 'found'
                self.trail.append(tuple(target))
                return 'empty'
            err = wrap(heading-yaw)
            if abs(err) > self.r.args.route_heading_tol:
                self.r.stop()
                self.say('搜索朝向偏离，停止本次实验')
                return 'failed'
            speed = min(self.r.args.speed, self.r.args.kp*remaining)
            vx, vy = speed*dx/remaining, speed*dy/remaining
            tw = self.r._Twist()
            tw.linear.x = math.cos(yaw)*vx+math.sin(yaw)*vy
            tw.linear.y = -math.sin(yaw)*vx+math.cos(yaw)*vy
            wz = self.r.args.kp_yaw*err-self.r.args.kd_yaw*self.r.yaw_rate
            tw.angular.z = max(-self.r.args.max_ang, min(self.r.args.max_ang, wz))
            self.r.publish_command(tw)
        self.r.stop()
        self.say('搜索运动超时，停止本次实验')
        return 'failed'

    def unwind(self):
        """取球后回到检测点，再沿已经走过的折线退出，不直接穿过置球线。"""
        self.r.stop()
        while self.trail:
            p = self.trail[-1]
            if not self.move_world(p):
                return False
            self.trail.pop()
        return True

    def search_area(self, area, cycle, search_only=False):
        cfg = self.cfg
        self.say(f'搜索 {area}，目标类别 {TASK_LABELS[self.task]}')
        self.trail = []
        # 深场搜索统一经 E、T；不在绿色半场线找目标球。
        transit = [cfg['entry']]
        if area != 'middle':
            transit.append(cfg['corridor'])
            if area == 'inner':
                transit.append([cfg['inner']['x'], cfg['corridor'][1]])
        for p in transit:
            if not self.move(p, remember=True):
                return 'failed'
        points = search_plan(cfg, area, cycle)
        if area == 'middle':
            # 先朝黄线到 C，再平移到规则图右端；这两段不读取球框触发取球。
            if not self.turn(points[0][1]):
                return 'failed'
            if not self.move([cfg['start'][0], cfg['middle']['y']], remember=True):
                return 'failed'
            self.say(f"中线预移至图右端 X={cfg['middle']['right_x']:.2f}，到位后向图左端搜索")
        if not self.move(points[0][0], remember=True):
            return 'failed'
        for p, yaw in points:
            if not self.turn(yaw):
                return 'failed'
            status = self.scan(p, yaw, detect=self.mode != 'drive')
            if status != 'empty':
                if status == 'found':
                    self.area = area
                    if search_only:
                        return status
                    before = self.r.pose[2]
                    if not self.r.rotate_to(wrap(before+math.pi), timeout=45.):
                        return 'failed'
                    saved_distance, saved_timeout = self.r.args.route_scoop_distance, self.r.args.route_scoop_timeout
                    self.r.args.route_scoop_distance = cfg[area]['scoop_m']
                    self.r.args.route_scoop_timeout = max(15., cfg[area]['scoop_m']/self.r.args.route_scoop_speed+5.)
                    try:
                        if not self.r.rear_visual_scoop():
                            return 'failed'
                    finally:
                        self.r.args.route_scoop_distance, self.r.args.route_scoop_timeout = saved_distance, saved_timeout
                    self.say('取球动作完成，实际入仓仍需实物/传感器确认')
                if not self.unwind():
                    return 'failed'
                return status
        return 'empty' if self.unwind() else 'failed'

    def action_point(self, aim=True):
        cfg = self.cfg
        # 投篮点在绿线右边，过半场仍走上侧通道；传球点位于左侧传球圆内。
        if self.task == 'shoot':
            for p in (cfg['entry'], cfg['corridor']):
                if not self.move(p):
                    return False
        else:
            if not self.move([cfg['start'][0], cfg['middle']['y']]):
                return False
        action = cfg['release'][self.task]
        if not self.move(action['point']):
            return False
        if not aim:
            self.r.stop()
            return True
        if action['aim'] is not None:
            target = self.frame.point(action['aim'])
            heading = math.atan2(target[1]-self.r.pose[1], target[0]-self.r.pose[0])
        else:
            heading = self.frame.yaw(math.radians(action.get('search_heading_deg', 0)))
        if not self.r.rotate_to(heading, timeout=45.):
            return False
        self.r.stop()
        self.target_aligned = False
        vision = cfg.get('pillar_alignment', {})
        if (self.mode != 'drive' and getattr(self.r.args, 'route_intake', True)
                and vision.get('enabled', False)):
            self.say('雷达到动作点，前摄用定位柱微调出球方向')
            self.target_aligned = align_pillar(self.r, vision, heading, emit=self.say)
            if not self.target_aligned and self.release_kind == 'match':
                self.say('正式出球须先对准目标柱；本次停车')
                return False
        return True

    def release(self):
        action = self.cfg['release'][self.task]
        if not getattr(self.r.args, 'route_intake', True):
            self.say('机构开关关闭：跳过全部吸球/发射/放球指令，继续本轮后续路线')
            return True
        if (self.release_kind == 'match' and self.cfg.get('pillar_alignment', {}).get('enabled')
                and not self.target_aligned):
            self.say('尚未对准定位柱，先完成动作点对准')
            return False
        commands = self.cfg['drop_commands'] if self.release_kind == 'drop' else action['commands']
        self.r.stop()
        for command in commands:
            ok, message = self.r.intake_text(command)
            self.say(f'出球指令 {command}: {message}')
            if not ok:
                self.last_error = f'机构指令 {command} 失败：{message}'
                return False
            self.r.spin(action['wait_s'])
        self.say('放球联调完成，不计正式传球/投篮' if self.release_kind == 'drop'
                 else '出球指令完成；串口成功不代表球已经投出/命中')
        return True

    def home(self):
        if self.task == 'shoot' and not self.move(self.cfg['corridor']):
            return False
        if not self.move(self.cfg['entry']) or not self.move(self.cfg['start']):
            return False
        return self.turn(0.)

    def execute(self):
        if self.mode == 'cycle':
            # 整轮和分阶段共用同一套动作；整轮自动衔接三个阶段。
            from match_round_steps import execute_cycle
            return execute_cycle(self.r, self.cfg, self.round, self.task,
                                 self.release_kind, emit=self.emit)
        self.r.args.ball_label = TASK_LABELS[self.task]
        self.r.args.speed = self.cfg['speed_mps']
        self.say('假定车已放回 A 区、面向篮筐；当前位姿作为本轮起点')
        self.say('雷达控制行驶/回位；前摄找目标球与出球柱；后摄仅横移对球')
        self.r.stop()
        self.r.spin(self.cfg['start_delay_s'])
        if not self.turn(-math.pi/2):
            return False
        preloaded = self.round == 1 and self.cfg.get('round1_first_ball', 'preloaded') == 'preloaded'
        if preloaded:
            if not self.move(self.cfg['entry']) or not self.action_point(aim=self.mode != 'collect'):
                return False
            if self.mode == 'collect':
                self.say('预装球已到动作点，停车；分步模式可继续放球、取第二颗和回位')
                return True
            if self.task == 'shoot' and not self.move(self.cfg['corridor']):
                return False
            if not self.move(self.cfg['entry']):
                return False
        for cycle in range(1):
            found = False
            for area in search_order(self.round, self.task):
                result = self.search_area(area, cycle)
                if result == 'failed':
                    return False
                if result == 'found':
                    found = True
                    break
                if self.mode == 'drive':
                    break  # 空场路线只走本环节优先区，不额外绕兜底区。
            if self.mode != 'drive' and not found:
                self.say('规划搜索区域未找到目标球，实验结束')
                self.r.stop()
                return False
            if not self.action_point(aim=self.mode != 'collect'):
                return False
            if self.mode == 'collect':
                self.say('到达动作点，停车等调试；本模式不传球/投篮、不继续拿第二颗')
                return True
        if not self.home():
            return False
        self.say('已回本轮起点；承重轮是否全部在出发区需现场检查')
        return True
