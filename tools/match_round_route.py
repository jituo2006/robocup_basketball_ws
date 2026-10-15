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
    # Y 原点平移时，场地边界与动作区域校验使用同一偏移。
    y_offset = cfg.get('coordinate_y_offset_m', 0.)
    number(y_offset, -math.inf, math.inf)
    def point(value, start=False):
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError('路线坐标须为 [x,y]')
        number(value[0], 0, 14)
        number(value[1], y_offset, (8.5 if start else 7.5)+y_offset)
    point(cfg['start'], start=True)
    for key in ('entry', 'corridor'):
        point(cfg[key])
    number(cfg['speed_mps'], math.nextafter(0., math.inf), math.inf)
    profiles = cfg.get('speed_profiles', {})
    if not isinstance(profiles, dict) or any(key not in ('1', '2', '3') for key in profiles):
        raise ValueError('speed_profiles 须为按回合1/2/3配置的字典')
    for profile in profiles.values():
        if not isinstance(profile, dict):
            raise ValueError('回合速度配置须为字典')
        for key, value in profile.items():
            if key not in ('travel_mps', 'carry_mps', 'scan_mps', 'pickup_exit_mps',
                           'turn_rad_s', 'approach_radius_m', 'approach_mps'):
                raise ValueError(f'未知回合速度参数: {key}')
            number(value, 0. if key in ('approach_radius_m', 'approach_mps')
                   else math.nextafter(0., math.inf), math.inf)
    number(cfg['start_delay_s'], 5, 15)
    if cfg.get('round1_first_ball', 'preloaded') not in ('preloaded', 'search'):
        raise ValueError('首球方式须为 preloaded/search')
    vision = cfg.get('pillar_alignment', {})
    if vision:
        if not isinstance(vision.get('enabled'), bool):
            raise ValueError('pillar_alignment.enabled 须为布尔值')
        for key, lo, hi in (('confidence', .25, 1), ('tolerance_deg', .2, 5),
                            ('stable_frames', 1, math.inf), ('timeout_s', math.nextafter(0., math.inf), math.inf),
                            ('max_turn_deg', 1, 30), ('max_ang_rad_s', .02, .25),
                            ('kp', math.nextafter(0., math.inf), math.inf), ('camera_yaw_offset_rad', -math.inf, math.inf),
                            ('launcher_yaw_offset_deg', -math.inf, math.inf)):
            number(vision[key], lo, hi)
        if not isinstance(vision['stable_frames'], int):
            raise ValueError('定位柱稳定帧数须为整数')
    m, a, i = cfg['middle'], cfg['arc'], cfg['inner']
    number(m['y'], y_offset, 7.5+y_offset)
    number(m['left_x'], 0, 14)
    number(m['right_x'], 0, 14)
    if m['left_x'] >= m['right_x']:
        raise ValueError('中线搜索左端须小于右端')
    point(a['center'])
    number(a['radius'], math.nextafter(0., math.inf), math.inf)
    if not isinstance(a['angles_deg'], list) or len(a['angles_deg']) < 2:
        raise ValueError('弧线至少需要两个角度')
    for angle in a['angles_deg']:
        number(angle, -math.inf, math.inf)
    for p, _ in search_plan(cfg, 'arc'):
        point(list(p))
    number(i['x'], 0, 14)
    number(i['top_y'], y_offset, 7.5+y_offset)
    number(i['bottom_y'], y_offset, 7.5+y_offset)
    if i['bottom_y'] >= i['top_y']:
        raise ValueError('内侧搜索下端须小于上端')
    for area in (m, a, i):
        number(area['scoop_m'], math.nextafter(.02, math.inf), math.inf)
    if not isinstance(cfg['drop_commands'], list) or not cfg['drop_commands'] or any(
            not isinstance(c, str) or not c.strip() or '\n' in c or '\r' in c for c in cfg['drop_commands']):
        raise ValueError('放球指令须为非空单行字符串列表')
    for task, action in cfg['release'].items():
        if task not in TASK_LABELS:
            raise ValueError('动作类型须为 pass/shoot')
        if action.get('aim_mode', 'target') not in ('target', 'fixed'):
            raise ValueError('出球朝向模式须为 target/fixed')
        if action.get('aim_mode') == 'fixed':
            number(action['search_heading_deg'], -math.inf, math.inf)
        point(action['point'])
        if action['aim'] is not None:
            point(action['aim'])
            if action['aim'] == action['point']:
                raise ValueError('动作点不能与瞄准目标重合')
        number(action['wait_s'], 0, math.inf)
        number(action.get('search_heading_deg', 0), -math.inf, math.inf)
        if not isinstance(action['commands'], list) or any(
                not isinstance(c, str) or not c.strip() or '\n' in c or '\r' in c
                for c in action['commands']):
            raise ValueError('出球指令须为非空单行字符串列表')
    if set(cfg['release']) != set(TASK_LABELS):
        raise ValueError('需要同时配置传球、投篮动作点')
    # 点的合法性只检查规划中心，不代替整机包络的实测。
    p = cfg['release']['pass']['point']
    q = cfg['release']['shoot']['point']
    if math.hypot(p[0]-2.5, p[1]-(3.75+y_offset)) >= 2.5:
        raise ValueError('传球动作点须在传球圆内')
    if math.hypot(q[0]-12.425, q[1]-(3.75+y_offset)) <= 2:
        raise ValueError('投篮动作点须在投篮边界外')
    return cfg


def release_ready(action):
    return bool(action['commands']) and (action.get('aim_mode') == 'fixed' or action['aim'] is not None)


def validate_mode(cfg, task, mode, release_kind='match', mechanism_enabled=True):
    if task not in TASK_LABELS or mode not in ('drive', 'collect', 'cycle'):
        raise ValueError('任务/实验模式无效')
    if release_kind not in ('drop', 'match'):
        raise ValueError('出球模式须为 drop/match')
    if not isinstance(mechanism_enabled, bool):
        raise ValueError('机构开关须为布尔值')
    if mode == 'cycle' and release_kind == 'match' and mechanism_enabled:
        action = cfg['release'][task]
        if not release_ready(action):
            raise ValueError('双次流程须配置真实出球 commands；朝向须配置 aim 或使用 fixed 模式的 search_heading_deg')


def search_order(round_number, task):
    if round_number == 1:
        return ['middle']
    if round_number == 2:
        return ['middle', 'arc']
    if round_number == 3:
        return ['arc', 'inner'] if task == 'pass' else ['inner', 'arc']
    raise ValueError('只支持第1、2、3回合')


def inner_x(cfg, task):
    """本任务该用哪条深场扫描线：投篮走 inner.x_shoot（球线 X=9 的右侧=三分线内侧），
    其余走 inner.x。search_plan 与 search_area 共用，避免两处不一致。"""
    i = cfg['inner']
    if task == 'shoot' and i.get('x_shoot') is not None:
        return i['x_shoot']
    return i['x']


def search_plan(cfg, area, cycle=0, round_number=None, task=None):
    """(点, 前摄朝向)。其它区域第二次动作换向。

    行径路线：
      中线 —— 左右端点取 middle.left_x/right_x；方向按【轮次】由 middle.scan_dir 给
              （第1轮车在右端附近 ⇒ 先到右端再往左；第2轮从进场点过来 ⇒ 先到左端）
      弧线 —— 传球用外侧 radius/angles_deg；投篮用内侧 radius_shoot，
              角度按【轮次】由 angles_shoot 给（弧线前后接的东西不同，最优方向不同）
      深场 —— 扫描线取 inner.x；投篮另取 inner.x_shoot（球线 X=9 的右侧=三分线内侧）
              上下方向按【任务】由 dir_pass / dir_shoot 给（pt=从下到上，否则从上到下）
    """
    key = None if round_number is None else str(round_number)

    def pick(value):
        """value 可以是标量，也可以是 {轮次: 值} 的字典。"""
        if isinstance(value, dict):
            return value.get(key) if key is not None else None
        return value

    if area == 'middle':
        m = cfg[area]
        direction = pick(m.get('scan_dir')) or 'lr'
        xs = (m['right_x'], m['left_x']) if direction == 'rl' else (m['left_x'], m['right_x'])
        return [((x, m['y']), -math.pi/2) for x in xs]

    if area == 'inner':
        i = cfg[area]
        shoot = task == 'shoot'
        x = inner_x(cfg, task)
        direction = (i.get('dir_shoot') if shoot else i.get('dir_pass')) or 'tb'
        ys = [i['bottom_y'], i['top_y']] if direction == 'bt' else [i['top_y'], i['bottom_y']]
        if cycle:
            ys = list(reversed(ys))
        return [((x, y), 0.) for y in ys]

    a = cfg['arc']
    radius = a['radius']
    angles = pick(a.get('angles_deg'))
    if task == 'shoot' and a.get('radius_shoot') is not None:
        radius = a['radius_shoot']
        angles = pick(a.get('angles_shoot')) or a['angles_deg']
    if cycle:
        angles = list(reversed(angles))
    return [((a['center'][0]+radius*math.cos(math.radians(v)),
              a['center'][1]+radius*math.sin(math.radians(v))),
             wrap(math.radians(v)+math.pi)) for v in angles]


class RoundRoute:
    def __init__(self, runner, cfg, round_number, task, mode, emit=print, release_kind='match'):
        validate_mode(cfg, task, mode, release_kind,
                      mechanism_enabled=getattr(runner.args, 'route_intake', True))
        self.r, self.cfg = runner, cfg
        self.round, self.task, self.mode = round_number, task, mode
        self.speed_profile = cfg.get('speed_profiles', {}).get(str(round_number), {})
        # 这是流程中的载球阶段标记，不是入仓传感器反馈。
        self.carrying = round_number == 1 and cfg.get('round1_first_ball', 'preloaded') == 'preloaded'
        self.release_kind = release_kind
        self.emit = emit
        self.frame = StartFrame(runner.pose, cfg['start'])
        runner.round_start_frame = self.frame
        self.trail = []
        self.search_origin = None
        self.target_aligned = False
        self.heading_ready = False
        self.last_error = ''
        if round_number == 1 and release_kind == 'match' and cfg.get('round1_first_ball', 'preloaded') != 'preloaded':
            raise ValueError('规则第1轮首球须预装；场上取首球仅用于放球联调')

    def say(self, message):
        self.emit(f'第{self.round}回合/{self.task}/{self.mode}: {message}')

    def move_world(self, p, remember=False, speed=None):
        # 走线和识球扫描分别选速；最终仍调用原有到点控制。
        # p 已在定位坐标系；remember 只保存返程点，不作为是否允许运动的条件。
        if speed is None:
            key = 'carry_mps' if self.carrying else 'travel_mps'
            speed = self.speed_profile.get(key, self.cfg['speed_mps'])
        distance = math.hypot(p[0]-self.r.pose[0], p[1]-self.r.pose[1])
        saved_speed = self.r.args.speed
        self.r.args.speed = speed
        try:
            ok = self.r.goto(*p, timeout=max(20., distance/speed+15.),
                             tolerance=self.r.args.step_tol)
        finally:
            self.r.args.speed = saved_speed
        if ok and remember:
            self.trail.append(tuple(p))
        return ok

    def move(self, p, remember=False):
        # 配置中的场地坐标先按本轮起点映射到定位坐标，再发送运动指令。
        return self.move_world(self.frame.point(p), remember)

    def turn(self, heading):
        return self.r.rotate_to(self.frame.yaw(heading), timeout=45.)

    def scan(self, target, heading, detect=True, centered=True):
        """复用固定路线的横移控制；中线遇球即停，其余区域保留中心触发。"""
        # found=目标触发停车，empty=到线端没找到，failed=运动异常/超时。
        target, heading = self.frame.point(target), self.frame.yaw(heading)
        distance = math.hypot(target[0]-self.r.pose[0], target[1]-self.r.pose[1])
        speed = self.speed_profile.get('scan_mps', self.cfg['speed_mps'])
        status, _det = self.r.scan_to(*target, heading,
                                     timeout=max(15., distance/speed+10.),
                                     speed=speed, detect=detect, centered=centered)
        if status == 'found':
            # 记下实际停车点，不把尚未到达的右端点加入返程路线。
            self.search_origin = tuple(self.r.pose)
            self.trail.append(self.search_origin[:2])
            self.say('识别到目标球，停车 → 转180° → 后摄横移对正')
        elif status == 'empty':
            self.trail.append(tuple(target))
        return status

    def unwind(self):
        """取球后回到检测点，再沿已经走过的折线退出，不直接穿过置球线。"""
        self.r.stop()
        leaving_pickup = self.carrying and self.search_origin is not None
        while self.trail:
            p = self.trail[-1]
            speed = self.speed_profile.get('pickup_exit_mps') if leaving_pickup else None
            if not self.move_world(p, speed=speed):
                return False
            leaving_pickup = False
            self.trail.pop()
        return True

    def search_area(self, area, cycle, search_only=False):
        # search_only=True：本函数找到球只停车返回，转身/对正/靠近由
        # match_round_steps.execute_step 的后续动作负责；不是漏掉铲球。
        cfg = self.cfg
        self.say(f'搜索 {area}，目标类别 {TASK_LABELS[self.task]}')
        self.trail = []
        # 深场搜索统一经 E、T；不在绿色半场线找目标球。
        transit = [cfg['entry']]
        if area != 'middle':
            transit.append(cfg['corridor'])
            if area == 'inner':
                transit.append([inner_x(cfg, self.task), cfg['corridor'][1]])
        for p in transit:
            if not self.move(p, remember=True):
                return 'failed'
        points = search_plan(cfg, area, cycle, self.round, self.task)
        if area == 'middle':
            # 先朝黄线到 C，再按本回合方向横移；第一遍就启用识球。
            if not self.turn(points[0][1]):
                return 'failed'
            if not self.move([cfg['start'][0], cfg['middle']['y']], remember=True):
                return 'failed'
            self.say(f"中线先扫至 X={points[0][0][0]:.2f}，再扫至 X={points[1][0][0]:.2f}，遇球即停")
        elif not self.move(points[0][0], remember=True):
            return 'failed'
        for p, yaw in points:
            if not self.turn(yaw):
                return 'failed'
            status = self.scan(p, yaw, detect=self.mode != 'drive', centered=area != 'middle')
            if status != 'empty':
                if status == 'found':
                    self.area = area
                    if search_only:
                        return status
                    # 单次 collect 的入口则在这里继续转180°、后摄对正和后退。
                    before = self.r.pose[2]
                    if not self.r.rotate_to(wrap(before+math.pi), timeout=45.):
                        return 'failed'
                    saved_distance, saved_timeout = self.r.args.route_scoop_distance, self.r.args.route_scoop_timeout
                    self.r.args.route_scoop_distance = cfg[area]['scoop_m']
                    # 距离取本区域 scoop_m：中线默认0.70m，arc/inner另有配置；
                    # 修改通用 route_scoop_distance 不会覆盖此处的区域配置。
                    self.r.args.route_scoop_timeout = max(15., cfg[area]['scoop_m']/self.r.args.route_scoop_speed+5.)
                    try:
                        if not self.r.rear_visual_scoop():
                            return 'failed'
                        self.carrying = True
                    finally:
                        self.r.args.route_scoop_distance, self.r.args.route_scoop_timeout = saved_distance, saved_timeout
                    self.say('取球动作完成，实际入仓仍需实物/传感器确认')
                if not self.unwind():
                    return 'failed'
                return status
        return 'empty' if self.unwind() else 'failed'

    def needs_pillar_alignment(self):
        return (self.cfg['release'][self.task].get('aim_mode') != 'fixed'
                and self.cfg.get('pillar_alignment', {}).get('enabled', False))

    def action_point(self, aim=True):
        cfg = self.cfg
        self.heading_ready = False
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
        if action.get('aim_mode') != 'fixed' and action['aim'] is not None:
            target = self.frame.point(action['aim'])
            heading = math.atan2(target[1]-self.r.pose[1], target[0]-self.r.pose[0])
        else:
            heading = self.frame.yaw(math.radians(action.get('search_heading_deg', 0)))
        if not self.r.rotate_to(heading, timeout=45.):
            return False
        self.r.stop()
        self.heading_ready = True
        if action.get('aim_mode') == 'fixed':
            self.say(f"按本轮起始朝向旋转至配置角度 {action['search_heading_deg']:.1f}°，不做前摄对柱微调")
        self.target_aligned = False
        vision = cfg.get('pillar_alignment', {})
        if (self.mode != 'drive' and getattr(self.r.args, 'route_intake', True)
                and self.needs_pillar_alignment()):
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
            self.carrying = False
            return True
        if self.release_kind == 'match' and action.get('aim_mode') == 'fixed' and not self.heading_ready:
            self.say('尚未转到配置的传球角度，本次不发射')
            return False
        if (self.release_kind == 'match' and self.needs_pillar_alignment()
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
        self.carrying = False
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
