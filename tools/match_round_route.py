"""A 区出发的三个回合实验；复用固定路线的 ROS/后摄/吸球实现。

drive: 空场走搜索轨迹、动作点和回位，不取球、不发送机构指令。
collect: 寻一颗目标球、转身对正吸球、退回通道、到动作点停车。
cycle: 放球联调或配置后正式出球两次并回位；第一回合先处理预装球。
所有场地坐标通过启动位姿建立相对变换，不改定位原点。
"""
import json
import math
import time
from contextlib import nullcontext
from pathlib import Path
from visual_target_alignment import align_pillar

CONFIG = Path(__file__).with_name('match_rounds.json')
TASK_LABELS = {'pass': 'ball_basketball', 'shoot': 'ball_volleyball'}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def side_heading(cfg, heading):
    """A帧的规划朝向在B场地取反；定位及相机测量不变。"""
    return -heading if cfg.get('start_point', 'A') == 'B' else heading


def start_wait(runner, seconds, emit=None):
    """起步前的静止等待，带【每秒倒计时】。

    原来只是 runner.spin(N) 闷等，日志里看不出还剩几秒，现场不好判断。
    这里把它拆成每秒一步，每一步都打印剩余时间；等待期间照常处理定位回调，
    但不发任何底盘速度。整轮入口和分阶段入口都用它。
    """
    left = max(0., float(seconds))
    if left <= 0:
        return
    if emit is not None:
        emit(f'起步准备：{left:.0f} 秒后开始动作（等待期间不发底盘速度）')
    while left > 1e-6:
        step = min(1., left)
        runner.spin(step)
        left -= step
        if left > 1e-6 and emit is not None:
            emit(f'  还有 {math.ceil(left)} 秒')


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


def mirror_to_b(cfg):
    """把【以 A 出发写的】路线镜像成 B 出发（关于场地宽度中线 y=0 对称）。

    球场本身关于 y=0 对称：篮筐、三分线圆心、中线置球区圆心都在轴上，
    半场置球区与三分线内置球区的 y 范围也对称，A 区与 B 区互为镜像。
    所以 B 出发不必另写一份路线，整份镜像即可 —— 12 条路线 = 6 份 × 镜像。

    变换（镜像完仍走原有范围校验，所以结果必须仍然合法）：
      所有 y 坐标                y → −y
      所有规划朝向                h → −h
      inner.dir_pass/dir_shoot   不变：已镜像的端点保持原访问顺序
      arc.angles_deg/angles_shoot v → −v
          （弧线点 = 圆心 + R·(cos v, sin v)，v 是场地系逆时针角，故取负；
            朝向 v+π 取负后与 −v+π 等价）
      middle.scan_dir            不动 —— 它是沿 x 的左右方向，y 翻转不影响
      half_court_x_m / 各种 x     不动
    """
    o = json.loads(json.dumps(cfg))          # 深拷贝，不改调用方的字典

    def flip_pair(seq):
        if isinstance(seq, list) and len(seq) >= 2:
            seq[1] = -seq[1]
        return seq

    for key in ('start', 'entry', 'corridor'):
        flip_pair(o.get(key))
    middle = o.get('middle') or {}
    if isinstance(middle.get('y'), (int, float)):
        middle['y'] = -middle['y']
    inner = o.get('inner') or {}
    for k in ('top_y', 'bottom_y'):
        if isinstance(inner.get(k), (int, float)):
            inner[k] = -inner[k]
    # inner_via 是现场加的中转点（见 search_area），y 也要跟着镜像，
    # 否则 B 出发会绕到场地另一侧去。
    if isinstance(o.get('inner_via'), dict):
        o['inner_via'] = {k: [[p[0], -p[1]] for p in (vs or [])]
                          for k, vs in o['inner_via'].items()}
    # dir_pass / dir_shoot 【不能】互换。
    # 它们的含义是"先扫 bottom_y 还是先扫 top_y"（见 search_plan 的 inner 分支），
    # 而 top_y / bottom_y 上面已经取过负了 —— 变量本身就代表镜像后的两端，
    # 所以 'bt' 仍然表示"从 bottom_y 扫到 top_y"。
    # 再换一次方向等于【双重翻转】，B 出发的深场扫描顺序会反掉。
    arc = o.get('arc') or {}
    flip_pair(arc.get('center'))
    if isinstance(arc.get('angles_deg'), list):
        arc['angles_deg'] = [-v for v in arc['angles_deg']]
    if isinstance(arc.get('angles_shoot'), dict):
        arc['angles_shoot'] = {k: [-v for v in vs]
                               for k, vs in arc['angles_shoot'].items()}
    for task in ('pass', 'shoot'):
        act = (o.get('release') or {}).get(task) or {}
        flip_pair(act.get('point'))
        flip_pair(act.get('aim'))
        if isinstance(act.get('search_heading_deg'), (int, float)):
            act['search_heading_deg'] = -act['search_heading_deg']
    o['start_point'] = 'B'
    return o


def load_config(path=CONFIG, start_point=None):
    with open(path, encoding='utf-8') as stream:
        cfg = json.load(stream)
    # 出发点：A 用文件里那份；B 先关于场地中线镜像，再应用 B 专用现场校准。
    chosen = str(start_point if start_point is not None
                 else cfg.get('start_point', 'A')).strip().upper()
    if chosen == 'B':
        cfg = mirror_to_b(cfg)
    elif chosen != 'A':
        raise ValueError(f"出发点只能是 A 或 B，收到 {chosen!r}")
    cfg['start_point'] = chosen
    from round_return_budget import validate_return_budget
    validate_return_budget(cfg)
    def number(value, lo, hi):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'路线参数不在范围 [{lo}, {hi}]: {value!r}')
    # B 场地现场校准：先镜像，再按场地坐标加偏移；A 完全不受影响。
    adjustments = cfg.get('b_y_adjustments_m', {})
    if not isinstance(adjustments, dict) or any(
            key not in ('middle', 'pass') for key in adjustments):
        raise ValueError('b_y_adjustments_m 只支持 middle 和 pass')
    for value in adjustments.values():
        number(value, -math.inf, math.inf)
    if chosen == 'B':
        cfg['middle']['y'] += adjustments.get('middle', 0.)
        cfg['release']['pass']['point'][1] += adjustments.get('pass', 0.)
    # Y 原点平移时，场地边界与动作区域校验使用同一偏移。
    y_offset = cfg.get('coordinate_y_offset_m', 0.)
    number(y_offset, -math.inf, math.inf)
    def point(value, start=False):
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError('路线坐标须为 [x,y]')
        number(value[0], 0, 14)
        # 出发区在场地【外侧】：A 从上面凸出、B 从下面凸出，所以 start 的
        # y 范围要【上下对称】都放宽 1 m（旧帧 −1.0 ~ 8.5）。
        # 其余动作点仍在场地内 0 ~ 7.5。原来下界写死 y_offset，B 区(−4.25)
        # 会被判成越界。
        number(value[1], (-1.0 if start else 0.0) + y_offset,
               (8.5 if start else 7.5) + y_offset)
    point(cfg['start'], start=True)
    for key in ('entry', 'corridor'):
        point(cfg[key])
    # inner_via：可选的中转点，按任务给一串 [x, y]（见 search_area 的 inner 分支）。
    # 不配就是 []，行为和以前完全一样。
    via = cfg.get('inner_via')
    if via is not None:
        if not isinstance(via, dict) or any(k not in ('pass', 'shoot') for k in via):
            raise ValueError('inner_via 只支持 pass / shoot')
        for k, pts in via.items():
            if not isinstance(pts, list):
                raise ValueError(f'inner_via.{k} 须为坐标列表')
            for p in pts:
                if not isinstance(p, (list, tuple)) or len(p) != 2:
                    raise ValueError(f'inner_via.{k} 的每一项须为 [x, y]')
                point(list(p))
    number(cfg['speed_mps'], math.nextafter(0., math.inf), math.inf)
    from transit_motion import parameters as transit_parameters
    transit_parameters(cfg.get('transit_control'))
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
        tasks = vision.get('tasks')
        if tasks is not None:
            if not isinstance(tasks, list) or any(
                    t not in ('pass', 'shoot') for t in tasks):
                raise ValueError('pillar_alignment.tasks 须为 pass/shoot 的列表，'
                                 '空列表表示出球不对柱')
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
    # top_y/bottom_y 是【以 A 帧命名】的两端，只要求不重合 —— 到底先走上端
    # 还是下端由 dir_pass / dir_shoot 决定（bt=从下端到上端）。
    # 原来写成 bottom_y >= top_y 报错，是把 A 帧的上下当成绝对方向了：
    # B 出发整份镜像后 top_y/bottom_y 会互换大小，那个检查会误判。
    if i['bottom_y'] == i['top_y']:
        raise ValueError('内侧搜索的上下两端不能重合')
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


def search_order(round_number, task, cfg=None, stage=None):
    """本回合去哪些区域找球、按什么顺序。

    【真源】tools/match_rounds.json 的 search_order[task][round] ——
    由场地图编辑器从你画的 route 里的 @区域 顺序导出，所以你在编辑器里
    把 @arc 和 @inner 调个位置，车就跟着调。
    cfg 没传或配置里没这一项时，才用下面的内置默认值。

    配置有两种写法：
      平铺   ["middle", "arc"]                 两个阶段都用这一份
      分阶段 [["middle"], ["arc"]]             阶段1 找中线、阶段2 找弧线
    分阶段写法由场地图编辑器从 route 里【按出球点切段】导出 —— 你画什么
    阶段找哪些区，车就按那个来，不会两个阶段都重搜一遍。
    stage 传 1 或 2 取对应阶段；不传则把各阶段合起来（兼容旧调用方）。
    """
    if round_number not in (1, 2, 3):
        raise ValueError('只支持第1、2、3回合')
    if cfg is not None:
        table = (cfg.get('search_order') or {}).get(task) or {}
        areas = table.get(str(round_number))
        if areas is None:
            areas = table.get(round_number)
        if areas:
            if isinstance(areas[0], (list, tuple)):     # 分阶段写法
                if stage is not None:
                    idx = int(stage) - 1
                    return ([str(a) for a in areas[idx]]
                            if 0 <= idx < len(areas) else [])
                out = []
                for group in areas:
                    for a in group:
                        if a not in out:
                            out.append(a)
                return [str(a) for a in out]
            return [str(a) for a in areas]
    # ↓ 内置默认（配置里没写时用）
    if round_number == 1:
        return ['middle']
    if round_number == 2:
        return ['middle', 'arc']
    return ['arc', 'inner']


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
        # 【方向】前摄要朝【球线 y=0】看，不是死朝 -y：
        #   车在 y>0（A 出发）⇒ 球线在 -y 侧 ⇒ 朝 -π/2
        #   车在 y<0（B 出发）⇒ 球线在 +y 侧 ⇒ 朝 +π/2
        # 写死 -π/2 的话，B 出发时前摄背对球线，整段中线扫描都看不到球。
        face = -math.pi/2 if m['y'] >= 0 else math.pi/2
        return [((x, m['y']), face) for x in xs]

    if area == 'inner':
        i = cfg[area]
        shoot = task == 'shoot'
        x = inner_x(cfg, task)
        direction = (i.get('dir_shoot') if shoot else i.get('dir_pass')) or 'tb'
        ys = [i['bottom_y'], i['top_y']] if direction == 'bt' else [i['top_y'], i['bottom_y']]
        if cycle:
            ys = list(reversed(ys))
        # 投篮从球线 X=9 的右侧扫描，前摄须朝 -X；传球从左侧朝 +X。
        face = math.pi if shoot else 0.
        return [((x, y), face) for y in ys]

    a = cfg['arc']
    radius = a['radius']
    angles = pick(a.get('angles_deg'))
    face_offset = math.pi  # 外侧扫描朝圆心看放球弧。
    if task == 'shoot' and a.get('radius_shoot') is not None:
        radius = a['radius_shoot']
        angles = pick(a.get('angles_shoot')) or a['angles_deg']
        face_offset = 0.  # 投篮在放球弧内侧，前摄应径向朝外。
    if cycle:
        angles = list(reversed(angles))
    return [((a['center'][0]+radius*math.cos(math.radians(v)),
              a['center'][1]+radius*math.sin(math.radians(v))),
             wrap(math.radians(v)+face_offset)) for v in angles]


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

    def move_world(self, p, remember=False, speed=None, stop_at_goal=True, heading=None):
        # 转场按制动包络和反馈调速；识球扫描仍用 scan_to 的独立低速控制。
        # p 已在定位坐标系；remember 只保存返程点，不作为是否允许运动的条件。
        if speed is None:
            key = 'carry_mps' if self.carrying else 'travel_mps'
            speed = self.speed_profile.get(key, self.cfg['speed_mps'])
        distance = math.hypot(p[0]-self.r.pose[0], p[1]-self.r.pose[1])
        saved_speed = self.r.args.speed
        self.r.args.speed = speed
        try:
            ok = self.r.goto(*p, timeout=max(20., distance/speed+15.),
                             tolerance=self.r.args.step_tol, transit=True,
                             stop_at_goal=stop_at_goal, heading=heading,
                             transit_options=self.cfg.get('transit_control'))
        finally:
            self.r.args.speed = saved_speed
        if ok and remember:
            self.trail.append(tuple(p))
        return ok

    def move(self, p, remember=False):
        # 配置中的场地坐标先按本轮起点映射到定位坐标，再发送运动指令。
        return self.move_world(self.frame.point(p), remember)

    def move_path(self, points, remember=False):
        """整串点一起规划速度及前视点，中间平滑衔接，终点制动确认。"""
        if not points:
            return True
        heading = self.r.pose[2]
        world = [self.frame.point(p) for p in points]
        key = 'carry_mps' if self.carrying else 'travel_mps'
        saved_speed = self.r.args.speed
        self.r.args.speed = self.speed_profile.get(key, self.cfg['speed_mps'])
        try:
            ok = self.r.goto(*world[-1], timeout=0., tolerance=self.r.args.step_tol,
                             transit=True, stop_at_goal=True, heading=heading,
                             transit_options=self.cfg.get('transit_control'), path_points=world,
                             path_progress=(lambda p: self.trail.append(tuple(p))) if remember else None)
            if not ok:
                self.r.stop()
            return ok
        except BaseException:
            self.r.stop()
            raise
        finally:
            self.r.args.speed = saved_speed

    def turn(self, heading):
        return self.r.rotate_to(self.frame.yaw(heading), timeout=45.)

    def scan(self, target, heading, detect=True, centered=False):
        # centered=True（scan_centered=true，当前配置）：只认画面中心带
        #     （0.5±route_center_tolerance）里的球 —— 球横移到画面中间才停、才转身。
        # centered=False：球出现在画面【任意位置】就停，随后由 face_detected()
        #     把车头转正对着球（停得更早，但可能停在球还在侧面的时候）。
        # 两种模式下都是【一检测到就立刻停车返回】，不会跑完整条寻球线。
        # found=目标触发停车，empty=到线端没找到，failed=运动异常/超时。
        target, heading = self.frame.point(target), self.frame.yaw(heading)
        distance = math.hypot(target[0]-self.r.pose[0], target[1]-self.r.pose[1])
        speed = self.speed_profile.get('scan_mps', self.cfg['speed_mps'])
        status, det = self.r.scan_to(*target, heading,
                                     timeout=max(15., distance/speed+10.),
                                     speed=speed, detect=detect, centered=centered)
        self.last_det = det          # 留给 face_detected() 用
        if status == 'found':
            # 记下实际停车点，不把尚未到达的右端点加入返程路线。
            self.search_origin = tuple(self.r.pose)
            self.trail.append(self.search_origin[:2])
            self.say('识别到目标球，停车 → 正对球 → 转180° → 后摄横移对正')
        elif status == 'empty':
            self.trail.append(tuple(target))
        return status

    def face_detected(self, timeout=20.):
        # 把车头转向刚检测到的球，让它落到画面中心（"正对着球"）。
        # 符号：Detection.bearing_rad 是相机约定【右正左负】；
        #       cmd_vel.angular.z 是 ROS 约定【逆时针为正】，两者相反
        #       ⇒ 目标朝向 = 当前朝向 − bearing。
        #       （与 geometry.face_bearing_command 的 err = −bearing 同一约定。）
        if self.cfg.get('face_ball_before_turn', True) is False:
            return True
        det = getattr(self, 'last_det', None)
        if det is None:
            return True
        try:
            bearing = float(getattr(det, 'bearing_rad', 0.0) or 0.0)
        except (TypeError, ValueError):
            return True
        if not math.isfinite(bearing):
            return True
        if abs(bearing) < math.radians(2.0):
            self.say('球已在正前方（偏差 <2°），不用转')
            return True
        if abs(bearing) > math.radians(120.0):
            # 球在侧面/后方：转过去会把车带离搜索线，宁可不停
            self.say('⚠️ 球的方位角 %+.0f° 超过 120°，放弃正对，继续搜索'
                     % math.degrees(bearing))
            return False
        target_yaw = wrap(self.r.pose[2] - bearing)
        self.say('正对球：方位角 %+.1f° ⇒ 车头转到 %+.1f°'
                 % (math.degrees(bearing), math.degrees(target_yaw)))
        return self.r.rotate_to(target_yaw, timeout=timeout)

    def area_x(self, area):
        # 各搜索区域在 x 上的代表位置（用来判断要不要跨半场置球线）
        if area == 'middle':
            m = self.cfg['middle']
            return (float(m['left_x']) + float(m['right_x'])) / 2.
        if area == 'arc':
            return float(self.cfg['arc']['center'][0])
        return float(self.cfg['inner']['x'])

    def crossing_via(self, target_x):
        # 从当前位置去 target_x，需要绕过半场置球线时返回途经点；
        # 不需要（同侧）返回空列表。规则与场地图编辑器里的画法一致：
        #   · 同侧        → 不绕
        #   · 左 → 右     → 先在【左侧】抬到通道高度，再沿通道越过去。
        #                   从进场点进场地时 y 已经在通道高度上（entry 与 corridor 的 y 相同），
        #                   这时不加横穿点，直接沿通道走，省一次到点停车。
        #   · 右 → 左     → 先回通道，再从上面越回来。
        cfg, corridor = self.cfg, self.cfg.get('corridor')
        if not corridor:
            return []
        if cfg.get('search_via_entry', False):
            # 回退开关：旧行为 = 每次都先回进场点
            return [cfg['entry'], corridor]
        half = float(cfg.get('half_court_x_m', 5.75))
        cur_x, cur_y = self.frame.nominal(self.r.pose[:2])
        if (cur_x - half) * (float(target_x) - half) >= 0:
            return []                                  # 同侧，直接去
        cross = [float(cfg.get('carry_cross_x_m', 4.8)), corridor[1]]
        # 【方向】"车已经在通道高度（或更外侧）"这件事是有方向的：
        #   通道在 y>0（A 出发）⇒ 更外侧是 y 更大，判 cur_y >= 通道y - 0.15
        #   通道在 y<0（B 出发，整份路线镜像过）⇒ 更外侧是 y 更小，
        #   必须判 cur_y <= 通道y + 0.15。
        # 写成单向的话，B 出发时这个条件几乎永远成立，就【不会加横穿点】，
        # 车会从球区中间直接斜穿半场置球线。
        out = 1.0 if corridor[1] >= 0 else -1.0
        at_corridor = out * cur_y >= abs(corridor[1]) - 0.15
        if cur_x < half:
            lift = [] if at_corridor else [cross]
            return lift + [corridor]
        return [corridor, cross]

    def carry_via(self):
        """取球后【回动作点之前】要经过的点，和场地图编辑器里的路线一致。

        编辑器路线：
          中线      @middle  → 传球点          （直接去，不绕）
          弧线/深场 @arc/@inner → corridor → [4.8, 通道y] → 传球点
                    ↑ 先回到上侧通道、再沿通道走到 x=4.8 才下去，
                      避免从右侧直接斜穿半场置球区（绿色竖线）。
        exit_after_pickup="unwind"（旧行为）时不返回途经点，
        由 unwind() 沿走过的折线原路退回。
        """
        if self.cfg.get('exit_after_pickup', 'direct') == 'unwind':
            return []
        # 取球点 → 动作点要不要绕半场置球线：传球点在左侧(3.55)、
        # 投篮点在右侧(10.45)，所以两者的绕行方向正好相反，交给 crossing_via 判断。
        action_x = float(self.cfg['release'][self.task]['point'][0])
        return self.crossing_via(action_x)

    def unwind(self):
        """取球后回到检测点，再沿已经走过的折线退出，不直接穿过置球线。"""
        self.r.stop()
        leaving_pickup = self.carrying and self.search_origin is not None
        heading = self.r.pose[2]
        while self.trail:
            p = self.trail[-1]
            speed = self.speed_profile.get('pickup_exit_mps') if leaving_pickup else None
            if not self.move_world(p, speed=speed, stop_at_goal=len(self.trail) == 1,
                                   heading=heading):
                return False
            leaving_pickup = False
            self.trail.pop()
        return True

    def search_area(self, area, cycle, search_only=False, via=None):
        # search_only=True：本函数找到球只停车返回，转身/对正/靠近由
        # match_round_steps.execute_step 的后续动作负责；不是漏掉铲球。
        cfg = self.cfg
        self.say(f'搜索 {area}，目标类别 {TASK_LABELS[self.task]}')
        self.trail = []
        # 搜球前的中转。默认【不先回 E】。
        # ⚠️ 原来写的是 transit = [cfg['entry']]，于是：
        #    第1轮出完第一颗球后，车会先退回进场点 (1.5, 中线y) 附近再出发搜索，
        #    看上去像"又回了一下出发位置"；
        #    第2/3轮从动作点去弧线/深场时，也要先绕回左侧再走通道，多走十几米。
        # 现在：
        #    中线      完全不中转（中线就在下方，直接去本回合的起扫点）
        #    弧线/深场  先在【还处于置球线左侧】时抬到通道高度，再沿通道越过置球线
        #              —— 就是编辑器路线里的 [4.8, 6.8] → corridor
        #                 （旧帧 6.8 = 新帧 3.05，正好在半场置球线顶边之上）
        # 想回到旧行为：search_via_entry = true
        # 搜球前的中转：要跨过半场置球线才绕，同侧直接去（crossing_via 判断）。
        transit = list(via or []) + self.crossing_via(self.area_x(area))
        if area == 'inner' and transit:
            # 现场加的中转点（配置键 inner_via，只在跨线进深场时插）。
            # 由来：投篮的深场线在三分线【内侧】(x = inner.x_shoot)，
            #   直接沿通道高度一路切进去，会在三分线内置球线的球带里穿过，
            #   容易撞球。所以先降到 y=0.74 这个空档，再往里进。
            # 每个点按顺序插在"通道高度那一点"之前。
            for extra in (cfg.get('inner_via') or {}).get(self.task) or []:
                transit.append(list(extra))
            transit.append([inner_x(cfg, self.task), cfg['corridor'][1]])
        points = search_plan(cfg, area, cycle, self.round, self.task)
        # 起扫点与前面的通道属于同一次转场；到起扫点才停车并对准扫描方向。
        if not self.move_path(transit + [points[0][0]], remember=True):
            return 'failed'
        if area == 'middle':
            # 直接到【本回合方向的第一扫描点】，再从那里扫向另一端。
            # ⚠️ 原来写的是 move([start.x, middle.y])，x 被硬编码成出发点的 1.5，
            #    于是第1轮（scan_dir=rl）会：先进场 → 到 (1.5, 中线y) → 向右到 4.4
            #    → 再向左扫。多了一趟"向右"，看上去就是从【左往右】找。
            #    改成 points[0] 后：
            #      第1轮 rl → 直接去 x=4.4（右端），再向左扫到 0.6  ✔
            #      第2轮 lr → 直接去 x=0.6（左端），再向右扫到 4.4  ✔
            #    到底点用 goto（不认球）；随后循环里的 scan 会先在 points[0] 查一次球，
            #    没有就继续扫到 points[1] —— 整条线仍然全程认球。
            if not self.turn(points[0][1]):
                return 'failed'
            self.say(f"中线到本回合起扫点 X={points[0][0][0]:.2f}，"
                     f"再扫至 X={points[1][0][0]:.2f}，遇球即停")
        for p, yaw in points:
            if not self.turn(yaw):
                return 'failed'
            # 球要进画面中心带（0.5±route_center_tolerance）才停 = scan_centered true。
            # 一检测到就立刻停车返回，不会跑完整条寻球线。
            # 设 scan_centered=false 则球出现在画面任意位置就停（更早停）。
            status = self.scan(p, yaw, detect=self.mode != 'drive',
                               centered=bool(cfg.get('scan_centered', False)))
            if status != 'empty':
                if status == 'found':
                    self.area = area
                    if search_only:
                        return status
                    # ① 先把车头转成正对球（球框进画面中心）
                    if not self.face_detected():
                        continue          # 球在侧面/后方 → 当作没找到，接着扫
                    # ② 再转身 180°，让球落到【后摄】视野里
                    # 单次 collect 的入口则在这里继续转180°、后摄对正和后退。
                    before = self.r.pose[2]
                    if not self.r.rotate_to(wrap(before+math.pi), timeout=45.):
                        return 'failed'
                    saved_distance, saved_timeout = self.r.args.route_scoop_distance, self.r.args.route_scoop_timeout
                    self.r.args.route_scoop_distance = cfg[area]['scoop_m']
                    if getattr(self.r.args, 'route_intake_settle', None) is None:
                        self.r.args.route_intake_settle = float(cfg.get('intake_settle_s', 1.5))
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
                # ③ 取球后：默认【直接去下一个点】，不原路退回。
                #    配 exit_after_pickup="unwind" 可回到旧行为（沿走过折线退出）。
                if cfg.get('exit_after_pickup', 'direct') == 'unwind':
                    if not self.unwind():
                        return 'failed'
                return status
        if cfg.get('exit_after_pickup', 'direct') == 'unwind':
            return 'empty' if self.unwind() else 'failed'
        return 'empty'

    def needs_pillar_alignment(self):
        # 出球【只看角度，不看视觉】：车头转到配置角度（fixed）或转向瞄准点
        # （target）之后就该发，不需要前摄先看到定位柱。
        # 真要做前摄对柱，把任务名填进 pillar_alignment.tasks，例如 ["pass"]。
        #   tasks 缺省（None）⇒ 兼容旧配置：非 fixed 模式就对柱
        #   tasks 为空列表    ⇒ 任何任务都不对柱（当前配置）
        vision = self.cfg.get('pillar_alignment', {})
        if not vision.get('enabled', False):
            return False
        tasks = vision.get('tasks')
        if tasks is None:
            return self.cfg['release'][self.task].get('aim_mode') != 'fixed'
        return self.task in tasks

    def action_point(self, aim=True, via=None):
        cfg = self.cfg
        self.heading_ready = False
        points = list(via or [])
        # 投篮点在绿线右边，过半场仍走上侧通道；传球点位于左侧传球圆内。
        if self.task == 'shoot':
            # 投篮动作点在右侧。要不要先绕半场置球线由 crossing_via 判断：
            #   从进场点进场地(y 已在通道高度) → entry → corridor → shootPt
            #   从中线取球(y 低)               → [4.8,通道y] → corridor → shootPt
            #   从弧线/深场(同在右侧)           → 直接去 shootPt
            if not points:
                points.extend(self.crossing_via(float(cfg['release']['shoot']['point'][0])))
        else:
            # 传球：默认直接去传球点（与编辑器路线一致）。
            # pass_action_via_middle=true 时先绕到 (start.x, 中线y) —— 旧行为。
            if self.cfg.get('pass_action_via_middle', False):
                points.append([cfg['start'][0], cfg['middle']['y']])
        action = cfg['release'][self.task]
        points.append(action['point'])
        if not self.move_path(points):
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
        budget = getattr(self.r, 'round_return_budget', None)
        action = self.cfg['release'][self.task]
        commands = self.cfg['drop_commands'] if self.release_kind == 'drop' else action['commands']
        waits = action.get('command_waits_s', []) if self.release_kind == 'match' else []
        seconds = sum(waits[i] if i < len(waits) else action['wait_s'] for i in range(len(commands)))
        if self.task == 'shoot' and self.release_kind == 'match':
            from serial_actions import SHOOT_RESERVE_SECONDS
            seconds = SHOOT_RESERVE_SECONDS
        if not getattr(self.r.args, 'route_intake', True):
            seconds = 0.
        section = budget.mechanism(seconds) if budget is not None else nullcontext()
        with section:
            ok = self._release()
        if ok and budget is not None:
            budget.check()
        return ok

    def _release(self):
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
        if self.task == 'shoot' and self.release_kind == 'match':
            from serial_actions import runner_actions
            self.r.round_release_confirmed = False
            report = runner_actions(self.r).shoot(commands)
            self.r.shoot_report = report
            self.r.round_release_confirmed = report.confirmed
            if not report.confirmed:
                self.say('投篮串口有异常/未确认步骤，已继续尝试全部命令：'+report.message)
            self.r.round_release_completed = True  # 序列尝试完毕，不表示物理动作成功。
            self.carrying = False
            self.say('投篮命令序列结束，继续后续任务；未检测球是否投出或机构是否到底')
            return True
        waits = action.get('command_waits_s', []) if self.release_kind == 'match' else []
        for index, command in enumerate(commands):
            ok, message = self.r.intake_text(command)
            self.say(f'出球指令 {command}: {message}')
            if not ok:
                self.last_error = f'机构指令 {command} 失败：{message}'
                return False
            wait = waits[index] if index < len(waits) else action['wait_s']
            if wait:
                self.r.spin(wait)
        self.say('放球联调完成，不计正式传球/投篮' if self.release_kind == 'drop'
                 else '出球指令完成；串口成功不代表球已经投出/命中')
        self.r.round_release_completed = True
        self.carrying = False
        return True

    def home(self):
        cfg = self.cfg
        points = [cfg['corridor']] if self.task == 'shoot' else []
        # 回正放在【进场点】：先把车头转回初始朝向，再直着进出发区。
        # 旧写法是先到出发点再原地转 —— 原地旋转会让已经到位的终点位置带偏，
        # 规则要求停稳在出发区内，所以默认改成在进场点回正。
        # 想回旧行为：home_align_at = "start"
        if cfg.get('home_align_at', 'entry') == 'start':
            if not self.move_path(points + [cfg['entry'], cfg['start']]):
                return False
            return self.turn(0.)
        if not self.move_path(points + [cfg['entry']]):
            return False
        self.say('已到出场点E，先恢复本轮初始朝向，再进入初始点S')
        if not self.turn(0.):
            return False
        return self.move(cfg['start'])

    def execute(self):
        if self.mode == 'cycle':
            # 整轮和分阶段共用同一套动作；整轮自动衔接三个阶段。
            from match_round_steps import execute_cycle
            return execute_cycle(self.r, self.cfg, self.round, self.task,
                                 self.release_kind, emit=self.emit)
        self.r.args.ball_label = TASK_LABELS[self.task]
        self.r.args.speed = self.cfg['speed_mps']
        if getattr(self.r.args, 'route_intake_settle', None) is None:
            self.r.args.route_intake_settle = float(self.cfg.get('intake_settle_s', 1.5))
        self.say(f"假定车已放回 {self.cfg.get('start_point', 'A')} 区、面向篮筐；当前位姿作为本轮起点")
        self.say('雷达控制行驶/回位；前摄找目标球与出球柱；后摄仅横移对球')
        self.r.stop()
        start_wait(self.r, self.cfg['start_delay_s'], emit=self.say)
        if not self.turn(side_heading(self.cfg, -math.pi/2)):
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
            for area in search_order(self.round, self.task, self.cfg):
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
