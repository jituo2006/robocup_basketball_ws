"""Two basketball pickups with one persistent start-to-field transform."""
import json
import math
from pathlib import Path

from match_round_route import RoundRoute, StartFrame, wrap
from field_coordinates import first_leg, PUBLIC_FRAME, LEGACY_FRAME, public_bounds

CONFIG = Path(__file__).with_suffix('.json')


def load_config(path=CONFIG):
    cfg = json.loads(Path(path).read_text(encoding='utf-8'))
    if cfg.get('coordinate_frame',LEGACY_FRAME) not in (PUBLIC_FRAME,LEGACY_FRAME):raise ValueError('invalid coordinate_frame')
    xmin,xmax,ymin,ymax=public_bounds() if cfg.get('coordinate_frame')==PUBLIC_FRAME else (0.,14.,0.,7.5)
    for key in ('return_point', 'second_scan_end', *(['pass_point'] if 'pass_point' in cfg else [])):
        p = cfg[key]
        if (not isinstance(p, list) or len(p) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in p)
                or not xmin <= p[0] <= xmax or not ymin <= p[1] <= ymax):
            raise ValueError(f'{key} 必须是场地内的 [x,y]')
    if cfg['return_point'] == cfg['second_scan_end']:
        raise ValueError('第二次搜索起点和终点不能重合')
    for key, lo, hi in (('second_scan_heading_deg', -math.inf, math.inf), ('second_scoop_m', math.nextafter(.02, math.inf), math.inf)):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f'{key} 不在范围 [{lo},{hi}]')
    if cfg.get('first_release', 'none') not in ('none', 'drop', 'match'):
        raise ValueError('first_release 须为 none/drop/match')
    cfg.setdefault('first_release', 'none')
    if cfg.get('target_reference', 'base') not in ('base', 'lidar'):
        raise ValueError('target_reference 须为 base/lidar')
    offset = cfg.get('lidar_in_base_m', [.30, 0.])
    if (not isinstance(offset, list) or len(offset) != 2 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) or abs(v)>1 for v in offset)):
        raise ValueError('lidar_in_base_m 须为有效车体安装偏移')
    return cfg


def validate_release(base, cfg, mechanism):
    if cfg['first_release'] not in ('none', 'drop', 'match'):
        raise ValueError('first_release 须为 none/drop/match')
    if mechanism and cfg['first_release'] == 'match':
        action = base['release']['pass']
        if not action['commands'] or action['aim'] is None:
            raise ValueError('正式传球需要已验证的 release.pass.commands 和 aim')


class FirstPassPlan(RoundRoute):
    """Field points refer to the radar; navigation consumes base centre points."""
    def __init__(self, runner, base, cfg, emit):
        super().__init__(runner, base, 1, 'pass', 'collect', emit=emit, release_kind='drop')
        self.assist_cfg=cfg
        runner.round_coordinate_frame=cfg.get('coordinate_frame',LEGACY_FRAME)
        self.leg=first_leg(base,cfg)
        self.reference=cfg.get('target_reference','base')
        self.offset=cfg.get('lidar_in_base_m',[.30,0.])
        if self.reference=='lidar':
            # Legacy relative mode anchors the RADAR at the nominal start.
            x,y,yaw=runner.pose
            dx,dy=self.offset
            anchor=(x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                    y+math.sin(yaw)*dx+math.cos(yaw)*dy,yaw)
            self.frame=StartFrame((*anchor[:2],wrap(anchor[2]-(math.pi/2 if cfg.get('coordinate_frame')==PUBLIC_FRAME else 0.))),self.leg['start'])
            runner.round_start_frame=self.frame
        elif cfg.get('coordinate_frame')==PUBLIC_FRAME:
            x,y,yaw=runner.pose
            self.frame=StartFrame((x,y,wrap(yaw-math.pi/2)),self.leg['start'])
            runner.round_start_frame=self.frame

    def target(self, point, yaw):
        x,y=self.frame.point(point)
        if self.reference=='lidar':
            dx,dy=self.offset
            x-=math.cos(yaw)*dx-math.sin(yaw)*dy
            y-=math.sin(yaw)*dx+math.cos(yaw)*dy
        return x,y

    def move(self, point, remember=False):
        return self.move_world(self.target(point,self.r.pose[2]),remember)

    def scan(self, target, heading, detect=True, centered=True):
        yaw=self.frame.yaw(heading)
        # scan_to holds this yaw; compensate with the same arrival heading.
        point=self.target(target,yaw)
        distance=math.hypot(point[0]-self.r.pose[0],point[1]-self.r.pose[1])
        status,_=self.r.scan_to(*point,yaw,timeout=max(15.,distance/self.r.args.speed+10.),
                              speed=self.r.args.speed,detect=detect,centered=centered)
        return status


def execute(runner, base, cfg, emit=print):
    """Stop at the second passing point; never run the old home/round2 logic."""
    mechanism = getattr(runner.args, 'route_intake', True)
    validate_release(base, cfg, mechanism)
    plan = FirstPassPlan(runner, base, cfg, emit)
    saved = {key: getattr(runner.args, key) for key in
             ('speed', 'route_scoop_distance', 'route_scoop_timeout')}
    runner.args.ball_label = 'ball_basketball'
    runner.args.speed = base['speed_mps']

    def pickup(distance):
        # Stop at a detected ball, turn around, align on fresh rear frames,
        # then use source-stamped displacement for the shovel approach.
        runner.stop()
        if not runner.rotate_to(wrap(runner.pose[2]+math.pi), timeout=45.):
            return False
        runner.args.route_scoop_distance = distance
        runner.args.route_scoop_timeout = max(15., distance/runner.args.route_scoop_speed+5.)
        return runner.rear_visual_scoop()

    try:
        runner.stop()
        emit('出发区 → 中线观察走廊 → 横移找第1颗篮球')
        runner.spin(base['start_delay_s'])
        if not plan.turn(plan.leg['heading_rad']):
            return False
        if not plan.move(plan.leg['middle_point']):
            return False
        status = 'empty'
        for point in plan.leg['scan_points']:
            status = plan.scan(point, plan.leg['heading_rad'],
                               detect=True, centered=False)
            if status != 'empty':
                break
        if status != 'found':
            emit('第1颗篮球未找到或搜索失败，停车结束')
            return False
        emit('第1次铲球 → 传球点')
        if not pickup(base['middle']['scoop_m']):
            return False
        if not plan.move(plan.leg['pass_point']):
            return False
        if mechanism and cfg['first_release'] != 'none':
            plan.release_kind = cfg['first_release']
            if cfg['first_release'] == 'match':
                from field_coordinates import to_public_point
                aim=base['release']['pass']['aim']
                if cfg.get('coordinate_frame')==PUBLIC_FRAME:aim=to_public_point(aim)
                target=plan.frame.point(aim)
                if not runner.rotate_to(math.atan2(target[1]-runner.pose[1],target[0]-runner.pose[0]),timeout=45.):return False
                from visual_target_alignment import align_pillar
                vision=base.get('pillar_alignment',{})
                if vision.get('enabled'):
                    plan.target_aligned=align_pillar(runner,vision,runner.pose[2],emit=emit)
                    if not plan.target_aligned:return False
            if not plan.release():
                emit('第1球出球指令失败，不开始第2次搜索')
                return False
        elif mechanism:
            emit('首球传球接口保留：不发出球串口，继续第2段流程；尚未确认首球离开机构')
        emit(f"返回 {cfg['return_point']} → 搜索至 {cfg['second_scan_end']}，途中遇篮球即停")
        heading = math.radians(cfg['second_scan_heading_deg'])
        if not plan.turn(heading):
            return False
        if not plan.move(cfg['return_point']):
            return False
        status = plan.scan(cfg['second_scan_end'], heading, detect=True, centered=False)
        if status != 'found':
            emit('第2段到终点仍未找到篮球或搜索失败，停车结束；不盲目铲球')
            return False
        emit('第2次铲球 → 传球点停车')
        if not pickup(cfg['second_scoop_m']):
            return False
        if not plan.move(plan.leg['pass_point']):
            return False
        emit('两次铲球运动及转运完成；第2次到传球点停车，不自动出球或回位')
        return True
    finally:
        for key, value in saved.items():
            setattr(runner.args, key, value)
        runner.stop()
