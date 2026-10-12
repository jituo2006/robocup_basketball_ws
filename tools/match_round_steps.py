"""网页/CLI 分步实验的最小运行状态：固定本轮起点和下一阶段，不重置 ROS 位姿。"""
import json
import math
from pathlib import Path

from match_round_route import RoundRoute, StartFrame, search_order, validate_mode, TASK_LABELS, wrap

STATE_DIR = Path('/tmp/rb_match_rounds')


def steps(round_number):
    if round_number == 1:
        return [
            ('entry', '入场：右转90°到入场点'),
            ('action', '预装球：到传球/投篮点'),
            ('release', '预装球：放球/出球'),
            ('search', '第2颗：中线横扫找球'),
            ('align', '第2颗：转180°、后摄对正'),
            ('approach', '第2颗：低速靠近吸球'),
            ('carry', '第2颗：持球到动作点'),
            ('release', '第2颗：放球/出球'),
            ('home', '回出发区、恢复初始朝向')]
    if round_number not in (2, 3):
        raise ValueError('回合须为1、2或3')
    result = [('entry', '入场：右转90°到入场点')]
    for ball in (1, 2):
        result += [('search', f'第{ball}颗：搜索目标球'),
                   ('align', f'第{ball}颗：转180°、后摄对正'),
                   ('approach', f'第{ball}颗：低速靠近吸球'),
                   ('carry', f'第{ball}颗：持球到动作点'),
                   ('release', f'第{ball}颗：放球/出球')]
    return result + [('home', '回出发区、恢复初始朝向')]


def session_path(round_number, task):
    steps(round_number)
    if task not in TASK_LABELS:
        raise ValueError('任务须为 pass/shoot')
    return STATE_DIR / f'round{round_number}_{task}.json'


def load_session(round_number, task):
    p = session_path(round_number, task)
    if not p.exists():
        return None
    state = json.loads(p.read_text(encoding='utf-8'))
    if state.get('version') != 1 or state.get('round') != round_number or state.get('task') != task:
        raise ValueError('分步状态不匹配，请从第1步重新开始')
    pose = state.get('start_pose', [])
    if len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
        raise ValueError('分步起点无效，请从第1步重新开始')
    return state


def save_session(state):
    p = session_path(state['round'], state['task'])
    p.parent.mkdir(parents=True, exist_ok=True)
    temp = p.with_suffix('.tmp')
    temp.write_text(json.dumps(state, ensure_ascii=False), encoding='utf-8')
    temp.replace(p)


def clear_session(round_number, task):
    session_path(round_number, task).unlink(missing_ok=True)


def validate_step(round_number, task, index, cfg, release_kind):
    if isinstance(index, bool) or not isinstance(index, int) or not 1 <= index <= len(steps(round_number)):
        raise ValueError('阶段编号无效')
    validate_mode(cfg, task, 'collect', release_kind)
    if index != 1:
        state = load_session(round_number, task)
        if state is None:
            raise ValueError('本轮尚未建立起点，请把车放回出发区并运行第1步')
        if index != state['next_step']:
            raise ValueError(f"请先运行第{state['next_step']}步；已完成的阶段不能直接重复，重新实验请从第1步开始")
    if steps(round_number)[index-1][0] == 'release':
        validate_mode(cfg, task, 'cycle', release_kind)


def workflow_state(round_number, task):
    state = load_session(round_number, task)
    next_step = state['next_step'] if state else 1
    return {'round': round_number, 'started': state is not None, 'next_step': next_step,
            'finished': next_step > len(steps(round_number)),
            'steps': [{'index': i, 'kind': kind, 'label': label, 'done': i < next_step}
                      for i, (kind, label) in enumerate(steps(round_number), 1)]}


def execute_step(runner, cfg, round_number, task, index, release_kind='drop', emit=print):
    validate_step(round_number, task, index, cfg, release_kind)
    if index == 1:
        state = {'version': 1, 'round': round_number, 'task': task,
                 'start_pose': list(runner.pose), 'next_step': 1,
                 'trail': [], 'search_origin': None, 'area': None}
        # 从第1步重试必须使旧阶段失效，失败后不能接着用上一轮状态。
        save_session(state)
    else:
        state = load_session(round_number, task)
    plan = RoundRoute(runner, cfg, round_number, task, 'collect', emit=emit,
                      release_kind=release_kind)
    plan.frame = StartFrame(state['start_pose'], cfg['start'])
    runner.round_start_frame = plan.frame
    runner.args.ball_label = TASK_LABELS[task]
    runner.args.speed = cfg['speed_mps']
    plan.trail = [tuple(p) for p in state['trail']]
    plan.search_origin = state['search_origin']
    kind, label = steps(round_number)[index-1]
    plan.say(f'阶段 {index}：{label}')
    runner.stop()
    ok = False
    if kind == 'entry':
        runner.spin(cfg['start_delay_s'])
        ok = plan.turn(-math.pi/2) and plan.move(cfg['entry'])
    elif kind == 'action':
        ok = plan.action_point()
    elif kind == 'search':
        cycle = sum(s[0] == 'search' for s in steps(round_number)[:index-1])
        # 已在右侧动作点时先退到 T，再回 E，避免直切绿色干扰球线。
        if task == 'shoot' and (round_number == 1 or cycle > 0):
            if not plan.move(cfg['corridor']):
                return False
        for area in search_order(round_number, task):
            result = plan.search_area(area, cycle, search_only=True)
            if result == 'failed':
                break
            if result == 'found':
                state['area'] = area
                ok = True
                break
        if not ok:
            plan.say('没有完成目标搜索，本阶段未推进；可重试当前阶段')
    elif kind == 'align':
        if plan.search_origin is None:
            raise ValueError('缺少前摄停车点，请重新从本轮第1步开始')
        ok = runner.rotate_to(wrap(plan.search_origin[2]+math.pi),
                              timeout=45.) and runner.rear_visual_scoop(approach=False)
        if ok:
            state['aligned_pose'] = list(runner.pose)
    elif kind == 'approach':
        # 沿上一步实际对正的方向靠近；此阶段不再读取 YOLO。
        if state['area'] is None or 'aligned_pose' not in state:
            raise ValueError('请先完成后摄对正阶段')
        if abs(wrap(runner.pose[2]-state['aligned_pose'][2])) > runner.args.route_heading_tol:
            raise ValueError('车头已偏离上一阶段对正方向，请重新测试本轮')
        distance = cfg[state['area']]['scoop_m']
        ax, ay, heading = state['aligned_pose']
        travelled = -(runner.pose[0]-ax)*math.cos(heading)-(runner.pose[1]-ay)*math.sin(heading)
        remaining = distance-travelled
        if remaining <= runner.args.step_tol:
            ok = True
        else:
            # 暂停/失败后重试本阶段只补剩余距离，避免再走一整段。
            runner.args.route_scoop_distance = remaining
            runner.args.route_scoop_timeout = max(15., remaining/runner.args.route_scoop_speed+5.)
            ok = runner.aligned_intake_approach()
    elif kind == 'carry':
        ok = plan.unwind() and plan.action_point()
    elif kind == 'release':
        # 可在填好 aim 后切换正式出球；放球联调只使用已验证的文本。
        if release_kind == 'match' and not plan.action_point():
            return False
        ok = plan.release()
    elif kind == 'home':
        ok = plan.home()
    runner.stop()
    if ok:
        state['next_step'] = index+1
        state['trail'] = [list(p) for p in plan.trail]
        state['search_origin'] = plan.search_origin
        save_session(state)
        plan.say('本阶段完成，已停车，等待下一阶段按钮')
    return ok
