"""每轮三个阶段：第一颗球、第二颗球、回位；内部动作进度用于失败后续做。"""
import json
import math
from pathlib import Path

from match_round_route import RoundRoute, StartFrame, search_order, validate_mode, TASK_LABELS, wrap

STATE_DIR = Path('/tmp/rb_match_rounds')


def steps(round_number, first_ball='preloaded'):
    if round_number not in (1, 2, 3):
        raise ValueError('回合须为1、2或3')
    first = '入场 → 预装球到动作点 → 放球' if round_number == 1 and first_ball == 'preloaded' else '入场 → 取第1球 → 到动作点放球'
    return [('first', first), ('second', '取第2球 → 到动作点放球'),
            ('home', '回出发区 → 恢复初始朝向')]


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
    if state.get('version') != 2:
        return None  # 旧的细分步骤不混入新的三个阶段。
    if state.get('round') != round_number or state.get('task') != task:
        raise ValueError('阶段状态不匹配，请从第1阶段重新开始')
    pose = state.get('start_pose', [])
    if len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
        raise ValueError('阶段起点无效，请从第1阶段重新开始')
    return state


def save_session(state):
    p = session_path(state['round'], state['task'])
    p.parent.mkdir(parents=True, exist_ok=True)
    temp = p.with_suffix('.tmp')
    temp.write_text(json.dumps(state, ensure_ascii=False), encoding='utf-8')
    temp.replace(p)


def clear_session(round_number, task):
    session_path(round_number, task).unlink(missing_ok=True)


def validate_step(round_number, task, index, cfg, release_kind, mechanism_enabled=True):
    if isinstance(index, bool) or not isinstance(index, int) or not 1 <= index <= 3:
        raise ValueError('阶段编号须为1、2或3')
    validate_mode(cfg, task, 'collect', release_kind, mechanism_enabled=mechanism_enabled)
    state = load_session(round_number, task)
    first_ball = cfg.get('round1_first_ball', 'preloaded')
    if (state and state['next_step'] <= 3 and round_number == 1
            and state.get('first_ball', 'preloaded') != first_ball):
        raise ValueError('首球方式已改变，请摆回出发区并点击“重开本轮”')
    if index != 1:
        state = load_session(round_number, task)
        if state is None:
            raise ValueError('本轮尚未建立起点，请把车放回出发区并运行第1阶段')
        if index != state['next_step']:
            raise ValueError(f"请先运行第{state['next_step']}阶段；重新实验请摆回出发点运行第1阶段")
    if index in (1, 2):
        validate_mode(cfg, task, 'cycle', release_kind, mechanism_enabled=mechanism_enabled)


def workflow_state(round_number, task, first_ball='preloaded'):
    state = load_session(round_number, task)
    next_step = state['next_step'] if state else 1
    return {'round': round_number, 'started': state is not None, 'next_step': next_step,
            'finished': next_step > 3, 'last_error': (state or {}).get('last_error', ''),
            'active_action': (state or {}).get('active_action', ''),
            'first_ball': (state or {}).get('first_ball', first_ball),
            'steps': [{'index': i, 'kind': kind, 'label': label, 'done': i < next_step}
                      for i, (kind, label) in enumerate(steps(round_number, (state or {}).get('first_ball', first_ball)), 1)]}


def execute_step(runner, cfg, round_number, task, index, release_kind='drop', emit=print,
                 continuous=False):
    validate_step(round_number, task, index, cfg, release_kind,
                  mechanism_enabled=getattr(runner.args, 'route_intake', True))
    previous = load_session(round_number, task)
    if index == 1 and (previous is None or previous['next_step'] != 1):
        state = {'version': 2, 'round': round_number, 'task': task,
                 'start_pose': list(runner.pose), 'next_step': 1, 'substep': 0,
                 'first_ball': cfg.get('round1_first_ball', 'preloaded'), 'last_error': '',
                 'trail': [], 'search_origin': None, 'area': None}
        save_session(state)
    else:
        state = previous
    state.setdefault('first_ball', 'preloaded')
    plan = RoundRoute(runner, cfg, round_number, task, 'collect', emit=emit,
                      release_kind=release_kind)
    plan.frame = StartFrame(state['start_pose'], cfg['start'])
    runner.round_start_frame = plan.frame
    runner.args.ball_label = TASK_LABELS[task]
    runner.args.speed = cfg['speed_mps']
    plan.trail = [tuple(p) for p in state['trail']]
    plan.search_origin = state['search_origin']
    plan.say(f'阶段 {index}：{steps(round_number, state["first_ball"])[index-1][1]}')
    runner.stop()
    if index == 3:
        actions = ['home']
    elif index == 1 and round_number == 1 and state['first_ball'] == 'preloaded':
        actions = ['entry', 'action', 'release']
    else:
        actions = (['entry'] if index == 1 else []) + ['search', 'align', 'approach', 'carry', 'release']
    for kind in actions[state['substep']:]:
        state['active_action'], state['last_error'] = kind, ''
        save_session(state)
        ok = False
        try:
            if kind == 'entry':
                runner.spin(cfg['start_delay_s'])
                ok = plan.turn(-math.pi/2) and plan.move(cfg['entry'])
            elif kind == 'action':
                ok = plan.action_point()
            elif kind == 'search':
                cycle = 0 if round_number == 1 else index-1
                if task == 'shoot' and index == 2:
                    if not plan.move(cfg['corridor']):
                        raise RuntimeError('去搜索通道未完成')
                for area in search_order(round_number, task):
                    result = plan.search_area(area, cycle, search_only=True)
                    if result == 'failed':
                        break
                    if result == 'found':
                        state['area'] = area
                        ok = True
                        break
            elif kind == 'align':
                if plan.search_origin is None:
                    raise ValueError('缺少前摄停车点，请重新开始本轮')
                ok = runner.rotate_to(wrap(plan.search_origin[2]+math.pi), timeout=45.) and runner.rear_visual_scoop(approach=False)
                if ok:
                    state['aligned_pose'] = list(runner.pose)
            elif kind == 'approach':
                if state['area'] is None or 'aligned_pose' not in state:
                    raise ValueError('尚未完成后摄对正')
                ax, ay, heading = state['aligned_pose']
                if abs(wrap(runner.pose[2]-heading)) > runner.args.route_heading_tol:
                    raise ValueError('车头已偏离对正方向，请重新开始本轮')
                travelled = -(runner.pose[0]-ax)*math.cos(heading)-(runner.pose[1]-ay)*math.sin(heading)
                remaining = cfg[state['area']]['scoop_m']-travelled
                if remaining <= runner.args.step_tol:
                    ok = True
                else:
                    runner.args.route_scoop_distance = remaining
                    runner.args.route_scoop_timeout = max(15., remaining/runner.args.route_scoop_speed+5.)
                    ok = runner.aligned_intake_approach()
            elif kind == 'carry':
                ok = plan.unwind() and plan.action_point()
            elif kind == 'release':
                # 失败后重试 release 会新建 plan，补做对准；同次 carry 已完成则不重复走动作点。
                if (release_kind == 'match' and getattr(runner.args, 'route_intake', True)
                        and not plan.target_aligned and not plan.action_point()):
                    raise RuntimeError('动作点对准未完成')
                ok = plan.release()
            elif kind == 'home':
                ok = plan.home()
        except (RuntimeError, ValueError) as exc:
            plan.last_error = str(exc)
            ok = False
        runner.stop()
        if not ok:
            state['trail'] = [list(p) for p in plan.trail]
            state['last_error'] = plan.last_error or f'{kind} 未完成；请查看路线日志'
            save_session(state)
            plan.say(f'{kind} 未完成，本阶段停车；重试会沿用本轮起点和已完成动作')
            return False
        state['substep'] += 1
        state['trail'] = [list(p) for p in plan.trail]
        state['search_origin'] = plan.search_origin
        save_session(state)
    state['next_step'] = index+1
    state['substep'] = 0
    state['active_action'], state['last_error'] = '', ''
    save_session(state)
    plan.say('本阶段完成，自动进入下一阶段' if continuous and index < 3
             else '本轮完成，已回出发区并恢复初始朝向' if continuous
             else '本阶段完成，已停车，等待下一阶段按钮')
    return True


def execute_cycle(runner, cfg, round_number, task, release_kind='drop', emit=print):
    """从本轮起点连续处理两颗球并回位，与三个阶段按钮共用动作实现。"""
    validate_mode(cfg, task, 'cycle', release_kind,
                  mechanism_enabled=getattr(runner.args, 'route_intake', True))
    steps(round_number, cfg.get('round1_first_ball', 'preloaded'))
    clear_session(round_number, task)
    emit(f'第{round_number}回合整轮开始：以当前位姿建立起点；两次处理球后回位')
    try:
        for index in (1, 2, 3):
            emit(f'整轮进度 {index}/3：{steps(round_number, cfg.get("round1_first_ball", "preloaded"))[index-1][1]}')
            if not execute_step(runner, cfg, round_number, task, index, release_kind,
                                emit=emit, continuous=True):
                state = load_session(round_number, task)
                emit(f'整轮在第{index}阶段停止：{state["last_error"]}')
                return False
        return True
    finally:
        runner.stop()
