"""每轮三个阶段：第一颗球、第二颗球、回位；内部动作进度用于失败后续做。"""
import json
import math
from pathlib import Path

from match_round_route import RoundRoute, StartFrame, search_order, validate_mode, TASK_LABELS, wrap, start_wait_seconds

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
    # 【整轮程序的实际动作衔接】阶段按钮和 execute_cycle 都用这里。
    # 取球动作：search(中线第一次横移也识球即停) → align(转180°+后摄横移)
    #          → approach(独立距离后退) → carry(走到动作点) → release(出球)。
    # 任一动作失败就停止本阶段，后续动作不会继续；注释中的参数可由界面覆盖。
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
    plan.carrying = state.get('carrying', index < 3 and (
        (index == 1 and round_number == 1 and state['first_ball'] == 'preloaded')
        or state.get('active_action') in ('carry', 'release')))
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
        # 正式首球预装模式：第一阶段直接到动作点，不执行寻球/后退。
        # 首球 search 联调模式才执行下面完整的 search→align→approach。
        actions = ['entry', 'action', 'release']
    else:
        actions = (['entry'] if index == 1 else []) + ['search', 'align', 'approach', 'carry', 'release']
    for kind in actions[state['substep']:]:
        # 分阶段重试从已保存的 substep 续做；整轮入口会先清空会话，从头执行。
        state['active_action'], state['last_error'] = kind, ''
        save_session(state)
        ok = False
        try:
            if kind == 'entry':
                # 起步默认先静止5秒，随后右转并进场；等待期间也会处理定位回调。
                runner.spin(start_wait_seconds(cfg, runner.args))
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
                # 【两处关口】and 会短路：转向未成功就不调用后摄对正。
                # approach=False 只做左右横移，成功后返回；本行不会前后移动。
                # 后摄有效目标居中即可；没有连续帧数、固定总时间或净位移上限。
                ok = runner.rotate_to(wrap(plan.search_origin[2]+math.pi), timeout=45.) and runner.rear_visual_scoop(approach=False)
                if ok:
                    # 保存对正完成的位置/朝向，下一动作从这里计算独立后退距离。
                    state['aligned_pose'] = list(runner.pose)
            elif kind == 'approach':
                # 【对正后向球靠近的入口】这一动作不再读取YOLO，后摄漏检不影响它。
                if state['area'] is None or 'aligned_pose' not in state:
                    raise ValueError('尚未完成后摄对正')
                ax, ay, heading = state['aligned_pose']
                if abs(wrap(runner.pose[2]-heading)) > runner.args.route_heading_tol:
                    # 续跑先恢复已保存朝向，再按原对正点扣除已走距离。
                    if not runner.rotate_to(heading, timeout=45.):
                        raise RuntimeError('恢复对正方向未完成')
                travelled = -(runner.pose[0]-ax)*math.cos(heading)-(runner.pose[1]-ay)*math.sin(heading)
                # 扣除已经后退的投影位移，避免分阶段重试又走完整一段。
                # 总距离来自本区域 scoop_m，中线默认0.70m，容差默认2cm。
                remaining = cfg[state['area']]['scoop_m']-travelled
                if remaining <= runner.args.step_tol:
                    # 已靠近到位，不重复行驶；仍补做本动作的铲球→等1秒→放下。
                    ok = runner.aligned_intake_approach(approach=False)
                else:
                    runner.args.route_scoop_distance = remaining
                    runner.args.route_scoop_timeout = max(15., remaining/runner.args.route_scoop_speed+5.)
                    # 先按距离后退，完成后才发铲球序列，等1秒再放下。
                    # 机构关闭则只靠近；实际负vx见 rear_distance_scoop。
                    ok = runner.aligned_intake_approach()
            elif kind == 'carry':
                ok = plan.unwind() and plan.action_point()
            elif kind == 'release':
                # 重试会新建 plan，须恢复出球朝向；同次 carry 已完成则不重复走动作点。
                if (release_kind == 'match' and getattr(runner.args, 'route_intake', True)
                        and (not plan.heading_ready or (plan.needs_pillar_alignment() and not plan.target_aligned))
                        and not plan.action_point()):
                    raise RuntimeError('动作点对准未完成')
                ok = plan.release()
            elif kind == 'home':
                ok = plan.home()
        except (RuntimeError, ValueError) as exc:
            # 定位过期/跳变、朝向或前置状态异常都会到这里，转成动作失败。
            plan.last_error = str(exc)
            ok = False
        runner.stop()
        if not ok:
            # 【失败不自动跨过去】保存重试位置后返回 False；因此 align 失败
            # 会表现为“转身以后不靠近”，不能据此认定 approach 已执行。
            state['trail'] = [list(p) for p in plan.trail]
            state['last_error'] = plan.last_error or f'{kind} 未完成；请查看路线日志'
            save_session(state)
            plan.say(f'{kind} 未完成，本阶段停车；重试会沿用本轮起点和已完成动作')
            return False
        if kind == 'approach':
            plan.carrying = True
        state['carrying'] = plan.carrying
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
    # 整轮自动连续跑三阶段，不需要人工点“下一步”；启动时清掉旧续跑状态。
    # 但 execute_step 返回 False 时仍立即退出，不会自动忽略失败进入下一阶段。
    validate_mode(cfg, task, 'cycle', release_kind,
                  mechanism_enabled=getattr(runner.args, 'route_intake', True))
    steps(round_number, cfg.get('round1_first_ball', 'preloaded'))
    clear_session(round_number, task)
    emit(f'第{round_number}回合整轮开始：以当前位姿建立起点；两次处理球后回位')
    profile = cfg.get('speed_profiles', {}).get(str(round_number), {})
    if profile:
        emit(f"本轮速度：空车走线{profile.get('travel_mps', cfg['speed_mps'])}m/s，"
             f"载球转运{profile.get('carry_mps', cfg['speed_mps'])}m/s，"
             f"识球扫描{profile.get('scan_mps', cfg['speed_mps'])}m/s；"
             f"转向上限{runner.args.max_ang}rad/s")
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
