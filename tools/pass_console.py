#!/usr/bin/env python3
"""六按键传球/投篮控制台：复用设备检查与任务交接，透传脚本结果。"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import shlex
import time
from pathlib import Path
import subprocess
import threading
import uuid

from pass_health import HardwareNotReady, PassHealth, RosHealthMonitor

WS = Path(__file__).resolve().parent.parent
ACTIONS_FILE = Path(__file__).with_name('pass_actions.json')
SHOOT_ACTIONS_FILE = Path(__file__).with_name('shoot_actions.json')


def command_for(side, number, config_path=None):
    if side not in ('A', 'B') or type(number) is not int or number not in (1, 2, 3):
        raise ValueError('请选择 A/B 场地及回合 1/2/3')
    # 每次点击读取后端映射：改脚本、路径或参数均无需改页面或重启服务。
    actions = json.loads(Path(config_path or ACTIONS_FILE).read_text(encoding='utf-8'))
    command = actions[f'{side}{number}']
    if (not isinstance(command, list) or not command
            or any(not isinstance(arg, str) for arg in command) or not command[0]):
        raise ValueError('后端动作映射须为非空命令参数列表')
    return list(command)


def shoot_command_for(side, number, config_path=None):
    return command_for(side, number, config_path or SHOOT_ACTIONS_FILE)


def workspace_route_pids(proc_root=Path('/proc'), workspace=WS):
    """Find only this checkout's route runner, never boot/ROS or another checkout."""
    found = []
    allowed = {'tools/test_perimeter_ball.py', str(workspace/'tools/test_perimeter_ball.py')}
    for proc in proc_root.glob('[0-9]*'):
        try:
            args = (proc/'cmdline').read_bytes().decode().split('\0')
            if (proc/'cwd').resolve() == workspace.resolve() and allowed.intersection(args):
                found.append(int(proc.name))
        except (OSError, UnicodeError):
            pass
    return found


def cleanup_legacy_routes():
    # Only run from the real console entry point, never from tests or API creation.
    pids = workspace_route_pids()
    for sig, seconds in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.)):
        live = [pid for pid in pids if Path('/proc', str(pid)).exists()]
        for pid in live:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        until = time.monotonic()+seconds
        while time.monotonic() < until and any(Path('/proc', str(pid)).exists() for pid in live):
            time.sleep(.05)
    live = [pid for pid in pids if Path('/proc', str(pid)).exists()]
    if live:
        raise RuntimeError('旧路线尚未退出，控制台未启动: '+str(live))
    if pids:
        print('已清理旧路线进程: '+str(pids), flush=True)


class PassRuns:
    def __init__(self, command_factory=command_for, start_guard=None):
        self.command_factory = command_factory
        self.start_guard = start_guard
        self.runs = {}
        self.latest = None
        self.lock = threading.Lock()
        self.launch_lock = threading.Lock()
        self.execution_lock = threading.Lock()

    @staticmethod
    def _stop_process(process):
        if process is None or process.poll() is not None:
            return
        # Each script owns a fresh process group; no robot stack lives in it.
        for sig, timeout in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                pass
        raise RuntimeError('旧路线进程未退出，不能交接底盘控制')

    def start(self, side, number):
        command = self.command_factory(side, number)
        # Accept each click, but replace the preceding task instead of driving twice.
        with self.launch_lock:
            if self.start_guard is not None:
                self.start_guard()
            run_id = uuid.uuid4().hex
            run = dict(id=run_id, side=side, round=number, status='running',
                       returncode=None, lines=[], command=command, process=None,
                       cancelled=False)
            with self.lock:
                previous = [r for r in self.runs.values() if r['status'] == 'running']
                for old in previous:
                    old['cancelled'] = True
                    old['lines'].append('已收到新任务，结束当前路线后交接底盘控制')
                self.runs[run_id] = run
                self.latest = run_id
                if previous:
                    run['lines'].append('正在接管：先结束旧路线，再启动本次选择')
            try:
                for old in previous:
                    self._stop_process(old['process'])
            except Exception as exc:
                with self.lock:
                    run['status'] = 'failed'
                    run['returncode'] = 127
                    run['lines'].append(str(exc))
                raise
            threading.Thread(target=self._execute, args=(run,), daemon=True).start()
        return run_id

    def _execute(self, run):
        with self.execution_lock:
            code = 127
            try:
                env = dict(os.environ, PYTHONUNBUFFERED='1', PYTHONIOENCODING='utf-8')
                with self.lock:
                    if run['cancelled'] or self.latest != run['id']:
                        run['status'] = 'superseded'
                        run['returncode'] = None
                        run['lines'].append('本次任务在启动前已被新的选择替换，未运行脚本')
                        return
                    if self.start_guard is not None:
                        self.start_guard()  # Recheck after waiting for the preceding route.
                    run['lines'].append('执行目录: '+str(WS))
                    run['lines'].append('执行命令: '+shlex.join(run['command']))
                    process = subprocess.Popen(run['command'], cwd=WS, env=env,
                                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                               text=True, encoding='utf-8', errors='replace',
                                               start_new_session=True)
                    run['process'] = process
                for line in process.stdout:
                    with self.lock:
                        run['lines'].append(line.rstrip('\r\n'))
                code = process.wait()
            except Exception as exc:
                with self.lock:
                    run['lines'].append(f'执行失败：{exc}')
            with self.lock:
                run['returncode'] = code
                run['status'] = ('superseded' if run['cancelled'] else
                                 'succeeded' if code == 0 else 'failed')
                run['lines'].append('旧路线已退出，控制权已交接' if run['cancelled'] else
                                    f'执行结束，退出码 {code}')

    def snapshot(self, run_id=None, after=0):
        with self.lock:
            selected = run_id or self.latest
            if selected is None:
                return dict(status='idle', lines=[], cursor=0)
            run = self.runs[selected]
            return dict(id=run['id'], side=run['side'], round=run['round'],
                        status=run['status'], returncode=run['returncode'],
                        command=list(run['command']),
                        lines=run['lines'][after:], cursor=len(run['lines']))


def make_app(runs=None, boot_watch=None, health=None, task='pass', tasks=None):
    from flask import Flask, jsonify, request, send_file
    app = Flask(__name__)
    if task not in ('pass', 'shoot'):
        raise ValueError('任务须为 pass/shoot')
    runs = runs if runs is not None else PassRuns(
        shoot_command_for if task == 'shoot' else command_for)
    health = health if health is not None else PassHealth()
    from console_trip import ConsoleTasks, TaskBusy
    tasks = tasks if tasks is not None else ConsoleTasks(runs, health,
        json.loads((WS/'tools/fallback_config.json').read_text()))
    app.extensions['console_tasks'] = tasks

    def hardware():
        return health.snapshot(boot_watch.snapshot() if boot_watch is not None else None)

    def require_ready():
        current = hardware()
        if not current['can_run']:
            raise HardwareNotReady(current)

    runs.start_guard = require_ready

    @app.post('/api/console/prepare-update')
    def prepare_update():
        if request.remote_addr not in ('127.0.0.1', '::1') or not request.is_json:
            return jsonify(error='仅允许本机更新控制界面'), 403
        with tasks.lock:
            if tasks.activity() != 'idle':
                return jsonify(error='任务正在执行，不能更新控制界面'), 409
            tasks.maintenance = True
            if boot_watch is not None:
                boot_watch.stop_event.set()
        return jsonify(ok=True)

    @app.post('/api/console/prepare-fallback')
    def retired_handover():
        return jsonify(error='独立保底已移除，请使用投篮页面底部的底盘往返'), 410

    @app.get('/api/shoot/chassis-trip/state')
    def trip_state():
        return jsonify(tasks.snapshot())

    @app.post('/api/shoot/chassis-trip/run')
    def trip_run():
        if task != 'shoot':
            return jsonify(error='请从投篮控制页面启动底盘往返'), 409
        if not request.is_json or request.get_json(silent=True) != {}:
            return jsonify(error='无效的往返执行请求'), 400
        try:
            return jsonify(tasks.start_trip()), 202
        except TaskBusy as exc:
            return jsonify(error=str(exc), **tasks.snapshot()), 409

    @app.post('/api/shoot/chassis-trip/stop')
    def trip_stop():
        return jsonify(tasks.stop_trip())

    @app.after_request
    def no_stale_status(response):
        response.headers['Cache-Control'] = 'no-store'
        return response

    @app.get('/')
    def index():
        return send_file(WS / 'tools' / ('shoot_ui.html' if task == 'shoot' else 'console_ui.html'))

    @app.post(f'/api/{task}/run')
    def start():
        data = request.get_json(silent=True) or {}
        if not isinstance(data, dict):
            return jsonify(error='请求须为 JSON 对象'), 400
        try:
            run_id = (tasks.start_shoot(data.get('side'), data.get('round')) if task == 'shoot'
                      else runs.start(data.get('side'), data.get('round')))
        except TaskBusy as exc:
            return jsonify(error=str(exc)), 409
        except (ValueError, TypeError) as exc:
            return jsonify(error=str(exc)), 400
        except HardwareNotReady as exc:
            return jsonify(error=str(exc), hardware=exc.hardware), 409
        except (OSError, KeyError, RuntimeError) as exc:
            return jsonify(error=f'后端脚本映射不可用：{exc}'), 500
        return jsonify(id=run_id, task=task), 202

    @app.get(f'/api/{task}/state')
    def state():
        try:
            after = max(0, int(request.args.get('after', 0)))
            result = runs.snapshot(request.args.get('id'), after)
            if boot_watch is not None:
                result['boot'] = boot_watch.snapshot()
            result['hardware'] = hardware()
            result['task'] = task
            result['chassis_trip'] = tasks.snapshot()
            return jsonify(result)
        except (KeyError, ValueError):
            return jsonify(error='执行记录不存在或游标无效', hardware=hardware()), 404

    if task == 'shoot':
        @app.post('/api/pass/run')
        def old_pass_run():
            return jsonify(error='控制台已切换为投篮，请刷新页面后选择投篮回合', task='shoot'), 409

    @app.get('/api/hardware/state')
    def hardware_state():
        return jsonify(hardware())

    @app.get('/api/boot/state')
    def boot_state():
        return jsonify(boot_watch.snapshot() if boot_watch is not None else {'status': 'disabled'})

    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['pass', 'shoot'], default='pass')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args(argv)
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    from pass_boot_watch import BootWatch
    cleanup_legacy_routes()
    boot_watch = BootWatch()
    health = PassHealth()
    from console_trip import ConsoleTasks
    runs = PassRuns(shoot_command_for if args.task == 'shoot' else command_for)
    tasks = ConsoleTasks(runs, health, json.loads((WS/'tools/fallback_config.json').read_text()))
    monitor = RosHealthMonitor(health, tasks=tasks)
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    monitor.start()
    boot_watch.start()
    try:
        make_app(runs=runs, boot_watch=boot_watch, health=health, task=args.task, tasks=tasks).run(
            host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        boot_watch.stop_event.set()
        monitor.close()


if __name__ == '__main__':
    raise SystemExit(main())
