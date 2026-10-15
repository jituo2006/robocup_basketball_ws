#!/usr/bin/env python3
"""六按键传球控制台：直接运行既有脚本，透传输出和退出码。"""
from __future__ import annotations

import argparse
import json
import logging
import os
from pathlib import Path
import subprocess
import threading
import uuid

WS = Path(__file__).resolve().parent.parent
ACTIONS_FILE = Path(__file__).with_name('pass_actions.json')


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


class PassRuns:
    def __init__(self, command_factory=command_for):
        self.command_factory = command_factory
        self.runs = {}
        self.latest = None
        self.lock = threading.Lock()

    def start(self, side, number):
        command = self.command_factory(side, number)
        run_id = uuid.uuid4().hex
        run = dict(id=run_id, side=side, round=number, status='running',
                   returncode=None, lines=[], command=command)
        with self.lock:
            self.runs[run_id] = run
            self.latest = run_id
        threading.Thread(target=self._execute, args=(run,), daemon=True).start()
        return run_id

    def _execute(self, run):
        try:
            env = dict(os.environ, PYTHONUNBUFFERED='1')
            process = subprocess.Popen(run['command'], cwd=WS, env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding='utf-8', errors='replace')
            for line in process.stdout:
                with self.lock:
                    run['lines'].append(line.rstrip('\r\n'))
            code = process.wait()
        except Exception as exc:
            with self.lock:
                run['lines'].append(f'执行失败：{exc}')
            code = 127
        with self.lock:
            run['returncode'] = code
            run['status'] = 'succeeded' if code == 0 else 'failed'
            run['lines'].append(f'执行结束，退出码 {code}')

    def snapshot(self, run_id=None, after=0):
        with self.lock:
            selected = run_id or self.latest
            if selected is None:
                return dict(status='idle', lines=[], cursor=0)
            run = self.runs[selected]
            return dict(id=run['id'], side=run['side'], round=run['round'],
                        status=run['status'], returncode=run['returncode'],
                        lines=run['lines'][after:], cursor=len(run['lines']))


def make_app(runs=None):
    from flask import Flask, jsonify, request, send_file
    app = Flask(__name__)
    runs = runs if runs is not None else PassRuns()

    @app.get('/')
    def index():
        return send_file(WS / 'tools' / 'console_ui.html')

    @app.post('/api/pass/run')
    def start():
        data = request.get_json(silent=True) or {}
        if not isinstance(data, dict):
            return jsonify(error='请求须为 JSON 对象'), 400
        try:
            run_id = runs.start(data.get('side'), data.get('round'))
        except (ValueError, TypeError) as exc:
            return jsonify(error=str(exc)), 400
        except (OSError, KeyError) as exc:
            return jsonify(error=f'后端脚本映射不可用：{exc}'), 500
        return jsonify(id=run_id), 202

    @app.get('/api/pass/state')
    def state():
        try:
            after = max(0, int(request.args.get('after', 0)))
            return jsonify(runs.snapshot(request.args.get('id'), after))
        except (KeyError, ValueError):
            return jsonify(error='执行记录不存在或游标无效'), 404

    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args(argv)
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    make_app().run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == '__main__':
    raise SystemExit(main())
