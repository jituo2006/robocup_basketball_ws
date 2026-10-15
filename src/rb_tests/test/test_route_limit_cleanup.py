"""检验已删除的附加门槛不会继续卡住当前路线；安全反馈仍受检验。"""
import json
from types import SimpleNamespace as NS

import pytest

from test_seek_observations import load
from test_match_round_route import console
from match_round_route import load_config


@pytest.mark.parametrize('route', ['fixed', 'first-pass', 'round1', 'round2', 'round3'])
def test_inactive_and_retired_parameters_do_not_block_route_start(route, monkeypatch):
    mod = load()
    monkeypatch.setattr(mod.subprocess, 'run', lambda *a, **kw: NS(returncode=1))
    # 仿真节点均未启动：应到达节点检查并返回1，而非旧参数导致解析退出。
    assert mod.main(['--route', route, '--dry', '--yes', '--no-mechanism',
                     '--goto-stop-speed', '-1', '--goto-stop-yaw-rate', '-1',
                     '--goto-settle-seconds', '-1', '--goto-velocity-window', '10',
                     '--goto-correction-speed', '-1', '--route-rear-center-frames', '0',
                     '--route-scan-start-offset', '6', '--servo-move', '-1',
                     '--stable-frames', '0', '--settle-seconds', '-1']) == 1


def test_route_speed_is_tunable_above_former_quarter_meter_cap(tmp_path):
    cfg = load_config(); cfg['speed_mps'] = .4
    p = tmp_path / 'route.json'; p.write_text(json.dumps(cfg))
    assert load_config(p)['speed_mps'] == .4
    args = console().fixed_seek_tool_args({'scoop_speed': .3})
    assert args[args.index('--route-scoop-speed')+1] == '0.3'


@pytest.mark.parametrize('speed', [0., -1., float('nan'), float('inf')])
def test_removed_speed_cap_does_not_accept_invalid_control_values(tmp_path, speed):
    cfg = load_config(); cfg['speed_mps'] = speed
    p = tmp_path / 'route.json'; p.write_text(json.dumps(cfg))
    with pytest.raises(ValueError): load_config(p)
    with pytest.raises(ValueError): console().fixed_seek_tool_args({'scoop_speed': speed})
