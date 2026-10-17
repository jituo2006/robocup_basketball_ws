"""Offline route calibration checks; no ROS or hardware runner."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'tools'))
import match_round_route as route


class ShootCalibrationTests(unittest.TestCase):
    def test_scoped_coordinates(self):
        for side in ('A', 'B'):
            for task in ('pass', 'shoot'):
                for number in (1, 2, 3):
                    with self.subTest(side=side, task=task, number=number):
                        cfg = route.load_config(start_point=side, round_number=number, task=task)
                        y = (1.2 if task == 'shoot' and number == 2 else .9) * (1 if side == 'A' else -1)
                        points = route.search_plan(cfg, 'middle', 0, number, task)
                        self.assertTrue(all(point[0][1] == y for point in points))
                        corridor = [6.4, 2.9] if (side, task, number) == ('A', 'shoot', 3) else [6.4, 3.05 if side == 'A' else -3.05]
                        self.assertEqual(cfg['corridor'], corridor)
                        route.validate_mode(cfg, task, 'cycle', 'match', mechanism_enabled=True)

    def test_clockwise_after_aim_and_failure(self):
        class Runner:
            def __init__(self, point, fail):
                self.pose = [*point, 0.]
                self.args = SimpleNamespace(route_intake=True)
                self.turns = []
                self.fail = fail

            def rotate_to(self, yaw, timeout):
                self.turns.append(yaw)
                if self.fail and len(self.turns) == 2:
                    return False
                self.pose[2] = yaw
                return True

            def stop(self):
                pass

        def align(runner, vision, heading, emit):
            runner.pose[2] = route.wrap(heading + .02)
            return True

        for side in ('A', 'B'):
            for task in ('shoot', 'pass'):
                for pillar in (False, True):
                    for fail in (False, True):
                        with self.subTest(side=side, task=task, pillar=pillar, fail=fail):
                            cfg = route.load_config(start_point=side, round_number=2, task=task)
                            plan = route.RoundRoute.__new__(route.RoundRoute)
                            plan.cfg, plan.task, plan.mode, plan.release_kind = cfg, task, 'cycle', 'match'
                            plan.r = Runner(cfg['release'][task]['point'], fail)
                            plan.frame = SimpleNamespace(point=lambda p: p, yaw=lambda yaw: yaw)
                            plan.move_path = lambda points: True
                            plan.crossing_via = lambda x: []
                            plan.say = lambda message: None
                            plan.needs_pillar_alignment = lambda: pillar
                            with patch.object(route, 'align_pillar', align):
                                ok = plan.action_point()
                            if task == 'shoot':
                                self.assertEqual(len(plan.r.turns), 2)
                                aligned = route.wrap(plan.r.turns[0] + (.02 if pillar else 0.))
                                self.assertAlmostEqual(route.wrap(plan.r.turns[1] - aligned), -math.radians(5))
                                self.assertEqual(ok, not fail)
                                self.assertEqual(plan.heading_ready, not fail)
                            else:
                                self.assertEqual(len(plan.r.turns), 1)
                                self.assertTrue(ok)

    def test_actual_entry_passes_round_and_task(self):
        tree = ast.parse((WS / 'tools/test_perimeter_ball.py').read_text())
        node = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'round_cfg' for t in n.targets)
                    and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
                    and n.value.func.id == 'load_config')
        for side in ('A', 'B'):
            scope = {'load_config': route.load_config,
                     'args': SimpleNamespace(round_config=str(WS / 'tools/match_rounds.json'),
                                             start_point=side, route='round3', round_task='shoot')}
            exec(compile(ast.Module(body=[node], type_ignores=[]), 'actual_entry', 'exec'), scope)
            self.assertEqual(scope['round_cfg']['corridor'], [6.4, 2.9] if side == 'A' else [6.4, -3.05])


if __name__ == '__main__':
    unittest.main()
