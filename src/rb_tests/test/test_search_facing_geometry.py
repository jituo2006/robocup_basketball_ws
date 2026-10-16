"""前摄扫描方向应朝放球线；纯几何回归，无 ROS 或硬件。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from match_round_route import load_config, search_plan, wrap


class SearchFacingGeometryTests(unittest.TestCase):
    def test_front_camera_faces_ball_line_from_both_scan_sides(self):
        for side in ('A', 'B'):
            cfg = load_config(start_point=side)
            for task in ('pass', 'shoot'):
                for number in (1, 2, 3):
                    for cycle in (0, 1):
                        for area in ('inner', 'arc'):
                            with self.subTest(side=side, task=task, number=number,
                                              cycle=cycle, area=area):
                                for (x, y), yaw in search_plan(cfg, area, cycle, number, task):
                                    if area == 'inner':
                                        dx, dy = 9. - x, 0.
                                    else:
                                        cx, cy = cfg['arc']['center']
                                        radius = math.hypot(x-cx, y-cy)
                                        dx = (x-cx) * (3.75/radius-1.)
                                        dy = (y-cy) * (3.75/radius-1.)
                                    # 球线在前摄正前方：前向投影为正，横向误差为零。
                                    self.assertGreater(dx*math.cos(yaw)+dy*math.sin(yaw), 0.)
                                    self.assertAlmostEqual(-dx*math.sin(yaw)+dy*math.cos(yaw), 0.)

    def test_shoot_headings_remain_mirrored(self):
        for number in (1, 2, 3):
            for cycle in (0, 1):
                for area in ('arc', 'inner'):
                    a = search_plan(load_config(start_point='A'), area, cycle, number, 'shoot')
                    b = search_plan(load_config(start_point='B'), area, cycle, number, 'shoot')
                    self.assertEqual(len(a), len(b))
                    for ((ax, ay), ah), ((bx, by), bh) in zip(a, b):
                        self.assertAlmostEqual(ax, bx)
                        self.assertAlmostEqual(ay, -by)
                        self.assertAlmostEqual(wrap(ah+bh), 0.)


if __name__ == '__main__':
    unittest.main()
