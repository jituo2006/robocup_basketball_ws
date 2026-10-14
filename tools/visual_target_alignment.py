"""出球点用前摄定位柱修正朝向；不把可移动篮架当固定定位地标。"""
import math
import time


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def pillar_candidate(runner, cfg, reference_yaw):
    candidates = []
    expected = wrap(runner.pose[2] + cfg['camera_yaw_offset_rad'] - reference_yaw)
    gate = math.radians(cfg['max_turn_deg'])
    for det in runner.dets:
        if det.label != 'pillar' or getattr(det, 'source', 'camera') != 'camera':
            continue
        values = [float(getattr(det, key, float('nan'))) for key in
                  ('confidence', 'bearing_rad', 'x_min', 'y_min', 'x_max', 'y_max')]
        confidence, bearing, x1, y1, x2, y2 = values
        stamp = det.stamp.sec + det.stamp.nanosec * 1e-9
        if (not all(math.isfinite(v) for v in values)
                or not cfg['confidence'] <= confidence <= 1
                or not 0 <= x1 < x2 <= 1 or not 0 <= y1 < y2 <= 1
                or not runner.detection_fresh(stamp)
                or abs(wrap(bearing - expected)) > gate):
            continue
        candidates.append(det)
    # 同一视野里的多个柱无法仅凭类别判定目标身份，不能取最高置信度猜一个。
    return candidates[0] if len(candidates) == 1 else None


def align_pillar(runner, cfg, reference_yaw, emit=print):
    origin_yaw = runner.pose[2]
    deadline = time.monotonic() + cfg['timeout_s']
    last_stamp, stable = None, 0
    tolerance = math.radians(cfg['tolerance_deg'])
    launcher_offset = math.radians(cfg['launcher_yaw_offset_deg'])
    reason = '未看到唯一的新鲜定位柱'
    try:
        while time.monotonic() < deadline:
            runner.spin(.03)
            if abs(wrap(runner.pose[2] - origin_yaw)) > math.radians(cfg['max_turn_deg']):
                reason = '定位柱对准旋转已到配置范围'
                break
            det = pillar_candidate(runner, cfg, reference_yaw)
            if det is None:
                runner.stop()
                stable, last_stamp = 0, None
                continue
            stamp = (det.stamp.sec, det.stamp.nanosec)
            error = wrap(cfg['camera_yaw_offset_rad'] - det.bearing_rad - launcher_offset)
            if abs(error) <= tolerance:
                runner.stop()
                if stamp != last_stamp:
                    stable += 1
                last_stamp = stamp
                if stable >= cfg['stable_frames']:
                    emit('前摄定位柱对准完成')
                    return True
            else:
                stable, last_stamp = 0, stamp
                tw = runner._Twist()
                tw.angular.z = max(-cfg['max_ang_rad_s'], min(cfg['max_ang_rad_s'], cfg['kp'] * error))
                runner.publish_command(tw)
        emit(reason + '，保留雷达到位后的朝向')
        return False
    finally:
        runner.stop()
