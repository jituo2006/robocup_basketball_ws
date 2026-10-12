from types import SimpleNamespace as NS
from rb_localization.localization_node import LocalizationNode
from rb_localization.freshness import SourceFreshness


def detector(stamp=10., bearing=0., confidence=.9):
    return NS(label='pillar', source='camera', confidence=confidence, bearing_rad=bearing,
              stamp=NS(sec=int(stamp), nanosec=int(round((stamp-int(stamp))*1e9))))


def node():
    calls = []
    freshness = SourceFreshness(); freshness.accept('odom', 10., 10., .5)
    return NS(landmarks=[(3., 0.)], source_freshness=freshness, source_timeout_s=.5,
              last_pillar_stamp=0., camera_yaw_offset=0., pillar_updates=0,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
              ekf=NS(pose=(0., 0., 0.), cfg=NS(bearing_min_dist=.5, bearing_max_dist=9.),
                     update_bearing=lambda lm, bearing: calls.append((lm, bearing)) or True),
              get_logger=lambda: NS(info=lambda *args, **kw: None)), calls


def test_duplicate_boxes_and_cached_frame_do_not_repeat_landmark_update():
    n, calls = node(); msg = NS(detections=[detector(), detector()])
    LocalizationNode.on_detections(n, msg)
    LocalizationNode.on_detections(n, msg)
    assert len(calls) == 1 and n.pillar_updates == 1


def test_old_future_low_confidence_and_nonfinite_pillars_are_ignored():
    n, calls = node()
    LocalizationNode.on_detections(n, NS(detections=[detector(9.), detector(11.),
        detector(confidence=.2), detector(bearing=float('inf'))]))
    assert calls == []


def test_ambiguous_known_landmarks_do_not_change_robot_pose():
    n, calls = node(); n.landmarks.append((3., .05))
    LocalizationNode.on_detections(n, NS(detections=[detector()]))
    assert calls == []
