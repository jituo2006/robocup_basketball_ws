"""Real DDS service acknowledgements, isolated from robot domain and hardware."""
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rb_msgs.srv import Launch
from rb_mission.mission_node import MissionNode


@pytest.mark.parametrize("accepted", [False, True])
def test_real_launch_response_controls_counting(accepted):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=190)
    executor = SingleThreadedExecutor(context=context)
    mission = server = None
    try:
        mission = MissionNode(context=context)
        server = Node("fake_launcher_rule_test", context=context)
        actions = []

        def respond(request, response):
            actions.append(request.action)
            response.success = accepted
            response.message = "test response, no hardware"
            return response

        server.create_service(Launch, "/rb_launcher/launch", respond)
        executor.add_node(mission)
        executor.add_node(server)
        assert mission.launch_client.wait_for_service(timeout_sec=5)
        # This test isolates the launch protocol; navigation has separate tests.
        mission._localization_ready = lambda: True
        mission.require_zone = False
        mission.cfg["launcher"]["automatic"]["enabled"] = False  # isolates service handshake
        mission.mission = "SHOOT"
        mission.launch_hold_s = 0.05
        mission.set_phase("LAUNCH")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and mission.phase == "LAUNCH":
            executor.spin_once(timeout_sec=0.05)
        assert actions[0] == 0
        if accepted:
            assert mission.actions_done == 1 and mission.phase == "SEEK_BALL"
            assert 7 in actions  # STOP handshake must complete before counting.
        else:
            assert mission.actions_done == 0 and mission.phase == "FAULT"
        assert mission._score_estimate() == -1
    finally:
        if mission is not None:
            mission.publish_zero()
            mission.destroy_node()
        if server is not None:
            server.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('failure', [None, 'stale', 'map_disagreement', 'unconfirmed', 'no_table', 'unaligned'])
def test_automatic_shot_uses_fused_range_or_refuses(failure):
    from rb_msgs.msg import Detection, DetectionArray
    context = Context()
    rclpy.init(args=[], context=context, domain_id=192)
    executor = SingleThreadedExecutor(context=context)
    mission = server = None
    try:
        mission = MissionNode(context=context)
        mission.timer.cancel()
        server = Node('fake_launcher_auto_test', context=context)
        requests = []
        def respond(request, response):
            requests.append((request.action, request.speed, request.angle))
            response.success = True
            return response
        server.create_service(Launch, '/rb_launcher/launch', respond)
        executor.add_node(mission); executor.add_node(server)
        assert mission.launch_client.wait_for_service(timeout_sec=5)
        mission._localization_ready = lambda: True
        mission.mission = 'SHOOT'
        mission.last_pose = (0, 0, 0)
        mission.field['hoop'] = dict(x=3, y=0)
        cfg = mission.cfg['launcher']['automatic']
        cfg.update(enabled=True, map_confirmed=True, detection_timeout_s=1,
                   table=[dict(distance_m=2,speed=2000,angle=50), dict(distance_m=4,speed=4000,angle=70)])
        array = DetectionArray()
        array.header.stamp = mission.get_clock().now().to_msg()
        det = Detection(label='hoop',confidence=.9,bearing_rad=0.,distance_m=3.2)
        det.stamp = array.header.stamp
        if failure == 'stale': det.stamp.sec -= 10
        if failure == 'map_disagreement': det.distance_m = 7.
        if failure == 'unaligned': det.bearing_rad = .5
        if failure == 'unconfirmed': cfg['map_confirmed'] = False
        if failure == 'no_table': cfg['table'] = []
        array.detections = [det]
        mission.on_detections(array)
        future = mission._call_launcher()
        if failure:
            assert future is None
            for _ in range(3): executor.spin_once(timeout_sec=.02)
            assert requests == []
        else:
            assert future is not None
            executor.spin_until_future_complete(future, timeout_sec=3)
            assert future.done() and future.result().success
            assert requests == [(0, 3060, 61)]
    finally:
        if mission is not None: mission.destroy_node()
        if server is not None: server.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
