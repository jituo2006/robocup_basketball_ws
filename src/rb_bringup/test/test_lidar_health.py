import importlib.util
from pathlib import Path

path = Path(__file__).parents[1] / 'scripts' / 'lidar_health.py'
spec = importlib.util.spec_from_file_location('lidar_health', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
LidarHealth = module.LidarHealth


def feed(h, start=100., duration=1.2):
    for i in range(int(duration*100)+1):
        t = start+i*.01
        h.observe('imu', t, t, t)
        if i % 10 == 0:
            h.observe('lidar', t, t, t)
    return t


def test_waits_for_continuous_imu_and_lidar():
    h=LidarHealth()
    t=feed(h, duration=.8)
    assert not h.inputs_stable(t,t)
    t=feed(h, start=t+.01, duration=1.2)
    assert h.inputs_stable(t,t)


def test_power_cycle_cannot_reuse_old_stable_period():
    h=LidarHealth();t=feed(h)
    assert h.inputs_stable(t,t)
    assert not h.inputs_live(t+20,t+20)
    t=feed(h,start=t+20,duration=.5)
    assert h.fault_serial >= 2
    assert not h.inputs_stable(t,t)
    t=feed(h,start=t+.01)
    assert h.inputs_stable(t,t)


def test_duplicate_data_never_renews_readiness():
    h=LidarHealth();t=feed(h)
    for i in range(1,81):
        now=t+i*.01
        h.observe('imu',t,now,now)
        h.observe('lidar',t,now,now)
    assert not h.inputs_live(now,now)


def test_clock_step_and_source_rollback_invalidate_session():
    h=LidarHealth();t=feed(h)
    assert not h.observe('imu',t-.1,t+.01,t+.01)
    assert h.fault_serial == 1
    assert not h.inputs_stable(t+.01,t+.01)
    assert not h.observe('lidar',t+.1,t+30,t+.1)
    assert not h.inputs_live(t+30,t+.1)


def test_odometry_jump_requests_reset_before_acceptance():
    h=LidarHealth()
    assert h.observe('odom',100,100,100,(0,0,0))
    assert not h.observe('odom',100.1,100.1,100.1,(475,0,0))
    assert h.fault_serial == 1
    assert not h.odom_stable(100.1,100.1)
    h.reset_odom()
    for i in range(12):
        t=101+i*.1
        assert h.observe('odom',t,t,t,(i*.025,0,0))
    assert h.odom_stable(t,t)


def test_lidar_without_imu_is_never_ready():
    h=LidarHealth()
    for i in range(30):
        t=100+i*.1
        h.observe('lidar',t,t,t)
    assert not h.inputs_stable(t,t)
