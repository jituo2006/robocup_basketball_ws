from rb_localization.freshness import SourceFreshness


def test_startup_requires_data_but_silence_does_not_expire_it():
    f = SourceFreshness()
    assert not f.valid(1, .5)
    assert f.accept('odom', 1, 1, .5)
    assert f.valid(1.3, .5)
    assert f.valid(1.6, .5)
    assert not f.accept('odom', 1, 1.6, .5)  # timer or replay cannot renew it
    assert f.accept('odom', 1.7, 1.7, .5)
    assert f.valid(1.8, .5)


def test_invalid_zero_future_timestamps_are_rejected_but_delayed_is_allowed():
    f = SourceFreshness()
    for t in [0, float('nan'), float('inf'), 2]:
        assert not f.accept('odom', t, 1, .5)
    assert f.stamp is None
    assert f.accept("odom", .1, 1, .5)


def test_two_sources_cannot_roll_filter_time_backwards():
    f = SourceFreshness()
    assert f.accept('odom', 1, 1.1, .5)
    assert not f.accept('abs', .9, 1.1, .5)
    assert f.accept('abs', 1.2, 1.3, .5)
    assert not f.accept('odom', 1.1, 1.3, .5)
    assert f.stamp == 1.2


def test_sim_clock_reset_invalidates_old_state_then_accepts_new_epoch():
    f = SourceFreshness()
    assert f.accept('odom',100,100,.5)
    assert not f.valid(1,.5)
    assert f.accept('odom',1,1,.5)
