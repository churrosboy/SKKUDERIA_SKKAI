"""Offline tests for the straight-sector EKF blend (StraightBlender).

Pure Python — no rclpy — so this runs under plain pytest / colcon test.
Synthetic track: L = 27.5 m, straight sector [0, 8.0] m, 80 Hz, 5 m/s.
"""
import numpy as np
import pytest

from state_estimation.straight_blender import StraightBlender, wrap_diff

L = 27.5
DT = 1.0 / 80.0
V = 5.0


def make_blender(alpha=0.05, rate=0.5, max_off=2.0, reloc=1.0,
                 ranges=((0.0, 8.0),)):
    b = StraightBlender(L, alpha, rate, max_off, reloc)
    b.set_straight_ranges(list(ranges))
    return b


def test_drift_rejection_and_continuity():
    """Growing raw bias + a mid-straight snap: s_out tracks truth, no steps."""
    b = make_blender(alpha=0.01)
    truth = 0.5
    b.reset(truth)
    prev_out = truth
    max_err_out, max_err_raw = 0.0, 0.0
    for k in range(int(7.0 / V / DT)):          # drive the straight
        truth += V * DT
        bias = 0.3 * (truth / 7.0)              # grows to 0.3 m
        if abs(truth - 4.0) < V * DT / 2:       # snap mid-straight
            bias += 0.5
        s_raw = (truth + bias) % L
        s_out, off, mode = b.update(s_raw, V, DT)
        assert mode == StraightBlender.MODE_BLENDING
        step = wrap_diff(s_out, prev_out, L)
        # per-tick step = v*dt plus the complementary pull alpha*err; err can
        # reach ~0.6 m right after the snap -> bound at alpha*0.7 + slack
        assert abs(step - V * DT) < 0.01 * 0.7 + 1e-3, "per-tick discontinuity"
        prev_out = s_out
        max_err_out = max(max_err_out, abs(wrap_diff(s_out, truth % L, L)))
        max_err_raw = max(max_err_raw, abs(wrap_diff(s_raw, truth % L, L)))
    assert max_err_out < 0.5 * max_err_raw


def test_reconvergence_monotonic_and_rate_capped():
    b = make_blender(rate=0.5)
    b.reset(10.0)                    # outside the straight
    b.s_blend = 10.5                 # 0.5 m offset to work off
    s_raw = 10.0
    prev_off = 0.5
    ticks = 0
    while True:
        s_raw = (s_raw + V * DT) % L
        s_out, off, mode = b.update(s_raw, V, DT)
        assert wrap_diff(s_out, s_raw, L) >= -1e-9   # s_out never behind raw here
        assert off <= prev_off + 1e-9, "offset not monotone"
        assert prev_off - off <= 0.5 * DT + 1e-9, "decay faster than rate"
        prev_off = off
        ticks += 1
        if off == 0.0:
            assert mode == StraightBlender.MODE_PASSTHROUGH
            break
        assert ticks < 2000
    assert ticks * DT == pytest.approx(0.5 / 0.5, rel=0.1)   # ~1 s


def test_saturation_and_lockout_bound():
    """Wheel lock (vs=0) while raw advances: offset caps, membership exits."""
    b = make_blender(alpha=0.0, max_off=2.0)
    b.reset(1.0)
    s_raw = 1.0
    exited = False
    for _ in range(int(12.0 / V / DT)):
        s_raw = (s_raw + V * DT) % L
        s_out, off, mode = b.update(s_raw, 0.0, DT)
        assert abs(off) <= 2.0 + 1e-9
        if mode != StraightBlender.MODE_BLENDING:
            # must exit no later than max_offset past the straight end
            # (+2 ticks: membership is checked before the same tick's clamp)
            assert wrap_diff(s_raw, 8.0, L) <= 2.0 + 2 * V * DT + 1e-6
            exited = True
            break
    assert exited, "blender never exited the straight (lockout)"


def test_wraparound():
    b = make_blender(ranges=[(25.0, 27.4)])     # straight across s=0... range ends before 0
    # place a straight that wraps: start > end
    b.set_straight_ranges([(26.0, 1.5)])
    b.reset(26.2)
    prev = 26.2
    for _ in range(int(3.0 / V / DT)):          # 26.2 -> wraps -> ~1.7
        raw = (prev + V * DT + 0.02) % L        # small growing bias
        s_out, off, mode = b.update(raw, V, DT)
        step = wrap_diff(s_out, prev, L)
        assert 0 < step < 2 * V * DT
        prev = s_out
    assert not np.isnan(prev)


def test_relocalization_reset():
    b = make_blender()
    b.reset(3.0)
    b.update(3.05, V, DT)
    s_out, off, mode = b.update((3.05 + 3.0) % L, V, DT)   # 3 m jump in one tick
    assert off == 0.0 and mode == StraightBlender.MODE_PASSTHROUGH
    assert s_out == pytest.approx((3.05 + 3.0) % L)


def test_dt_gap_reset():
    b = make_blender()
    b.reset(3.0)
    s_out, off, mode = b.update(3.2, V, 1.0)    # 1 s gap
    assert off == 0.0 and s_out == pytest.approx(3.2)


def test_reverse_driving():
    b = make_blender()
    b.reset(5.0)
    prev = 5.0
    for _ in range(40):
        raw = (prev - 1.0 * DT) % L
        s_out, off, mode = b.update(raw, -1.0, DT)
        assert not np.isnan(s_out)
        assert abs(off) <= 2.0
        assert wrap_diff(s_out, prev, L) < 0    # moving backward
        prev = s_out
