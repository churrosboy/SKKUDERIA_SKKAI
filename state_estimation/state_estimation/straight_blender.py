"""Sector-gated complementary blend of frenet s.

On straight sectors cartographer's scan matcher has no along-track information
(corridor + 9 m lidar), so the pose slides along the track. While the car is in
a sector flagged is_straight (speed_scaling.yaml), the along-track coordinate is
propagated from the EKF velocity with only a small complementary pull toward the
scan-derived s; lateral d and heading stay with cartographer.

Pure Python on purpose: no rclpy import, so the offline pytest can drive it.
"""


def wrap_diff(a, b, track_length):
    """Shortest signed arc from b to a on a closed track of given length."""
    return (a - b + track_length / 2.0) % track_length - track_length / 2.0


class StraightBlender:
    # modes reported by update() for the debug topic
    MODE_PASSTHROUGH = 0
    MODE_RECONVERGING = 1
    MODE_BLENDING = 2

    def __init__(self, track_length, alpha, reconverge_rate_mps,
                 max_offset_m, reloc_jump_m):
        self.track_length = float(track_length)
        self.alpha = float(alpha)
        self.reconverge_rate_mps = float(reconverge_rate_mps)
        self.max_offset_m = float(max_offset_m)
        self.reloc_jump_m = float(reloc_jump_m)
        self.straight_ranges = []   # [(s_start_m, s_end_m)], may wrap: start > end
        self.s_blend = None
        self.last_s_raw = None

    def set_straight_ranges(self, ranges_m):
        self.straight_ranges = list(ranges_m)

    def reset(self, s_raw):
        self.s_blend = s_raw % self.track_length
        self.last_s_raw = s_raw % self.track_length

    def _in_straight(self, s):
        s = s % self.track_length
        for start, end in self.straight_ranges:
            if start <= end:
                if start <= s <= end:
                    return True
            else:  # range wraps s=0
                if s >= start or s <= end:
                    return True
        return False

    def update(self, s_raw, vs, dt):
        """One 80 Hz tick. Returns (s_out, offset, mode).

        s_raw: frenet s from the raw cartographer pose [m]
        vs:    frenet along-track velocity from the EKF twist [m/s] (signed)
        dt:    time since last update [s]
        """
        L = self.track_length
        s_raw = s_raw % L

        if self.s_blend is None:
            self.reset(s_raw)
            return s_raw, 0.0, self.MODE_PASSTHROUGH

        # relocalization guard: a raw jump this large in one tick is physically
        # impossible (/initialpose restart or TF glitch), so follow it completely.
        if abs(wrap_diff(s_raw, self.last_s_raw, L)) > self.reloc_jump_m:
            self.reset(s_raw)
            return s_raw, 0.0, self.MODE_PASSTHROUGH
        prev_s_raw = self.last_s_raw
        self.last_s_raw = s_raw

        # dt guard: TF outage / startup / sim-time reset -> do not integrate
        # blindly across the gap.
        if dt <= 0.0 or dt > 0.5:
            self.reset(s_raw)
            return s_raw, 0.0, self.MODE_PASSTHROUGH

        # membership on s_blend (the corrected state is where the car "is");
        # the saturation below bounds |s_blend - s_raw| so this can never lock in.
        if self._in_straight(self.s_blend):
            self.s_blend = (self.s_blend + vs * dt) % L
            err = wrap_diff(s_raw, self.s_blend, L)
            self.s_blend = (self.s_blend + self.alpha * err) % L
            mode = self.MODE_BLENDING
        else:
            # offset is carried against the PREVIOUS tick's raw s (the blend
            # state must ride along with the car), then decayed rate-limited.
            offset = wrap_diff(self.s_blend, prev_s_raw, L)
            step = min(abs(offset), self.reconverge_rate_mps * dt)
            offset -= step if offset > 0 else -step
            if abs(offset) < 1e-3:
                offset = 0.0
            self.s_blend = (s_raw + offset) % L
            mode = self.MODE_RECONVERGING if offset != 0.0 else self.MODE_PASSTHROUGH

        # saturate (continuous, never a snap): divergence bound + lockout bound
        offset = wrap_diff(self.s_blend, s_raw, L)
        if offset > self.max_offset_m:
            offset = self.max_offset_m
        elif offset < -self.max_offset_m:
            offset = -self.max_offset_m
        self.s_blend = (s_raw + offset) % L

        return self.s_blend, offset, mode
