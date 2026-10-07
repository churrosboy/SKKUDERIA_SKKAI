"""Scan-to-map correlative matcher used by initialpose_to_cartographer.

Pure numpy — deliberately no rclpy import — so the matching maths can be unit
tested offline (see test/test_initialpose_snap.py), the same way
straight_blender.py is split out of carstate_node.py.
"""
import math

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover - scipy is present on the NUCs
    distance_transform_edt = None

# nav2 map_server: cells >= this value are occupied (occupied_thresh 0.65)
OCCUPIED_MIN = 65


def wall_surface(occupied):
    """Occupied cells that touch free space, i.e. the visible wall faces.

    On these maps everything outside the track is a solid black blob, so ~88%
    of cells are "occupied" while only ~0.7% are actual wall surface. Measuring
    distance to any occupied cell would give a beam that overshoots a wall a
    perfect score (it lands deep in the blob, distance 0), which makes the
    score almost useless for telling a right pose from a wrong one. Measuring
    distance to the surface penalizes overshoot the same way it penalizes
    undershoot. 4-neighbour dilation, pure numpy (no scipy needed here)."""
    free = ~occupied
    touching = np.zeros_like(free)
    touching[:-1, :] |= free[1:, :]
    touching[1:, :] |= free[:-1, :]
    touching[:, :-1] |= free[:, 1:]
    touching[:, 1:] |= free[:, :-1]
    return occupied & touching


class LikelihoodField:
    """Occupancy grid -> per-cell likelihood exp(-d^2 / 2 sigma^2), where d
    is the distance (m) to the nearest wall SURFACE cell (see wall_surface).
    Zero-padded by `pad` cells on every side so shifted index lookups never go
    out of bounds. Pure numpy; no ROS dependency (unit-testable offline)."""

    def __init__(self, grid, resolution, origin_xy, sigma, pad):
        # grid: int array (H, W), row = y cell, col = x cell (ROS layout)
        occupied = grid >= OCCUPIED_MIN
        surface = wall_surface(occupied)
        if distance_transform_edt is None or not surface.any():
            dist_cells = np.where(surface, 0.0, np.inf)
        else:
            dist_cells = distance_transform_edt(~surface)
        dist_m = dist_cells * resolution
        field = np.exp(-0.5 * (dist_m / sigma) ** 2).astype(np.float32)
        self.pad = int(pad)
        self.field = np.pad(field, self.pad, mode='constant',
                            constant_values=0.0)
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = grid.shape

    def cells(self, xy, margin=0):
        """World (N,2) -> padded integer cell indices (col, row), clipped so
        that index +- margin stays inside the padded array. Anything outside
        the grid lands in the zero padding border (needs margin < pad)."""
        cx = np.floor((xy[:, 0] - self.ox) / self.res).astype(np.int64)
        cy = np.floor((xy[:, 1] - self.oy) / self.res).astype(np.int64)
        cx = np.clip(cx + self.pad, margin, self.w + 2 * self.pad - 1 - margin)
        cy = np.clip(cy + self.pad, margin, self.h + 2 * self.pad - 1 - margin)
        return cx, cy

    def interp(self, xy):
        """Bilinear likelihood at world points (..., 2) (sub-cell)."""
        u = (xy[..., 0] - self.ox) / self.res - 0.5 + self.pad
        v = (xy[..., 1] - self.oy) / self.res - 0.5 + self.pad
        u = np.clip(u, 0.0, self.w + 2 * self.pad - 1.001)
        v = np.clip(v, 0.0, self.h + 2 * self.pad - 1.001)
        u0 = np.floor(u).astype(np.int64)
        v0 = np.floor(v).astype(np.int64)
        fu = u - u0
        fv = v - v0
        f = self.field
        return ((1 - fu) * (1 - fv) * f[v0, u0] + fu * (1 - fv) * f[v0, u0 + 1]
                + (1 - fu) * fv * f[v0 + 1, u0] + fu * fv * f[v0 + 1, u0 + 1])


def correlative_match(lf, pts_base, seed, search_xy, search_yaw,
                      yaw_step, fine_yaw_step=None):
    """Brute-force search for the base_link pose maximizing the mean
    likelihood of the scan endpoints.

    lf: LikelihoodField
    pts_base: (N,2) scan endpoints in base_link
    seed: (x, y, yaw) initial guess
    search_xy: +- meters around seed (x and y); translation step is one map
               cell so shifted candidates are exact integer index offsets
    search_yaw: +- rad around seed
    yaw_step: rad between coarse yaw candidates
    fine_yaw_step: optional finer yaw pass around the coarse winner
    returns (x, y, yaw, score, seed_score); score in [0, 1]
    """
    n_xy = max(0, int(round(search_xy / lf.res)))
    n_xy = min(n_xy, lf.pad - 1)
    offs = np.arange(-n_xy, n_xy + 1)
    ii = offs[None, None, :]   # x (col) shift
    jj = offs[None, :, None]   # y (row) shift
    sx, sy, syaw = seed

    def score_yaw_block(yaws):
        best = (-1.0, 0, 0, 0.0)
        for yaw in yaws:
            c, s = math.cos(yaw), math.sin(yaw)
            xy = np.empty_like(pts_base)
            xy[:, 0] = c * pts_base[:, 0] - s * pts_base[:, 1] + sx
            xy[:, 1] = s * pts_base[:, 0] + c * pts_base[:, 1] + sy
            cx, cy = lf.cells(xy, margin=n_xy)
            win = lf.field[cy[:, None, None] + jj, cx[:, None, None] + ii]
            sc = win.sum(axis=0) / float(len(pts_base))   # (ny, nx)
            j, i = np.unravel_index(np.argmax(sc), sc.shape)
            if sc[j, i] > best[0]:
                best = (float(sc[j, i]), int(offs[i]), int(offs[j]), yaw)
        return best

    yaws = syaw + np.arange(-search_yaw, search_yaw + 1e-9, yaw_step)
    best_score, di, dj, best_yaw = score_yaw_block(yaws)
    bx, by = sx + di * lf.res, sy + dj * lf.res

    # Fine stage: sub-cell translation (bilinear field) +- one cell around
    # the coarse winner and a finer yaw sweep +- one coarse step.
    if fine_yaw_step and fine_yaw_step < yaw_step:
        fine_step = lf.res / 5.0
        f_offs = np.arange(-lf.res, lf.res + 1e-9, fine_step)
        fx, fy = np.meshgrid(f_offs, f_offs)
        f_shift = np.stack([fx.ravel(), fy.ravel()], 1)   # (K,2)
        # re-baseline: the fine sweep includes the coarse winner itself and can
        # only improve on it
        best_score = -1.0
        fine_best = (bx, by, best_yaw)
        for yaw in best_yaw + np.arange(-yaw_step, yaw_step + 1e-9,
                                        fine_yaw_step):
            c, s = math.cos(yaw), math.sin(yaw)
            xy = np.empty_like(pts_base)
            xy[:, 0] = c * pts_base[:, 0] - s * pts_base[:, 1] + bx
            xy[:, 1] = s * pts_base[:, 0] + c * pts_base[:, 1] + by
            cand = xy[:, None, :] + f_shift[None, :, :]   # (N,K,2)
            sc = lf.interp(cand).mean(axis=0)             # (K,)
            k = int(np.argmax(sc))
            if sc[k] > best_score:
                best_score = float(sc[k])
                fine_best = (bx + f_shift[k, 0], by + f_shift[k, 1], yaw)
        bx, by, best_yaw = fine_best

    # seed score for the log (yaw = seed, zero shift)
    c, s = math.cos(syaw), math.sin(syaw)
    xy = np.empty_like(pts_base)
    xy[:, 0] = c * pts_base[:, 0] - s * pts_base[:, 1] + sx
    xy[:, 1] = s * pts_base[:, 0] + c * pts_base[:, 1] + sy
    seed_score = float(lf.interp(xy).mean())

    return (bx, by, best_yaw, best_score, seed_score)


def nearest_waypoint(wpnts, x, y):
    """Index of the waypoint closest to (x, y). wpnts: (N,>=2) [x, y, ...]."""
    return int(np.argmin(np.hypot(wpnts[:, 0] - x, wpnts[:, 1] - y)))


def signed_lateral(wpnts, i, x, y):
    """Lateral offset of (x, y) from waypoint i, positive to the LEFT of its
    heading psi (same sign convention as Wpnt.d_left / d_right)."""
    psi = wpnts[i, 2]
    dx, dy = x - wpnts[i, 0], y - wpnts[i, 1]
    return -math.sin(psi) * dx + math.cos(psi) * dy


def raceline_search(lf, pts_base, wpnts, s_center, s_window, d_offsets,
                    yaw_offsets, allow_reverse=True, max_beams=120,
                    chunk=256, far_dist=1.0, prior_weight=0.02,
                    log_eps=0.02):
    """Global-ish relocalization along the raceline, for the
    hand-guided collision recovery: the car is pushed a few metres
    back onto the track, so the true pose lies within +-s_window of the
    crash prior along the track but at unknown lateral offset and heading
    (the car may even be set down backwards).

    Candidates = waypoints with wrapped |s - s_center| <= s_window, shifted
    by each d in d_offsets along the left normal, heading psi + each yaw in
    yaw_offsets (and +pi copies when allow_reverse). Every candidate is
    scored by the mean likelihood of <= max_beams scan endpoints
    (vectorized in chunks; ~6k candidates x 120 beams takes tens of ms).

    lf: LikelihoodField
    pts_base: (N,2) scan endpoints in base_link
    wpnts: (M,4) [x, y, psi, s] in ascending s (closed loop)
    Score = GEOMETRIC mean of the beam likelihoods (mean log(l + log_eps)),
    not the arithmetic mean correlative_match uses: along a straight the
    many side-wall beams score the same at every candidate and swamp the few
    end-wall beams that actually fix the along-track position; the log makes
    a handful of badly mismatched beams expensive. log_eps bounds the cost
    of genuine outliers (the person who just pushed the car is in the scan).
    prior_weight: score penalty per metre of wrapped |s - s_center|. On a
    featureless straight a pose 1 m along the corridor scores only ~0.1
    below the truth, so the crash location is the natural tiebreaker: the
    user puts the car back near where it crashed. 0.02/m = 0.08 at 4 m.
    returns (x, y, yaw, score, second_score, second_dist) where scores are
    prior-penalized and second_* is the best candidate farther than far_dist
    from the winner (None if there is none) -- for the ambiguity gate on
    look-alike straights.
    """
    if len(pts_base) > max_beams:
        idx = np.linspace(0, len(pts_base) - 1, max_beams).astype(int)
        pts_base = pts_base[idx]
    s = wpnts[:, 3]
    track_len = float(s[-1] - s[0]) + (float(s[1] - s[0]) if len(s) > 1 else 0.0)
    ds = np.abs(s - s_center)
    if track_len > 0:
        ds = np.minimum(ds, track_len - ds)
    sel = ds <= s_window
    if not sel.any():
        sel[int(np.argmin(ds))] = True  # degenerate window: nearest only
    base = wpnts[sel]
    base_ds = ds[sel]

    d_offsets = np.asarray(d_offsets, dtype=np.float64)
    yaw_offsets = np.asarray(yaw_offsets, dtype=np.float64)
    if allow_reverse:
        yaw_offsets = np.concatenate([yaw_offsets, yaw_offsets + math.pi])
    psi = base[:, 2]
    # (B, D) lateral shifts along the left normal (-sin psi, cos psi)
    cx = base[:, 0][:, None] - d_offsets[None, :] * np.sin(psi)[:, None]
    cy = base[:, 1][:, None] + d_offsets[None, :] * np.cos(psi)[:, None]
    cx = np.repeat(cx.ravel(), len(yaw_offsets))
    cy = np.repeat(cy.ravel(), len(yaw_offsets))
    cyaw = (np.repeat(psi, len(d_offsets))[:, None] + yaw_offsets[None, :]).ravel()
    cds = np.repeat(np.repeat(base_ds, len(d_offsets)), len(yaw_offsets))

    scores = np.empty(len(cx), dtype=np.float64)
    for k0 in range(0, len(cx), chunk):
        k1 = min(k0 + chunk, len(cx))
        c = np.cos(cyaw[k0:k1])[:, None]
        sn = np.sin(cyaw[k0:k1])[:, None]
        xy = np.empty((k1 - k0, len(pts_base), 2))
        xy[:, :, 0] = c * pts_base[None, :, 0] - sn * pts_base[None, :, 1] + cx[k0:k1, None]
        xy[:, :, 1] = sn * pts_base[None, :, 0] + c * pts_base[None, :, 1] + cy[k0:k1, None]
        scores[k0:k1] = np.log(lf.interp(xy) + log_eps).mean(axis=1)
    scores = np.exp(scores) - prior_weight * cds

    b = int(np.argmax(scores))
    dist = np.hypot(cx - cx[b], cy - cy[b])
    far = dist > far_dist
    if far.any():
        f = np.flatnonzero(far)
        j = f[int(np.argmax(scores[f]))]
        second_score, second_dist = float(scores[j]), float(dist[j])
    else:
        second_score, second_dist = None, None
    yaw = math.atan2(math.sin(cyaw[b]), math.cos(cyaw[b]))
    return (float(cx[b]), float(cy[b]), yaw, float(scores[b]),
            second_score, second_dist)
