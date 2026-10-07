"""Offline tests for the scan-to-map matcher behind the RViz snap
(initialpose_to_cartographer).

Pure Python — no rclpy — so this runs under plain pytest / colcon test.
Synthetic map: a rectangular ring "track" drawn on an all-occupied canvas,
which is how the real saved maps look (~88% of cells are the solid black
region outside the track, only ~0.7% are actual wall surface).
"""
import math

import numpy as np

from state_estimation.scan_matcher import (
    LikelihoodField, correlative_match, wall_surface, OCCUPIED_MIN)

RES = 0.05
ORIGIN = (0.0, 0.0)
SIGMA = 0.1

# snap params, mirroring initialpose_to_cartographer's defaults
SNAP_XY = 0.5
SNAP_YAW = math.radians(15.0)
SNAP_COARSE = math.radians(1.0)
SNAP_FINE = math.radians(0.2)
# a deliberately wider window, to check the pad sizing rule
WIDE_XY = 0.75


def make_track():
    """Ring corridor: free between an outer and an inner rectangle, solid
    everywhere else. Returns (grid, occupied)."""
    n = 300
    grid = np.full((n, n), 100, dtype=np.int16)

    def rect(x0, y0, x1, y1):
        return (slice(int(y0 / RES), int(y1 / RES)),
                slice(int(x0 / RES), int(x1 / RES)))

    grid[rect(2.0, 2.0, 13.0, 9.0)] = 0
    grid[rect(4.0, 3.6, 11.0, 6.6)] = 100
    grid[rect(2.6, 2.0, 3.4, 2.5)] = 100
    return grid, grid >= OCCUPIED_MIN


GRID, OCCUPIED = make_track()


def raycast(x, y, yaw, n_beams=360, rmax=9.0, step=0.02, noise=0.0, seed=0):
    """Synthetic 2D LiDAR endpoints in the sensor frame (N, 2)."""
    rng = np.random.default_rng(seed)
    angs = yaw + np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    ts = np.arange(step, rmax, step)
    cx = np.clip(((x + np.cos(angs)[:, None] * ts[None, :]) / RES).astype(int),
                 0, GRID.shape[1] - 1)
    cy = np.clip(((y + np.sin(angs)[:, None] * ts[None, :]) / RES).astype(int),
                 0, GRID.shape[0] - 1)
    hit = OCCUPIED[cy, cx]
    r = ts[np.where(hit.any(1), hit.argmax(1), len(ts) - 1)]
    if noise:
        r = r + rng.normal(0.0, noise, r.shape)
    ok = hit.any(1)
    rel = angs - yaw
    return np.stack([r[ok] * np.cos(rel[ok]), r[ok] * np.sin(rel[ok])], 1)


def make_field(search_xy):
    return LikelihoodField(GRID, RES, ORIGIN, SIGMA,
                           int(round(search_xy / RES)) + 2)


def test_wall_surface_excludes_the_solid_interior():
    """The bug this guards: on these maps ~everything outside the track is
    'occupied', so distance-to-nearest-occupied is ~0 almost everywhere and a
    beam that overshoots a wall scores a perfect 1.0. Only the thin wall face
    may count as a match target."""
    surface = wall_surface(OCCUPIED)
    # a surface cell is always an occupied cell
    assert not (surface & ~OCCUPIED).any()
    # the solid region dominates the map (75% here, ~88% on the real maps),
    # and the wall face is a thin skin of it (1.7% here, ~0.7% on the real maps)
    assert OCCUPIED.mean() > 0.7
    assert surface.sum() < 0.05 * OCCUPIED.sum()
    # a cell deep inside the solid interior is occupied but not a wall face
    deep = (int(5.0 / RES), int(7.5 / RES))       # (row, col) in the infield
    assert OCCUPIED[deep]
    assert not surface[deep]
    # a cell on the infield's left face is
    face = (int(5.0 / RES), int(4.0 / RES))
    assert OCCUPIED[face] and surface[face]


def test_deep_interior_scores_far_lower_than_a_wall_face():
    """Same bug, expressed as a score: the field must fall off away from the
    wall face, including *into* the solid region."""
    lf = make_field(SNAP_XY)

    def at(x, y):
        return float(lf.interp(np.array([[x, y]]))[0])

    # on the infield's left face
    assert at(4.0, 5.0) > 0.9
    # 1 m deep inside the same solid block: must be ~zero, not 1.0
    assert at(5.0, 5.0) < 0.01
    # and 1 m out into the free corridor, likewise
    assert at(3.0, 5.0) < 0.01


def test_snap_refines_to_within_a_cell():
    """Seeded near the truth, the fine pass must land on it."""
    tx, ty, tyaw = 3.0, 5.0, 0.4
    pts = raycast(tx, ty, tyaw)
    seed = (tx + 0.22, ty - 0.17, tyaw + math.radians(6.0))
    x, y, yaw, score, seed_score = correlative_match(
        make_field(SNAP_XY), pts, seed, SNAP_XY, SNAP_YAW,
        SNAP_COARSE, SNAP_FINE)
    assert math.hypot(x - tx, y - ty) <= RES
    assert abs(yaw - tyaw) <= 2 * SNAP_COARSE
    assert score > seed_score


def test_a_wrong_pose_scores_below_the_reject_gate():
    """The snap_min_score gate must be reachable: a clearly wrong pose inside
    the search window must score below the gate, otherwise the 'using raw
    click' branch is dead code."""
    lf = make_field(SNAP_XY)
    pts = raycast(3.0, 5.0, 0.4)
    # a pose 2 m away and 40 deg off, i.e. outside any plausible click error
    _, _, _, _, wrong_score = correlative_match(
        lf, pts, (5.0, 6.4, 0.4 + math.radians(40.0)), 0.0, 0.0,
        SNAP_COARSE, None)
    assert wrong_score < 0.35


def test_pad_must_cover_the_search_window():
    """correlative_match clamps n_xy to pad-1, so a field built for the narrow
    snap window would silently shrink a wider search."""
    narrow = make_field(SNAP_XY)
    wide = make_field(WIDE_XY)
    assert narrow.pad - 1 < WIDE_XY / RES     # narrow field would clamp
    assert wide.pad - 1 >= WIDE_XY / RES      # correctly sized field does not


# ----------------------------------------------------------------------
# raceline_search (hand-guided collision recovery)
# ----------------------------------------------------------------------
from state_estimation.scan_matcher import (  # noqa: E402
    raceline_search, nearest_waypoint, signed_lateral)


def make_waypoints(spacing=0.1):
    """Closed loop along the corridor centreline of make_track():
    (3,2.8) -> (12,2.8) -> (12,7.8) -> (3,7.8) -> back. (M,4) [x,y,psi,s]."""
    corners = [(3.0, 2.8), (12.0, 2.8), (12.0, 7.8), (3.0, 7.8)]
    pts = []
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        n = int(round(math.hypot(x1 - x0, y1 - y0) / spacing))
        psi = math.atan2(y1 - y0, x1 - x0)
        for k in range(n):
            f = k / n
            pts.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), psi))
    w = np.array(pts)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(w[:, 0]), np.diff(w[:, 1])))])
    return np.column_stack([w, s])


WPNTS = make_waypoints()
D_OFFS = [-0.6, -0.3, 0.0, 0.3, 0.6]
YAW_OFFS = np.radians(np.arange(-45.0, 45.0 + 1e-9, 5.0))


def _search(truth, prior_xy, allow_reverse=True):
    lf = make_field(SNAP_XY)
    pts = raycast(*truth, n_beams=360)
    i = nearest_waypoint(WPNTS[:, :2], *prior_xy)
    res = raceline_search(lf, pts, WPNTS, WPNTS[i, 3], 4.0, D_OFFS, YAW_OFFS,
                          allow_reverse=allow_reverse)
    x, y, yaw, score, second, sdist = res
    # fine snap around the coarse winner, as the node does
    fx, fy, fyaw, fscore, _ = correlative_match(
        lf, pts, (x, y, yaw), SNAP_XY, SNAP_YAW, SNAP_COARSE, SNAP_FINE)
    return (fx, fy, fyaw, fscore), res


def test_raceline_search_finds_pushed_pose():
    # crashed at (5.0, 2.8) heading +x; user pushed it 2.5 m along, 0.2 m
    # left and set it down 20 deg crooked
    truth = (7.5, 3.0, math.radians(20.0))
    (fx, fy, fyaw, fscore), (x, y, yaw, score, second, sdist) = _search(
        truth, (5.0, 2.8))
    assert math.hypot(x - truth[0], y - truth[1]) <= 0.35      # coarse grid
    assert abs(yaw - truth[2]) <= math.radians(5.0) + 1e-6
    assert math.hypot(fx - truth[0], fy - truth[1]) <= RES
    assert abs(fyaw - truth[2]) <= 2 * SNAP_COARSE
    assert fscore > 0.8


def test_raceline_search_reversed_heading():
    # set down facing backwards along the bottom straight
    truth = (6.0, 2.6, math.radians(180.0 - 10.0))
    (fx, fy, fyaw, _), (x, y, yaw, score, second, sdist) = _search(
        truth, (5.0, 2.8))
    assert math.hypot(fx - truth[0], fy - truth[1]) <= RES
    d = math.atan2(math.sin(fyaw - truth[2]), math.cos(fyaw - truth[2]))
    assert abs(d) <= 2 * SNAP_COARSE


def test_raceline_search_reports_a_runner_up_far_away():
    truth = (7.5, 3.0, 0.0)
    _, (x, y, yaw, score, second, sdist) = _search(truth, (7.0, 2.8))
    assert second is not None and sdist > 1.0
    assert score > second            # a 1 m-off pose must not tie the truth
    assert score - second > 0.05     # the node's reloc_min_margin default


def test_signed_lateral_matches_left_positive():
    i = nearest_waypoint(WPNTS[:, :2], 7.0, 2.8)   # bottom straight, psi=0
    assert signed_lateral(WPNTS, i, 7.0, 3.1) > 0   # +y is LEFT of +x
    assert signed_lateral(WPNTS, i, 7.0, 2.5) < 0
