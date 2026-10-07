#!/usr/bin/env python3
"""Drive the ORIGINAL python spliner (spline_planner/spline_planner.py) through a
synthetic scenario and record its outputs as golden files for test_spliner_core.

Usage (workspace sourced):  python3 test/gen_spliner_golden.py
"""
import copy
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

import rclpy  # noqa: E402
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

N_FRAMES = 300
DT = 0.05


def build_track():
    straight, radius, spacing = 12.0, 2.5, 0.1
    length = 2 * straight + 2 * math.pi * radius
    n = int(length / spacing)
    wpnts = []
    for i in range(n):
        s = i * spacing
        if s < straight:
            x, y, psi, kappa = s, 0.0, 0.0, 0.0
        elif s < straight + math.pi * radius:
            a = (s - straight) / radius
            x, y, psi, kappa = straight + radius * math.sin(a), radius - radius * math.cos(a), a, 1 / radius
        elif s < 2 * straight + math.pi * radius:
            u = s - straight - math.pi * radius
            x, y, psi, kappa = straight - u, 2 * radius, math.pi, 0.0
        else:
            a = (s - 2 * straight - math.pi * radius) / radius
            x, y, psi, kappa = -radius * math.sin(a), radius + radius * math.cos(a), math.pi + a, 1 / radius
        psi = math.atan2(math.sin(psi), math.cos(psi))
        vx = 3.0 + 1.0 * math.sin(2 * math.pi * s / length)
        d_left = 1.2 + 0.2 * math.sin(s / 3.0)
        d_right = 1.2 - 0.2 * math.sin(s / 5.0)
        wpnts.append(dict(id=i, s=s, x=x, y=y, psi=psi, kappa=kappa, d_left=d_left,
                          d_right=d_right, vx=vx, vx_scaled=vx * 0.85))
    return wpnts


def mk_obs(oid, s_c, d_c, size, L, vs=0.0, vd=0.0, static=True, visible=True):
    o = Obstacle()
    o.id = oid
    o.s_center = s_c % L
    o.s_start = (s_c - size / 2) % L
    o.s_end = (s_c + size / 2) % L
    o.d_center = d_c
    o.d_left = d_c + size / 2
    o.d_right = d_c - size / 2
    o.size = size
    o.vs = vs
    o.vd = vd
    o.is_static = static
    o.is_visible = visible
    return o


def build_scenario(L):
    rng = np.random.default_rng(20260822)
    frames = []
    for k in range(N_FRAMES):
        t = k * DT
        car_s = (3.0 * t) % L
        car_d = 0.4 if 120 <= k < 140 else 0.02 * math.sin(t)
        car_vs = 3.0 + 0.5 * math.sin(0.3 * t)
        obs = []
        if not (45 <= k % 50 < 50):
            obs.append(mk_obs(1, 10.0 + rng.normal(0, 0.01), 0.3, 0.3, L))
            obs.append(mk_obs(2, 25.0, -0.2, 0.3, L))
            obs.append(mk_obs(3, 26.8, 0.1, 0.3, L))
            obs.append(mk_obs(4, 6.0 + 2.8 * t, 0.1 * math.sin(0.5 * t), 0.35, L,
                              vs=2.8, vd=0.05 * math.cos(0.5 * t), static=False))
            obs.append(mk_obs(5, 30.5, 0.5, 0.3, L))
            obs.append(mk_obs(6, 18.0, 0.65, 0.4, L))
            if k % 9 == 0:
                obs.append(mk_obs(7, car_s + 3.0, 1.0, 0.2, L))  # off-raceline, filtered
            rng.shuffle(obs)
        frames.append(dict(k=k, car=(car_s, car_d, car_vs), obs=obs))
    return frames


def fmt(v):
    return repr(float(v))


def main():
    wpnts = build_track()
    L = wpnts[-1]['s']
    frames = build_scenario(L)
    golden = os.path.join(HERE, 'golden')
    os.makedirs(golden, exist_ok=True)
    with open(os.path.join(golden, 'spliner_scenario.txt'), 'w') as f:
        f.write(f"TRACK {len(wpnts)}\n")
        for w in wpnts:
            f.write(f"{w['id']} " + " ".join(fmt(w[k]) for k in
                    ('s', 'x', 'y', 'psi', 'kappa', 'd_left', 'd_right', 'vx', 'vx_scaled')) + "\n")
        for fr in frames:
            f.write(f"FRAME {fr['k']}\n")
            f.write("CAR " + " ".join(fmt(v) for v in fr['car']) + "\n")
            f.write(f"OBS {len(fr['obs'])}\n")
            for o in fr['obs']:
                f.write(f"{o.id} " + " ".join(fmt(v) for v in (
                    o.s_start, o.s_end, o.s_center, o.d_left, o.d_right, o.d_center, o.size,
                    o.vs, o.vd)) + f" {int(o.is_static)} {int(o.is_visible)}\n")
        f.write("END\n")

    path, scaled = WpntArray(), WpntArray()
    for w in wpnts:
        a = Wpnt()
        a.id, a.s_m, a.x_m, a.y_m, a.psi_rad, a.kappa_radpm = w['id'], w['s'], w['x'], w['y'], w['psi'], w['kappa']
        a.d_left, a.d_right, a.vx_mps = w['d_left'], w['d_right'], w['vx']
        b = copy.deepcopy(a)
        b.vx_mps = w['vx_scaled']
        path.wpnts.append(a)
        scaled.wpnts.append(b)

    rclpy.init()
    from spline_planner.spline_planner import ObstacleSpliner

    def fake_wait(self):
        od = Odometry()
        od.pose.pose.position.x = frames[0]['car'][0]
        od.pose.pose.position.y = frames[0]['car'][1]
        od.twist.twist.linear.x = frames[0]['car'][2]
        self.state_cb(od)
        self.gb_cb(path)
        self.gb_scaled_cb(scaled)
    ObstacleSpliner.wait_for_messages = fake_wait
    node = ObstacleSpliner()

    class Rec:
        def __init__(self):
            self.msgs = []

        def publish(self, m):
            self.msgs.append(copy.deepcopy(m))

        def get_subscription_count(self):
            return 0
    rec = Rec()
    node.evasion_pub = rec
    node.mrks_pub = Rec()
    node.closest_obs_pub = Rec()
    node.pub_propagated = Rec()

    with open(os.path.join(golden, 'spliner_expected.txt'), 'w') as out:
        for fr in frames:
            od = Odometry()
            od.pose.pose.position.x, od.pose.pose.position.y = fr['car'][0], fr['car'][1]
            od.twist.twist.linear.x = fr['car'][2]
            node.state_cb(od)
            oa = ObstacleArray()
            oa.obstacles = list(fr['obs'])
            node.obs_cb(oa)
            rec.msgs.clear()
            node.spliner_loop()
            m = rec.msgs[-1]
            out.write(f"FRAME {fr['k']}\n")
            out.write(f"OT {m.header.frame_id or '-'} {m.ot_side or '-'} {m.ot_line or '-'} "
                      f"{int(m.side_switch)} {len(m.wpnts)}\n")
            for w in m.wpnts:
                out.write(f"{w.id} {fmt(w.s_m)} {fmt(w.d_m)} {fmt(w.vx_mps)} {fmt(w.x_m)} {fmt(w.y_m)}\n")
            out.write(f"LAST {node.last_ot_side or '-'}\n")
    node.destroy_node()
    rclpy.shutdown()
    print(f"wrote {N_FRAMES} frames to {golden}")


if __name__ == '__main__':
    main()
