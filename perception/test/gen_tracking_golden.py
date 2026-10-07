#!/usr/bin/env python3
"""Drive the ORIGINAL python tracking node (perception/tracking.py) through a
synthetic scenario and record its outputs as golden files for test_tracking_core.

Usage (workspace sourced):
    python3 src/perception/test/gen_tracking_golden.py
Writes test/golden/tracking_scenario.txt and test/golden/tracking_expected.txt.
"""
import copy
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)  # import the src copy of perception.tracking, not the installed one

import rclpy  # noqa: E402
from tf_transformations import quaternion_from_euler  # noqa: E402
from f110_msgs.msg import Wpnt, WpntArray, Obstacle, ObstacleArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

RATE = 20
N_FRAMES = 420
DT = 1.0 / RATE


def build_track():
    """Oval: two 12 m straights + two semicircles (r = 2.5 m), 0.1 m spacing."""
    straight, radius, spacing = 12.0, 2.5, 0.1
    length = 2 * straight + 2 * math.pi * radius
    n = int(length / spacing)
    wpnts = []
    for i in range(n):
        s = i * spacing
        if s < straight:
            x, y, psi = s, 0.0, 0.0
        elif s < straight + math.pi * radius:
            a = (s - straight) / radius
            x, y, psi = straight + radius * math.sin(a), radius - radius * math.cos(a), a
        elif s < 2 * straight + math.pi * radius:
            u = s - straight - math.pi * radius
            x, y, psi = straight - u, 2 * radius, math.pi
        else:
            a = (s - 2 * straight - math.pi * radius) / radius
            x, y, psi = -radius * math.sin(a), radius + radius * math.cos(a), math.pi + a
        psi = math.atan2(math.sin(psi), math.cos(psi))
        vx = 3.0 + 1.0 * math.sin(2 * math.pi * s / length)
        wpnts.append((s, x, y, psi, vx))
    return wpnts


def car_pose(wpnts, s, track_length):
    s = s % track_length
    idx = min(int(round(s / 0.1)), len(wpnts) - 1)
    _, x, y, psi, _ = wpnts[idx]
    return x, y, psi


def build_scenario(wpnts):
    rng = np.random.default_rng(20260822)
    track_length = wpnts[-1][0]  # python node: data.wpnts[-1].s_m
    frames = []
    n_beams = 180
    a_min, a_max = -0.75 * math.pi, 0.75 * math.pi
    inc = (a_max - a_min) / (n_beams - 1)

    def norm(x):
        v = x % track_length
        return v - track_length if v > track_length / 2 else v

    for k in range(N_FRAMES):
        t = k * DT
        car_s = (3.0 * t) % track_length
        x, y, yaw = car_pose(wpnts, car_s, track_length)
        # scans: walls far away, except an "occluded" window near the end
        ranges = [9.0] * n_beams
        if 380 <= k < 420:
            ranges = [1.0] * n_beams
        meas = []
        # static obstacle A (disappears from the track after 12 s)
        if k < 240:
            sA, dA = 10.0, 0.3
            if 0 < norm(sA - car_s) < 8:
                meas.append((sA + rng.normal(0, 0.01), dA + rng.normal(0, 0.01), 0.3))
        # static obstacle B (flickers: 1 frame in 7 dropped, fully dropped during occlusion window)
        sB, dB = 25.0, -0.4
        if 0 < norm(sB - car_s) < 8 and k % 7 != 3 and not (380 <= k < 420):
            meas.append((sB + rng.normal(0, 0.01), dB + rng.normal(0, 0.01), 0.25))
        # opponent: 2.8 m/s ahead of the car, weaving in d, occluded for 2.5 s
        s_opp = (6.0 + 2.8 * t) % track_length
        d_opp = 0.25 * math.sin(0.7 * t)
        if 0 < norm(s_opp - car_s) < 8 and not (150 <= k < 200):
            meas.append((s_opp + rng.normal(0, 0.02), d_opp + rng.normal(0, 0.02), 0.35))
        # spurious one-frame detection
        if k % 37 == 0:
            meas.append(((car_s + 4.0 + rng.normal(0, 0.5)) % track_length, 0.8, 0.15))
        rng.shuffle(meas) if len(meas) > 1 else None
        frames.append(dict(k=k, car=(car_s, x, y, yaw),
                           scan=(a_min, a_max, inc, ranges), meas=meas))
    return frames


def fmt(v):
    return repr(float(v))


def write_scenario(path, wpnts, frames):
    with open(path, 'w') as f:
        f.write(f"PARAMS {RATE}\n")
        f.write(f"TRACK {len(wpnts)}\n")
        for w in wpnts:
            f.write(" ".join(fmt(v) for v in w) + "\n")
        for fr in frames:
            f.write(f"FRAME {fr['k']}\n")
            f.write("CAR " + " ".join(fmt(v) for v in fr['car']) + "\n")
            a_min, a_max, inc, ranges = fr['scan']
            f.write(f"SCAN {fmt(a_min)} {fmt(a_max)} {fmt(inc)} {len(ranges)} " +
                    " ".join(f"{r:.3f}" for r in ranges) + "\n")
            f.write(f"MEAS {len(fr['meas'])}\n")
            for m in fr['meas']:
                f.write(" ".join(fmt(v) for v in m) + "\n")
        f.write("END\n")


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(copy.deepcopy(msg))


def main():
    wpnts = build_track()
    frames = build_scenario(wpnts)
    golden_dir = os.path.join(HERE, 'golden')
    os.makedirs(golden_dir, exist_ok=True)
    write_scenario(os.path.join(golden_dir, 'tracking_scenario.txt'), wpnts, frames)

    rclpy.init(args=['--ros-args', '-p', f'rate:={RATE}'])
    from perception.tracking import StaticDynamic
    node = StaticDynamic()
    est_rec, raw_rec = Recorder(), Recorder()
    node.estimated_obstacles_pub = est_rec
    node.raw_opponent_pub = raw_rec
    node.static_dynamic_marker_pub = Recorder()

    path = WpntArray()
    for i, (s, x, y, psi, vx) in enumerate(wpnts):
        w = Wpnt()
        w.id = i
        w.s_m, w.x_m, w.y_m, w.psi_rad, w.vx_mps = s, x, y, psi, vx
        path.wpnts.append(w)
    node.pathCallback(path)

    def obs_line(o):
        return " ".join([str(o.id), fmt(o.s_center), fmt(o.d_center), fmt(o.s_start),
                         fmt(o.s_end), fmt(o.d_right), fmt(o.d_left), fmt(o.size),
                         fmt(o.vs), fmt(o.vd), str(int(o.is_static)), str(int(o.is_visible))])

    with open(os.path.join(golden_dir, 'tracking_expected.txt'), 'w') as out:
        for fr in frames:
            car_s, x, y, yaw = fr['car']
            od = Odometry()
            od.pose.pose.position.x = car_s
            node.carStateCallback(od)
            og = Odometry()
            og.pose.pose.position.x, og.pose.pose.position.y = x, y
            q = quaternion_from_euler(0.0, 0.0, yaw)
            (og.pose.pose.orientation.x, og.pose.pose.orientation.y,
             og.pose.pose.orientation.z, og.pose.pose.orientation.w) = q
            node.carStateGlobCallback(og)
            a_min, a_max, inc, ranges = fr['scan']
            sc = LaserScan()
            sc.angle_min, sc.angle_max, sc.angle_increment = a_min, a_max, inc
            sc.ranges = [float(f"{r:.3f}") for r in ranges]
            node.scansCallback(sc)
            oa = ObstacleArray()
            for (s, d, size) in fr['meas']:
                o = Obstacle()
                o.s_center, o.d_center, o.size = s, d, size
                oa.obstacles.append(o)
            node.obstacleCallback(oa)

            est_rec.msgs.clear()
            raw_rec.msgs.clear()
            node.loop()
            est = est_rec.msgs[-1].obstacles
            raw = raw_rec.msgs[-1].obstacles

            out.write(f"FRAME {fr['k']}\n")
            out.write(f"EST {len(est)}\n")
            for o in est:
                out.write(obs_line(o) + "\n")
            out.write(f"RAW {len(raw)}\n")
            for o in raw:
                out.write(obs_line(o) + "\n")
            opp = node.opponent_obstacle
            kf = opp.dynamic_kf
            out.write("OPP " + " ".join([str(int(opp.isInitialised)), str(opp.id or 0),
                                          fmt(kf.x[0]), fmt(kf.x[1]), fmt(kf.x[2]), fmt(kf.x[3]),
                                          fmt(kf.P[0][0]), str(opp.ttl or 0),
                                          str(int(opp.useTargetVel))]) + "\n")
            out.write(f"STATE {node.current_lap} {len(node.tracked_obstacles)}\n")
    node.destroy_node()
    rclpy.shutdown()
    print(f"wrote {N_FRAMES} frames to {golden_dir}")


if __name__ == '__main__':
    main()
