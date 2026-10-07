#!/usr/bin/env python3
"""Golden-vector generator for the C++ port of PP_Controller (pp_core).

Runs deterministic multi-cycle scenarios through the PYTHON PP_Controller
(controller/pp.py) and dumps the inputs + outputs to:
  test/data/pp_golden.json  -- human-inspectable
  test/data/pp_golden.inc   -- C++ initializer code included by test_pp_core.cpp

Regenerate whenever pp.py changes:
    cd src/controller && PYTHONPATH=. python3 test/generate_pp_golden.py
A parity failure in test_pp_core after regeneration means the C++ and python
implementations diverged.

Multi-cycle sequences matter: PP_Controller keeps state across calls
(curr_steering_angle slew base, i_gap integrator, retained curvature mean).
"""
import json
import math
import os

import numpy as np

from controller.pp import PP_Controller

# Fixture params: hardcoded on purpose -- the test fixture must not drift when
# the live yaml is tuned.
PARAMS = dict(
    t_clip_min=0.8,
    t_clip_max=5.0,
    m_l1=0.55,
    q_l1=-0.167,
    speed_lookahead=0.25,
    lat_err_coeff=1.0,
    acc_scaler_for_steer=1.0,
    dec_scaler_for_steer=0.9,
    start_scale_speed=7.0,
    end_scale_speed=8.0,
    downscale_factor=0.2,
    speed_lookahead_for_steer=0.0,
    prioritize_dyn=True,
    trailing_gap=2.0,
    trailing_p_gain=1.0,
    trailing_i_gain=0.0,
    trailing_d_gain=0.2,
    blind_trailing_speed=1.5,
    trailing_vel_gain=0.0,
    trailing_min_speed=0.8,
    loop_rate=20,
    wheelbase=0.338,
    trailing_creep_always=True,
    trailing_stop_gap=0.3,
    trailing_nose_offset=0.45,
)
# attributes controller_manager injects per cycle
INJECT = dict(
    recovery_l1_gain=0.3,
    recovery_t_clip_max=6.0,
    l1_curv_cap_enable=False,
    l1_curv_cap_max_heading=1.57,
)


def make_controller(overrides=None, inject_overrides=None):
    p = dict(PARAMS)
    if overrides:
        p.update(overrides)
    ctrl = PP_Controller(
        p["t_clip_min"], p["t_clip_max"], p["m_l1"], p["q_l1"],
        p["speed_lookahead"], p["lat_err_coeff"], p["acc_scaler_for_steer"],
        p["dec_scaler_for_steer"], p["start_scale_speed"], p["end_scale_speed"],
        p["downscale_factor"], p["speed_lookahead_for_steer"],
        p["prioritize_dyn"], p["trailing_gap"], p["trailing_p_gain"],
        p["trailing_i_gain"], p["trailing_d_gain"], p["blind_trailing_speed"],
        p["trailing_vel_gain"], p["trailing_min_speed"], p["loop_rate"],
        p["wheelbase"],
        logger_info=lambda *_: None, logger_warn=lambda *_: None,
        trailing_creep_always=p["trailing_creep_always"],
        trailing_stop_gap=p["trailing_stop_gap"],
        trailing_nose_offset=p["trailing_nose_offset"])
    inj = dict(INJECT)
    if inject_overrides:
        inj.update(inject_overrides)
    for k, v in inj.items():
        setattr(ctrl, k, v)
    return ctrl, p, inj


def straight_wpts(n=200, v=3.0):
    # along +x, 0.1 m spacing, s == x
    return [[i * 0.1, 0.0, v, 0.25, i * 0.1, 0.0, 0.0, 0.0] for i in range(n)]


def circle_wpts(n=200, radius=5.0, v=2.5):
    # constant-curvature left turn starting at origin heading +x
    kappa = 1.0 / radius
    pts = []
    for i in range(n):
        s = i * 0.1
        ang = s / radius
        x = radius * math.sin(ang)
        y = radius * (1.0 - math.cos(ang))
        psi = ang
        pts.append([x, y, v, 0.25, s, kappa, psi, 0.0])
    return pts


def hairpin_wpts(n=200, v=2.0):
    # straight 5 m then a tight r=0.6 hairpin
    pts = []
    for i in range(n):
        s = i * 0.1
        if s < 5.0:
            pts.append([s, 0.0, v, 0.25, s, 0.0, 0.0, 0.0])
        else:
            r = 0.6
            ang = (s - 5.0) / r
            x = 5.0 + r * math.sin(ang)
            y = r * (1.0 - math.cos(ang))
            pts.append([x, y, v, 0.25, s, 1.0 / r, ang, 0.0])
    return pts


def run_scenario(name, wpts, cycles, overrides=None, inject_overrides=None,
                 track_length=100.0):
    ctrl, p, inj = make_controller(overrides, inject_overrides)
    wpts_np = np.array(wpts)
    out_cycles = []
    for c in cycles:
        state = c.get("state", "StateType.GB_TRACK")
        opponent = c.get("opponent")  # None or [s, d, vs, static, visible, size]
        pos = np.array([[c["x"], c["y"], c["yaw"]]])
        frenet = np.array([c["s"], c["d"], c.get("vs", 0.0), c.get("vd", 0.0)])
        acc = np.array([c.get("acc_mean", 0.0)])  # pp.py only uses np.mean(acc)
        speed, accel, jerk, steer, l1_point, l1_dist, idx = ctrl.main_loop(
            state, pos, wpts_np, c["speed_now"], opponent, frenet, acc,
            track_length)
        trailing_ran = state == "StateType.TRAILING" and opponent is not None
        out_cycles.append({
            "in": {
                "is_trailing": state == "StateType.TRAILING",
                "is_recovery": state == "StateType.RECOVERY",
                "x": c["x"], "y": c["y"], "yaw": c["yaw"],
                "s": c["s"], "d": c["d"],
                "vs": c.get("vs", 0.0), "vd": c.get("vd", 0.0),
                "speed_now": c["speed_now"],
                "acc_mean": c.get("acc_mean", 0.0),
                "track_length": track_length,
                "opponent": opponent,
            },
            "out": {
                "speed": float(speed),
                "steering_angle": float(steer),
                "l1_x": float(l1_point[0]), "l1_y": float(l1_point[1]),
                "l1_distance": float(l1_dist),
                "idx_nearest_waypoint": int(idx),
                "trailing_ran": trailing_ran,
                "gap": float(ctrl.gap) if ctrl.gap is not None else 0.0,
                "gap_should": float(ctrl.gap_should) if ctrl.gap_should is not None else 0.0,
                "gap_error": float(ctrl.gap_error) if ctrl.gap_error is not None else 0.0,
                "v_diff": float(ctrl.v_diff) if ctrl.v_diff is not None else 0.0,
                "i_gap": float(ctrl.i_gap),
                "trailing_command": float(ctrl.trailing_command),
            },
        })
    return {
        "name": name,
        "params": p,
        "inject": inj,
        "waypoints": wpts,
        "cycles": out_cycles,
    }


def build_scenarios():
    scenarios = []

    # 1. straight GB_TRACK: car converging onto the line, speed rising
    cycles = []
    for i in range(10):
        d = 0.3 - 0.03 * i
        cycles.append(dict(x=1.0 + 0.15 * i, y=d, yaw=-0.05, s=1.0 + 0.15 * i,
                           d=d, vs=3.0, speed_now=1.0 + 0.2 * i))
    scenarios.append(run_scenario("straight_gb", straight_wpts(), cycles))

    # 2. constant-curvature circle: sustained steering + slew transient from 0
    cycles = []
    for i in range(10):
        s = 0.5 + 0.12 * i
        ang = s / 5.0
        cycles.append(dict(x=5.0 * math.sin(ang) + 0.05, y=5.0 * (1 - math.cos(ang)) - 0.02,
                           yaw=ang + 0.02, s=s, d=-0.02, vs=2.4,
                           speed_now=2.4, acc_mean=1.2))  # acc_scaling active
    scenarios.append(run_scenario("circle_gb", circle_wpts(), cycles))

    # 3. far off-track: lower_bound = sqrt(2)*|d| exceeds t_clip_max -> numpy
    # clip lo>hi quirk (hi wins); also exercises lat-err speed scaling floor
    cycles = [dict(x=2.0, y=4.0, yaw=0.3, s=2.0, d=4.0, vs=1.0, speed_now=2.0),
              dict(x=2.2, y=3.8, yaw=0.25, s=2.2, d=3.8, vs=1.0, speed_now=2.1),
              dict(x=2.4, y=3.6, yaw=0.2, s=2.4, d=3.6, vs=1.0, speed_now=2.2)]
    scenarios.append(run_scenario("large_lat_err", straight_wpts(), cycles))

    # 4. TRAILING sequence: approach visible opponent -> blind -> creep window
    # -> bumper-clearance stop; includes s-wrap (opponent s < ego s)
    cycles = []
    # visible, gap > gap_should (creep floor for visible targets)
    for i in range(3):
        cycles.append(dict(state="StateType.TRAILING",
                           opponent=[8.0 - 0.3 * i, 0.0, 0.5, False, True, 0.3],
                           x=4.0 + 0.1 * i, y=0.0, yaw=0.0, s=4.0 + 0.1 * i,
                           d=0.0, vs=1.5, speed_now=1.5))
    # blind cycles (is_visible False, gap > gap_should -> blind floor)
    for i in range(2):
        cycles.append(dict(state="StateType.TRAILING",
                           opponent=[7.5, 0.0, 0.5, False, False, 0.3],
                           x=4.3, y=0.0, yaw=0.0, s=4.3, d=0.0, vs=1.2,
                           speed_now=1.2))
    # inside desired gap with creep_always: clearance above stop_gap -> creep
    cycles.append(dict(state="StateType.TRAILING",
                       opponent=[6.0, 0.0, 0.0, True, True, 0.3],
                       x=4.5, y=0.0, yaw=0.0, s=4.5, d=0.0, vs=0.8,
                       speed_now=0.8))
    # clearance below stop_gap -> truly stop (no floor)
    cycles.append(dict(state="StateType.TRAILING",
                       opponent=[5.3, 0.0, 0.0, True, True, 0.3],
                       x=4.5, y=0.0, yaw=0.0, s=4.5, d=0.0, vs=0.3,
                       speed_now=0.3))
    # s-wrap: opponent just past track origin, ego near track end
    cycles.append(dict(state="StateType.TRAILING",
                       opponent=[1.0, 0.0, 1.0, False, True, 0.3],
                       x=9.0, y=0.0, yaw=0.0, s=99.0, d=0.0, vs=1.0,
                       speed_now=1.0))
    # back to GB_TRACK: integrator resets (i_gap = 0 branch)
    cycles.append(dict(x=5.0, y=0.0, yaw=0.0, s=5.0, d=0.0, vs=2.0,
                       speed_now=2.0))
    scenarios.append(run_scenario("trailing_seq", straight_wpts(), cycles,
                                  track_length=100.0))

    # 5. RECOVERY: extended lookahead active (gain 0.3) vs plain second cycle
    cycles = [dict(state="StateType.RECOVERY", x=1.0, y=0.6, yaw=0.0, s=1.0,
                   d=0.6, vs=2.0, speed_now=3.0),
              dict(state="StateType.RECOVERY", x=1.3, y=0.5, yaw=-0.05, s=1.3,
                   d=0.5, vs=2.2, speed_now=3.2),
              dict(x=1.6, y=0.4, yaw=-0.05, s=1.6, d=0.4, vs=2.4,
                   speed_now=3.4)]
    scenarios.append(run_scenario("recovery", straight_wpts(), cycles))

    # 6. hairpin curv cap ON: L1 capped where cumulative |kappa|*0.1 > pi/2
    cycles = [dict(x=4.0, y=0.0, yaw=0.0, s=4.0, d=0.0, vs=2.0, speed_now=4.0),
              dict(x=4.4, y=0.0, yaw=0.0, s=4.4, d=0.0, vs=2.0, speed_now=4.0),
              dict(x=4.8, y=0.0, yaw=0.0, s=4.8, d=0.0, vs=2.0, speed_now=4.0)]
    scenarios.append(run_scenario(
        "hairpin_cap", hairpin_wpts(), cycles,
        inject_overrides=dict(l1_curv_cap_enable=True)))

    # 7. deceleration steer scaling + steer slew clip: hard turn request from
    # straight-line state, acc_mean below -1
    cycles = [dict(x=0.5, y=0.0, yaw=0.0, s=0.5, d=0.0, vs=2.0, speed_now=2.0,
                   acc_mean=-1.5),
              dict(x=0.6, y=-0.4, yaw=0.8, s=0.6, d=-0.4, vs=2.0,
                   speed_now=2.0, acc_mean=-1.5),
              dict(x=0.7, y=-0.5, yaw=1.2, s=0.7, d=-0.5, vs=2.0,
                   speed_now=2.0, acc_mean=-1.5)]
    scenarios.append(run_scenario("decel_slew", circle_wpts(radius=2.0), cycles))

    # 8. high speed: speed_steer_scaling active (between start/end scale speeds)
    cycles = [dict(x=2.0 + 0.4 * i, y=0.05, yaw=0.0, s=2.0 + 0.4 * i, d=0.05,
                   vs=7.5, speed_now=7.5) for i in range(5)]
    scenarios.append(run_scenario("high_speed", straight_wpts(v=7.6), cycles))

    return scenarios


def fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int,)):
        return str(v)
    return repr(float(v))  # shortest round-trip representation


def emit_inc(scenarios, path):
    lines = []
    lines.append("// AUTO-GENERATED by test/generate_pp_golden.py -- do not edit.")
    lines.append("// Regenerate: cd src/controller && PYTHONPATH=. python3 test/generate_pp_golden.py")
    lines.append("static const std::vector<GoldenScenario> kGoldenScenarios = []{")
    lines.append("  std::vector<GoldenScenario> all;")
    for sc in scenarios:
        p = sc["params"]
        inj = sc["inject"]
        lines.append("  {")
        lines.append("    GoldenScenario s;")
        lines.append(f"    s.name = \"{sc['name']}\";")
        for k in ("t_clip_min", "t_clip_max", "m_l1", "q_l1", "speed_lookahead",
                  "lat_err_coeff", "acc_scaler_for_steer", "dec_scaler_for_steer",
                  "start_scale_speed", "end_scale_speed", "downscale_factor",
                  "speed_lookahead_for_steer", "trailing_gap", "trailing_p_gain",
                  "trailing_i_gain", "trailing_d_gain", "blind_trailing_speed",
                  "trailing_vel_gain", "trailing_min_speed",
                  "trailing_creep_always", "trailing_stop_gap",
                  "trailing_nose_offset", "wheelbase"):
            lines.append(f"    s.params.{k} = {fmt(p[k])};")
        lines.append(f"    s.params.loop_rate = {fmt(float(p['loop_rate']))};")
        for k, v in inj.items():
            lines.append(f"    s.params.{k} = {fmt(v)};")
        rows = ", ".join(
            "{" + ", ".join(fmt(x) for x in w) + "}" for w in sc["waypoints"])
        lines.append(f"    s.wpts = {{{rows}}};")
        for c in sc["cycles"]:
            i, o = c["in"], c["out"]
            opp = i["opponent"]
            opp_str = ("{true, " + ", ".join(
                [fmt(opp[0]), fmt(opp[1]), fmt(opp[2]), fmt(bool(opp[3])),
                 fmt(bool(opp[4])), fmt(opp[5])]) + "}") if opp is not None \
                else "{false, 0.0, 0.0, 0.0, false, false, 0.0}"
            row = ", ".join([
                fmt(i["is_trailing"]), fmt(i["is_recovery"]),
                fmt(i["x"]), fmt(i["y"]), fmt(i["yaw"]),
                fmt(i["s"]), fmt(i["d"]), fmt(i["vs"]), fmt(i["vd"]),
                fmt(i["speed_now"]), fmt(i["acc_mean"]), fmt(i["track_length"]),
                opp_str,
                fmt(o["speed"]), fmt(o["steering_angle"]),
                fmt(o["l1_x"]), fmt(o["l1_y"]), fmt(o["l1_distance"]),
                fmt(o["idx_nearest_waypoint"]), fmt(o["trailing_ran"]),
                fmt(o["gap"]), fmt(o["gap_should"]), fmt(o["gap_error"]),
                fmt(o["v_diff"]), fmt(o["i_gap"]), fmt(o["trailing_command"]),
            ])
            lines.append(f"    s.cycles.push_back({{{row}}});")
        lines.append("    all.push_back(std::move(s));")
        lines.append("  }")
    lines.append("  return all;")
    lines.append("}();")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(here, "data")
    os.makedirs(data_dir, exist_ok=True)
    scenarios = build_scenarios()
    with open(os.path.join(data_dir, "pp_golden.json"), "w") as f:
        json.dump(scenarios, f, indent=1)
    emit_inc(scenarios, os.path.join(data_dir, "pp_golden.inc"))
    n = sum(len(s["cycles"]) for s in scenarios)
    print(f"wrote {len(scenarios)} scenarios / {n} cycles to {data_dir}")


if __name__ == "__main__":
    main()
