# f1tenth_gym 시뮬레이션 (1단계) — 노트북용

Jetson `fresh_ws`와 완전히 분리된 `~/sim_ws`에서 스택 전체(carstate → planner → SM → controller)를
2D gym 시뮬레이터 위에서 돌린다. 코드 동기화는 git(`sim-baseline` 브랜치)으로만 한다.

## 셋업
```bash
source /opt/ros/humble/setup.bash
git clone -b sim-baseline https://github.com/churrosboy/SKKUDERIA_SKKAI.git /tmp/skkai && \
  /tmp/skkai/sim/setup_sim_ws.sh ~/sim_ws ~/Downloads/maps_latest.tar.gz
```
`maps_latest.tar.gz`는 Jetson에서 `tar czf` 로 만든 `stack_master/maps/latest` (pbstream 제외). 맵 폴더는 .gitignore라 git으로 안 넘어온다.

## 실행 (터미널마다 `sim` 함수 먼저)
```bash
# 1) 시뮬 + base system (RViz 포함)
ros2 launch stack_master base_system_launch.xml sim:=True racecar_version:=SIM map_name:=latest
# 2) 타임트라이얼
ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM ctrl_algo:=PP            # python PP
ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM ctrl_algo:=PP ctrl_exec:=controller_cpp
ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM ctrl_algo:=MAP
# 3) 장애물/추월: config/SIM/sim.yaml num_agent: 2 (opp 차 sx1/sy1) 후
ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM overtake_mode:=spliner   # lane_change / sqp
```
시뮬에서 실차와 다른 점: localization(Cartographer) 없음 — gym이 정답 pose를 `/car_state/odom`, TF로 준다.
따라서 initialpose snap / straight EKF blend / tight lua 등 localization 관련 항목은 여기서 검증 불가.

## 검증 체크리스트 (모두 "car untested"였던 것)
- [ ] TT PP python 1랩 → 랩타임, `|d|` 최대, `/drive` 20 Hz
- [ ] `ctrl_exec:=controller_cpp` A/B (동일 궤적?)
- [ ] `enable_recovery_state:=True` — 회피 후 복귀 시 스티어 스파이크 없는지
- [ ] `l1_curv_cap_enable` (hairpin L1 cap), launch ramp (정지 출발)
- [ ] speed_scaling.yaml `SectorN.lane` middle/left/right
- [ ] 정지 opp 차 → spliner / lane_change(weave 켜고/끄고) / sqp / FTG 진입 (`ftg_threshold_speed 1.0`)
- [ ] 저속 이동 opp 차 → TRAILING creep, closing gate
- [ ] `ros2 bag record -a` 로 남겨 실차 bag(lap_21_53_45, obs_stop_21_03_50)과 비교

## 시뮬 LiDAR = 실차 GL-5 조건 (2026-08-25)
`config/SIM/sim.yaml`: `scan_fov 4.712` (270°), `scan_beams 1500` (0.18°/빔), `scan_range_max 6.0` (실측 유효거리; 초과 빔은 inf 드롭아웃), `scan_rate_hz 40`.
브릿지 패치: 스캔 전용 타이머 + range 클리핑 (`gym_bridge.py`, 서브모듈), gym에 `num_beams/fov` kwargs 연결 (`f110_env.py`, `base_classes.py`, 서브모듈 — 원래 sim.yaml 빔 수가 gym에 전달되지 않는 upstream 버그).
발행 주기도 실차와 동일 (`odom_rate_hz 80` = carstate_node, `tf_rate_hz 200` = cartographer pose_publish_period 5 ms); 물리 스텝은 100 Hz 그대로.
측정: scan 39.8 / odom·pose·frenet 80 / TF ~190 (5 ms 타이머 지터) / drive 20 Hz. 원래(30 m/100 Hz/1080빔)로 돌리려면 yaml 값만 바꾸면 됨.

## 운용 gotcha
- 충돌하면 gym이 차를 영구 고정 → RViz **2D Pose Estimate**(또는 `/initialpose` best-effort)로 리셋. "안 움직임" = 대부분 이것.
- 시뮬(`sim`)만 재시작하면 떠 있던 TT 컨트롤러가 스테일 lookahead로 첫 조향을 풀락으로 내 벽에 박힘 → 재시작 후 2D Pose Estimate 한 번.
- launch 실패 시 `gym_bridge` 고아가 남아 `/scan`이 이중 발행됨(200 Hz) → `pkill -x gym_bridge` 후 재시작.
- gym은 물리 100 Hz마다 1500빔 레이캐스트 → 브릿지 CPU ~80%. 젯슨에서 부담되면 `scan_beams 1080`.

## 파라미터 출처
- `config/SIM/l1_params.yaml`, `sim_params.yaml` = upstream 값. 실차 튜닝값으로 맞추려면 `config/NUC2/l1_params.yaml` 을 SIM 으로 복사.
- 시작 포즈는 `sim.yaml sx/sy/stheta` = `maps/latest/latest.yaml initial_pose` 로 맞춰 둠 (2026-08-25).
- 맵 스케일/섹터: `maps/latest/speed_scaling.yaml`, `ot_sectors.yaml` 그대로 사용.

## 동기화
- 실차 → 시뮬: Jetson에서 `fresh` 커밋 후 `git push origin fresh:sim-baseline`, 노트북 `git pull`.
- 시뮬 → 실차: 노트북에서 브랜치 커밋 → Jetson에서 검토 후 merge. 실차 코드는 절대 노트북에서 직접 건드리지 않는다.
