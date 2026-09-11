<div align="center">

# RX150 · Hedge Algebra Control

**Does the fuzzy rule base actually earn its keep?**
One real robot arm, three controllers swapped into the same slot, and the measurements to answer it.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Robot](https://img.shields.io/badge/Robot-Interbotix%20RX150-0A7E8C)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

[Tiếng Việt](README.md) · **English**

</div>

---

## The idea

The **Interbotix ReactorX-150** arm closes its position loop **inside the Dynamixel motor
firmware** by default. That loop cannot be observed, cannot be modified, and therefore
cannot be compared — if you want to ask "is this control law better than that one", there
is nowhere to ask it.

This project **pulls that loop out**. The motors are switched to `operating_mode: pwm`,
the firmware is demoted to a power amplifier, and the entire position loop runs on the
host at **100 Hz**. The control law becomes a **swappable object** — and that is where
the question gets asked.

Into that exact slot, this repository plugs **three different laws**:

| Controller | Law | Feedforward |
| :--- | :--- | :--- |
| [`rx150_fuzzy_controller`](src/rx150/controllers/rx150_fuzzy_controller/README.md) | Fuzzy Mamdani type-1, compiled from `.fis` to plain C | Gravity compensation `g(q)` (Pinocchio RNEA) |
| [`rx150_ff_controller`](src/rx150/controllers/rx150_ff_controller/README.md) | **The same** fuzzy rule base | `Kv·q̇ + Ka·q̈` — purely kinematic, no dynamics model |
| [`rx150_hac_controller`](src/rx150/controllers/rx150_hac_controller/README.md) | **HAC — Hedge Algebra Controller**, a linear three-coefficient surface | Gravity compensation `g(q)` |

### The central question

**HAC** collapses the **entire** Mamdani rule base — fuzzy sets, membership functions,
inference, defuzzification — down to **three scalars** `a`, `b`, `c`, through the semantic
quantifying mapping of hedge algebra:

$$u \;=\; \frac{2c}{3a}\,e \;+\; \frac{c}{3b}\,\dot e$$

The whole control surface fits in one line of C
([`hac.c`](src/rx150/controllers/rx150_hac_controller/src/hac/hac.c)), and `a/b/c` are
live-tunable through ROS parameters while the robot runs.

> **If HAC tracks as well as the fuzzy controller, then the "fuzzy" part contributed
> nothing beyond a nonlinear surface.** That is the question this repository was built to
> answer — with measurements on real hardware, not in simulation.

The comparison only means something if **everything else is held constant**: the same
100 Hz cycle, the same PWM command path, the same `joint_states` feedback, the same Ruckig
profile, and HAC's `error_limit`/`error_dot_limit` set to exactly `1/Ke`, `1/Ked` of the
fuzzy controller.

### Measured under real load, not on a bench

The three controllers do not run unloaded. A complete application sits on top of them:

* **Picking test tubes into a rack** — YOLO segmentation finds the tubes, mask → contour →
  endpoint → yaw angle, cap colour for sorting, and the gripper **confirms an object is
  between the fingers** before lifting.
* **Gesture-driven human–robot interaction** — MediaPipe Hands, an index-finger ray to
  point at the target, an OK sign to confirm.
* **AprilTag hand-eye calibration** plus a MoveIt planning scene for dynamic obstacles.

## What has been measured

Calibrating the gravity model directly on hardware (132 samples, 2026-09-09) replaces the
Pinocchio + datasheet-coefficient model with a trigonometric model fitted directly in PWM
units:

| Joint | Residual RMS — Pinocchio + `Gff` | Residual RMS — fitted model |
| :--- | ---: | ---: |
| shoulder | 53.8 | **35.4** |
| elbow | 80.0 | **60.2** |

Worst-case steady-state error over 6 poses: **2.55° → 1.95°**.

**But the remaining residual is not gravity.** Approaching the same pose from two
directions makes the error **flip sign** (elbow: −1.65° from one side, +1.43° from the
other). That is **stiction** — and no amount of gravity calibration will remove it. The
next lever is `friction_coulomb`/`friction_viscous`, still set to `[0,0,0,0,0]`.

→ Full numbers: [rx150_hac_controller/README.md](src/rx150/controllers/rx150_hac_controller/README.md) ·
raw data: `tuning_runs/gravity_identify_20260909_182815/`

## Architecture

Five layers following Interbotix's
[IRROS](https://github.com/Interbotix/interbotix_ros_core#code-structure) standard, so
project code drops into the upstream ecosystem unmodified:

```
Application         rx150_pick_place · rx150_hri          ← decides WHAT to pick, WHERE to place
Application Support rx150_modules · rx150_motion_common · rx150_perception
Control             rx150_fuzzy_controller │ rx150_ff_controller │ rx150_hac_controller
Driver              interbotix_xs_sdk · realsense2_camera · apriltag_ros
Hardware            RX150 (XL430/XM430) + U2D2 · RealSense D435i · AprilTag
```

**Upper layers call lower layers, never the reverse.** The three controllers share one
directory because they are **three variants of one problem** — you pick one of the three,
you do not run all three.

→ Full tree and dependency law: [docs/cau_truc_kho.md](docs/cau_truc_kho.md) ·
node/topic/rate map: [docs/so_do_dieu_khien.md](docs/so_do_dieu_khien.md)

## Quick start

Requires Ubuntu 22.04 + ROS 2 Humble.

```bash
git clone https://github.com/nguyenbinh-shark/RX150_Hedge_Algebra_Control.git ~/RX150_Hedge_Algebra_Control && cd ~/RX150_Hedge_Algebra_Control
./tools/setup_vendor.sh      # fetch Interbotix vendor sources (NOT committed) — required before building
./tools/build.sh             # colcon build, with the setuptools/packaging trap already handled
source source_all.sh
```

Every run mode goes through [`rx150.sh`](rx150.sh); call it with no arguments to list them.

```bash
./rx150.sh reach      # B0  reachability table + config check  — NO robot needed
./rx150.sh t1         # B1  robot + MoveIt + camera            (torque ON, arm homes itself)
./rx150.sh t2         # B2  perception + calibrated TF
./rx150.sh check      # B3  smoke test: joint_states + action server + TF/detection
./rx150.sh dry-fake   # B4  full state machine on fake tubes — no camera needed
./rx150.sh tubes      # B5  pick real test tubes
```

> ⚠️ In PWM mode, **losing the node drops the arm** — the firmware no longer holds
> position. The B0→B6 ladder has one rule: **whichever rung fails, stop at that rung.**

→ Detailed install: [docs/cai_dat.md](docs/cai_dat.md) ·
hardware procedure: [RUNBOOK](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md)

## Documentation

This README introduces the idea only. Every deeper document — installation, architecture,
control, calibration, perception, testing — is indexed in **[docs/](docs/README.md)**.

> Deep documentation is written in Vietnamese; these two root READMEs are the bilingual
> entry point.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). In short: new packages go at their correct IRROS
layer, **every package carries its own README**, and you never reimplement a primitive
that `rx150_modules` already provides.

## License & citation

BSD 3-Clause — see [LICENSE](LICENSE). `src/vendor/` retains the original licenses of
Interbotix and third parties. If you use this repository in academic work, see
[CITATION.cff](CITATION.cff).
