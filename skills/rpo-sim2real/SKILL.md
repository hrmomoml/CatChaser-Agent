---
name: rpo-sim2real
description: Adapt this project's RPO 23DOF MuJoCo motions to its ROS 2 upper-body Interrupt interface, verify SDK versions and joint contracts, and prepare host/robot deployment. Use for RPO motion migration and connected-robot commissioning, not generic ROS setup or RP1 control.
---

Work from this repository root. Read `config/upstream.lock.json`, `docs/INTERFACES.md`, and `docs/MIGRATION.md` before changing the transport or joint mapping. Source files under `src/rpo_cat_control` implement the project interface; `/rpo/command` is our extension, not an official vendor topic.

## Choose the stage

- No robot: run `scripts/doctor.py --role host`, export candidates with `scripts/export_motion.py`, build with `bash scripts/build_host.sh`, then run the mock and ROS integration test. This validates software messages, not hardware or balance.
- Robot available: use `docs/DEPLOYMENT.md`. The PC sends ROS messages; only the robot controller owns CAN and IMU. Inspect actual configuration, source commits, calibration and support setup. Apply the pinned patch with `scripts/apply_sdk_patch.sh` and build on the robot. The patch has a checked diff and unit-tested guard; its complete SDK build and hardware behavior remain unverified until performed there.
- Legs, waist or combined posture motion: the current Interrupt subscription controls only the last ten arm joints. Do not send a 23-element vector to `/joint_ref_states`, reinterpret `/cmd_vel` as a squat command, or describe a static SDK pose replay as a balanced controller. Follow the lower-body stages in `docs/MIGRATION.md`.

## Preserve the actual contract

`position` is ten finite absolute radian angles in left-five/right-five order. The pinned upstream ignores `name`, `velocity` and `effort`. Robot feedback has 23 logical positions and generic `joint_1` names. The SDK applies direction, zero offsets and parallel ankle mapping; do not apply those transforms a second time on the host.

Use the official robot gains for hardware, not the MuJoCo gains or simulator bias forces. The squat reference is now shallow: hip pitch -1.40 rad, knee 1.55 rad, ankle pitch -0.50 rad, torso lean 0.35 rad. All twelve posture/arm combinations passed 2 ms checks of actual/reference angles against the deployment profile. Mesh intersections and real lower-body balance remain unresolved; do not treat the limit correction as hardware validation.

## Validate and report

Run contract tests, the C++ packet guard test, ROS mock integration, and inspect exported JSON metadata. Before enabling real output, confirm the robot-side patched service is running, feedback is fresh, and deployment prerequisites are satisfied; use explicit `backend:=official_interrupt` and `hardware_enabled:=true`. Stop the sender to exercise the robot-side timeout while the robot is supported.

Report separately: code built, mock tested, cross-machine DDS tested, SDK compiled, hardware calibrated, supported hardware trial, and free-standing motion validated. Never promote one stage into another. GitHub upload requires a user-selected destination and usable authentication; keep a local repository ready when either is absent.

For behavioral checks and commands, use [the scenario reference](references/scenarios.md). Do not initialize motors, write zero points, or start inference as part of an environment check.
