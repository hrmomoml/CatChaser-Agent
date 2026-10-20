# Workflow checks

## “没有真机，演示动作 7”

Build the ROS package, export the arm library, start `run_mock.sh` and `run_player.sh`, source `host_env.sh`, and publish `std_msgs/msg/String` with `data: '7'` to `/rpo/command`. Expected: ten finite joint targets on `/rpo_mock/joint_ref_states`, changing mock joint feedback, no CAN or SDK access. `/rpo/status` reports playback state.

## “发送 1+8 到真机”

The present network player must reject this command: its backend has no lower-body posture interface. Explain the trained lower-body controller and limit/geometry work in the migration plan. Exporting a full reference is allowed for analysis; keep its whole-body hardware replay flag false.

## “上位机掉线”

With only the mock, terminate the player. Within the receiver's 0.5-second timeout it must leave external arm mode. Invalid 9-element or NaN packets must not refresh the timeout. On hardware, verify this behavior on a supported robot after rebuilding the patched SDK. The fallback returns arm ownership to the existing policy; it is not motor deinitialization or a physical emergency stop.

## “上传 GitHub”

Check `git status`, ignored data, `docs/VALIDATION.md` and the selected remote. Use user-provided repository and visibility. Never include build/install/log directories, robot calibration files, tokens or downloaded SDK binaries. With no selected remote, report local readiness rather than inventing a published URL.
