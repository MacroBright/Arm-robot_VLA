# Server Simulation Startup Fixes Design

## Objective

Make the Arm-robot_VLA MuJoCo simulator start safely in headless, camera-disabled mode on a shared Linux server without requiring command-line workarounds or touching another user's shared-memory objects.

## Scope

This change fixes exactly two verified defects:

1. The default scene path points to the removed `scripts/simulation/mujoco_scene/scene.xml` path instead of the tracked `scripts/simulation/scene.xml` file.
2. `--no-camera` still allocates and attempts to replace the fixed shared-memory objects `mujoco_frame_0` and `mujoco_frame_ee`, even though no camera producer or frame consumer is started.

The change does not alter TCP protocol behavior, robot dynamics, camera-enabled shared-memory names, hardware paths, Docker files, or the top-level TuinaDex integration skeleton.

## Design

### Default scene

`SCENE_PATH` will resolve directly beside `mujoco_sim.py` as `scene.xml`. The command `python scripts/simulation/mujoco_sim.py --no-viewer --no-camera` must therefore find the tracked model without `--scene`.

### Optional frame buffers

`MuJoCoArm` will accept a boolean `enable_frame_buffers` argument, defaulting to `True` for backward compatibility. When false:

- it will not open, unlink, create, write, close, or unlink either shared-memory object;
- its shared-memory attributes will remain `None`;
- physics stepping and TCP commands will remain available.

`main()` will pass `enable_frame_buffers=not args.no_camera`. Camera-enabled behavior remains unchanged, including the legacy shared-memory names expected by `camera_server.py`, `record_sim.py`, `control_hub.py`, and `evaluate_policy.py`.

This is preferred over renaming shared memory because it solves the verified `--no-camera` defect without changing the inter-process contract for camera-enabled workflows.

## Error handling and safety

Camera-enabled mode retains current cleanup behavior. Camera-disabled cleanup must tolerate `None` buffers. The implementation must never unlink an existing shared-memory object when frame buffers are disabled.

No test or implementation step may delete or change `/dev/shm/mujoco_frame_0` or `/dev/shm/mujoco_frame_ee`, which are owned by another server user.

## Tests

Tests will be added before production changes and observed failing for the intended reasons:

1. The default `SCENE_PATH` exists and equals the tracked `scripts/simulation/scene.xml` path.
2. Constructing `MuJoCoArm` with frame buffers disabled does not call `SharedMemory` and leaves both buffer attributes as `None`.
3. Cleanup with frame buffers disabled completes without accessing shared-memory methods.

After implementation, run the focused regression tests, the existing 74-test no-hardware suite, `pip check`, and a temporary TCP smoke test using the default scene path and `--no-camera`. The smoke test must verify `get_state`, `set_torque`, `set_joints`, and `e_stop`, then confirm its port is closed and it left no `maziqi` shared-memory objects.

## Git isolation

All changes live in the server worktree `~/HUAWEI_contest_project/worktrees/arm-server-sim-baseline` on branch `codex/server-sim-baseline`. No push, merge, submodule bump, or modification of the main TuinaDex checkout is included.
