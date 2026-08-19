# Server Simulation Startup Fixes — Verification

## Scope

Branch `codex/server-sim-baseline` fixes two shared-server startup defects without changing camera-enabled behavior, TCP protocol semantics, robot dynamics, hardware access, Docker configuration, or the TuinaDex main branch.

## Improvements

1. `SCENE_PATH` now resolves to the tracked `scripts/simulation/scene.xml`, so the default CLI no longer requires an explicit `--scene` workaround.
2. `MuJoCoArm` accepts `enable_frame_buffers=True` for backward compatibility.
3. `main()` maps `--no-camera` to `enable_frame_buffers=False`.
4. Camera-disabled mode does not open, unlink, create, close, or unlink `mujoco_frame_0` or `mujoco_frame_ee`.
5. Cleanup explicitly handles disabled frame buffers.
6. `docs/MUJOCO_SIM.md` documents the server-safe command:

   ```bash
   python scripts/simulation/mujoco_sim.py --no-viewer --no-camera
   ```

## Regression evidence

- New focused tests: `2 passed`.
- Complete selected no-hardware suite: `76 passed in 11.75s`.
- Dependency integrity: `python -m pip check` reported `No broken requirements found`.
- Direct CLI smoke test used the default scene and port `15555`.
- TCP responses succeeded for `get_state`, `set_torque 1`, `set_joints`, a second `get_state`, and `e_stop`.
- The temporary TCP port closed after the test.
- The set of `maziqi`-owned `mujoco_frame*` shared-memory objects was unchanged.
- Existing `/dev/shm/mujoco_frame_0` and `/dev/shm/mujoco_frame_ee` remained owned by `zhuyan` and were not modified.

## Repository isolation

The work exists only in the server worktree `~/HUAWEI_contest_project/worktrees/arm-server-sim-baseline`. It has not been pushed, merged, or applied to the TuinaDex main checkout.
