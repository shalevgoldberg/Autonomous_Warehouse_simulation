# Autonomous Warehouse Physics Migration Plan

## Introduction

This document outlines the phased implementation plan for migrating the autonomous warehouse simulation from in-house kinematic physics to real MuJoCo physics contacts. The plan prioritizes simplicity, robustness, and non-invasive changes while maintaining existing interfaces and scalability.

**Objective:**
- Keep simulation simple and robust
- Use real MuJoCo solver for robot/shelf/world contacts eventually, but no physical shelf carrying
- Maintain existing interfaces; minimize invasive changes

## Phase 0 — Stabilize current shared world (non‑invasive)

### Changes
- Add 1 kHz time‑gating and reentrant lock in `SharedMuJoCoEngine.step()`
- Guard against stepping before initialization
- Remove non‑ASCII/emoji from `demo_multi_robot_simulation.py` logs (Windows-safe)
- Clarify docstrings/comments: current mode is "kinematic + contact guard," not solver

### Acceptance
- Demo runs on Windows; no Unicode errors
- With 2–10 robots, CPU stable; step cadence ≈ 1 kHz; no double‑stepping
- No behavior change except improved stability

## Phase 1 — Visual shelf "carry" (no physics welds)

### Changes
- Add `AppearanceService` (engine-owned) to toggle robot material/rgba when "carrying"
- Integrate with `TaskHandlerImpl`: on shelf lock/acquire → set "carrying" appearance; on release → revert
- Add config flag: `appearance.carry_color` and enable/disable

### Acceptance
- Robot visibly marked while "carrying"
- No MuJoCo model rebuilds; no stalls; physics unchanged

## Phase 2 — Authoritative MuJoCo contacts (no acceleration, no shelf welds)

### Changes
- Add `physics.mode` flag: `kinematic_guard` (default) | `mujoco_authoritative`
- Model each robot with planar DOFs: slide‑x, slide‑y, hinge‑yaw; cylinder geom; zero gravity
- Add velocity actuators for the 3 DOFs; map controller v,w → (ẋ, ẏ, θ̇). No acceleration modeling; optional mild damping
- Engine runs a dedicated physics thread calling `mj_step` at dt=0.001. Adapters write actuator targets; poses read from `data`
- In `mujoco_authoritative` mode, remove `_would_collide` use and any pose rewinds

### Acceptance
- Robots do not interpenetrate shelves/robots via solver (no manual O(N) scans)
- Controller API unchanged; motion remains "snappy"
- Switchable at runtime via config/env for A/B

## Phase 3 — Clean up kinematic path and registries

### Changes
- Keep `kinematic_guard` path for compatibility; mark `CollisionRegistry` as deprecated
- Ensure both modes are tested; solvers handle all contacts in authoritative mode

### Acceptance
- No O(N) contact scans when `mujoco_authoritative` is enabled
- Old mode still works for fallback

## Phase 4 — LiDAR hooks (solver-agnostic)

### Changes
- Add `RaycastService` in engine for batched rays against current `data` snapshot
- Expose `SharedMuJoCoPhysics.get_lidar_scan(config)`; compute on physics thread or a locked snapshot

### Acceptance
- Deterministic scans against simple fixtures (walls/shelves) within tolerances
- No coupling to shelf welding (shelves remain static)

## Phase 5 — Tests, metrics, performance

### Tests
- Mode parity tests: follow path in both modes
- Contact tests: robot vs shelf, robot vs robot (authoritative mode uses solver; no manual checks)
- Visual carry tests: appearance toggles with task lifecycle
- LiDAR sanity tests: known ranges in fixtures

### Observability
- Add step timing metrics, ncon/contact counts, mode, and skipped-step counters

### Acceptance
- All tests green; physics step meets 1 kHz under typical loads (10–20 robots)

## Design notes (SOLID and scalability)

### SOLID Principles
- **Single responsibility**: `SharedMuJoCoEngine` owns world state/stepping; adapters only translate commands and expose poses
- **Open/closed**: Add authoritative mode without breaking existing APIs
- **Dependency inversion**: Agents depend on `IPhysicsCommandSink`/`IStateHolder`; engine satisfies both modes
- **Extensibility**: Shelf welding remains an optional future feature; current "visual carry" avoids model rebuilds entirely

## Risks and mitigations

### Risks
- **Solver tuning**: start with conservative defaults (0 gravity, stable friction, high DOF damping); adjust only if needed
- **Threading**: strict locking around `data` access; snapshots for sensors
- **Migration**: feature-flag rollout; keep kinematic path as fallback

### Current Implementation Status
- ✅ **Phase 0**: Completed (shared engine, time-gating, Windows-safe logs)
- ✅ **Duplicate cleanup**: SQL function duplication eliminated
- 🔄 **Phase 1**: Ready for implementation (visual shelf carry)
- ⏳ **Phase 2**: Planned (authoritative MuJoCo contacts)
- ⏳ **Phase 3-5**: Future phases

## Next Steps

If you approve, I'll execute Phase 1 (Visual shelf "carry") next, then introduce the feature-flagged authoritative mode in Phase 2. The current kinematic + contact guard system provides a stable foundation for these incremental improvements.
