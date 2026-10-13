# Copilot handoff — rebuild GP4 table arm on restored `main`

## Your assignment

You are working on the UTS Industrial Robotics tea-making workcell. The GitHub repository `Louisarnold143/IndustrialRobotics` has been recovered: its **restored `main`** contains the original 24 historical commits plus commit `32eb238` (`Restore updated robot workcell code`), giving **25 commits** before any new GP4 implementation.

**Implement our new table robot directly in the project's production entry point `main.py` and production controller `tableArm.py`. Do NOT create, retain, or wire in test entry points such as `main_GP4.py`, `main_GP4_base_closer.py`, `main_GP4_wall_lower.py`, `main_GP7.py`, or a second production `tableArmGP4.py`.** The completed project should run via `python main.py` as before. It is acceptable and necessary to keep the reusable kinematic model as **`gp4_robot.py`** and the real GP4 assets in **`GP4_reference/`**. The aim is to change the active table arm from the previous JAKA/test robot to the **Yaskawa Motoman GP4**, using its actual 6-axis kinematics and STL meshes.

Do not add a `Co-authored-by` line or mention Copilot as an author. Do not force-push, change Git history, merge an unrelated history, or push anything unless the user explicitly authorizes it.

## 1. Safeguards before touching code

1. Work **only on a new feature branch based on the restored `origin/main`**. Confirm `git log -1 --oneline` and `git status` first. The restored history must remain intact. Do not reset, clean, overwrite or delete the old working folder to prepare this project; the user said they will replace it themselves after backups.
2. The previous GP4 implementation was committed locally as **`487498d` (`change table arm`)**, on the old single-commit history. A safe reference branch was named `gp4-work-backup` and may be on the GitHub remote. **Use that commit only as a source of individual files; do not merge or cherry-pick its whole commit into the restored history.** It also contains accidental `.DS_Store` and `__pycache__` files.
3. Confirm the GP4 reference exists, either by copying it from the user's safe backup or taking only these paths from `gp4-work-backup`/`487498d`: `GP4_reference/`, `gp4_robot.py`, and the existing `tableArmGP4.py` **as source material**. If those files cannot be found, stop and ask for them rather than inventing robot geometry.
4. Before edits, inspect the restored `main.py`, `tableArm.py`, `sinkArmDatis.py`, `stoveArmLouis.py` and the existing JAKA asset directories. Preserve teammates' work and the house, bench, GUI, milk cartons, tea selection and stove logic.
5. Do not commit compiled bytecode, `.DS_Store`, virtual environments, or local IDE state. Add suitable patterns to `.gitignore` if needed.

## 2. Source implementation from the prior GP4 iteration

These details are grounded in the actual GP4 model/controller and earlier main files, not a hypothetical design:

- `gp4_robot.py` contains the independently defined `GP4Robot` six-axis model, with real GP4 STL collision meshes loaded from `GP4_reference/meshes/gp4/collision/`, `frames(q)`, `fkine(q)`, bounded `ikine_LM(...)`, joint limits, and `add_to_env(env)`.
- The GP4 model uses offsets (metres): `[[0,0,.330],[0,0,0],[0,0,.260],[.290,0,.015],[0,0,0],[0,0,0]]`; axes `[[0,0,1],[0,1,0],[0,-1,0],[-1,0,0],[0,-1,0],[-1,0,0]]`; joint limits (degrees) `[-170,170],[-110,130],[-65,200],[-200,200],[-123,123],[-455,455]`; and a **72 mm +X flange offset** on link 6. Retain these as found in the model; verify against `GP4_reference/urdf/gp4_macro.xacro` if changing kinematics.
- Existing `tableArmGP4.py` was adapted from the prior table-arm controller. It includes waypoint/IK planning, cup attachment/release, obstacle bounding boxes, carried-cup collision checking, sink-arm clearance checking, and diagnostic messages. Reuse this proven source rather than rewriting from scratch, but do not assume all moves were successful.
- It already has the crucial GP4 moving-link collision check: for a `GP4Robot`, `armPoints(arm,q)` calculates world mesh points using `frames(q)` and **omits the fixed base mesh** (`arm.meshPoints[1:]`, `frames[1:]`) from moving-link collision testing. Do not reintroduce false base/bench collisions.
- Original production `main.py` receives the sink result as `sinkRobot, cupFilled`, and should invoke the table arm **after** Datis's sink arm finishes, passing `env`, `teaCupParts`, `cupFilled`, `cupHeight=teaCupSizeZ`, `cupWidth=teaCupSizeX`, the GP4 mount pose, placement base pose, and `sinkArm=sinkRobot`.

## 3. Required project structure

Keep the original repo assets and teammate modules. Target project shape:

```text
IndustrialRobotics/
├── main.py                  # ONLY active workcell entry point
├── tableArm.py              # ONLY active table-arm controller, now GP4
├── gp4_robot.py             # GP4 model, FK/IK, mesh linkage
├── GP4_reference/           # Real GP4 URDF/Xacro and meshes
├── sinkArmDatis.py          # Existing sink arm, not replaced
├── stoveArmLouis.py         # Existing stove arm logic, preserve
├── JakaZu5/                 # Keep existing assets
├── JakaMiniCobo/            # Keep existing assets
└── .gitignore
```

`preview_GP4.py` can be used outside production for diagnostics if already available, but **do not create alternate mains or maintain `tableArmGP4.py` as a second active implementation**. Once its contents are integrated and tested, do not retain or import the duplicate controller in the final production layout unless the user specifically requests a transitional copy.

### Integration work

1. Adapt the existing `tableArm.py` in place using the working logic from `tableArmGP4.py`. Its public entry point should remain `runTableArm(...)` (or adjust minimally to match existing naming), and it should instantiate `GP4Robot` from `gp4_robot.py` rather than the prior table-arm model.
2. In `main.py`, import **`runTableArm` from `tableArm`**, not `tableArmGP4`. Retain the sink-arm call and pass its resulting filled cup to the GP4 table controller.
3. Preserve the cup geometry, tracking and animation in Swift (`env.step(dt)` and updated link mesh transforms). Avoid duplicating the cup or accidentally leaving it attached after release.
4. Keep the stove arm disabled if the restored project currently has it disabled. Do not modify its motion, model or coordinates as part of this GP4 task.
5. Keep GP4 mesh loading relative to the module path (`Path(__file__).resolve().parent`) so the production code runs from VS Code without relying on absolute user-machine paths.

## 4. Geometry, pickup and placement — read carefully

The **sink arm's original handoff is intentionally preserved** at:

```python
cup_handoff_world = (1.12, -1.32, 1.10)
```

Do not change Datis's sink handoff to the experimental `(1.12,-1.52,1.10)` from an earlier test. The sink arm must finish its existing pickup/fill/handoff routine first.

Known test positions from our session:

| Mount (XYZ, metres) | Result/meaning |
|---|---|
| `(0.65, -2.00, 1.55)` | Original GP4 back-wall mount, nominally out of reach of the handoff |
| `(0.80, -1.62, 1.10)` | Forward-of-wall test: **GP4 successfully planned the low cup pickup and a clearance move**, but could not find a safe drop-off; base appeared in front of the backsplash |
| `(0.70, -2.00, 1.00)` | Latest requested **lower, slightly left and flush-to-wall** visual placement; straight-line distance to handoff ≈ 0.81 m, **beyond GP4's ~0.55 m nominal reach**; not a verified working pickup |

The world back wall is near `Y=-2.00`. The world bench top is `Z=benchSizeZ`, approximately `0.90 m`.

**Important engineering constraint:** Mounting the GP4 flush on that back wall while leaving the handoff at `(1.12,-1.32,1.10)` cannot be made to work simply by lowering the robot or shifting it sideways. The Y separation alone is **0.68 m**, greater than the GP4's nominal 0.55 m reach. Do not fabricate successful IK or disable collisions to conceal this.

**User preference:** GP4 should visually be against the wall, a little lower and a little left, take the cup from the unchanged sink-arm handoff, and release it on the **stove-side/right as seen from the intended viewing angle**. Existing fallback drop positions had world X values around `0.18–0.55`, with preferred cup placement base previously `SE3(0.4,-1.35,benchSizeZ)`. **Do not assume a lower world X is mechanically the robot's right; verify the viewpoint/world axes.**

Copilot must handle the conflicting physical requirements explicitly: test the user-requested wall pose faithfully and print a useful unreachable-target message; explain that a bracket/forward mount, relocation of the handoff, or a longer-reach robot will be required for a real transfer. **Do not silently move the sink arm, scale the GP4 geometry, invent a longer robot, or present the wall version as a successful full cycle.** If experimenting with a functional bracket-mounted layout, get the user's approval before changing from the flush-wall requirement.

## 5. What worked and what still failed

At the forward-mount GP4 test `(0.80,-1.62,1.10)`, the observed terminal output included:

```text
Sink arm: cup ready for handoff at [ 1.12 -1.32  1.1 ]
Table arm: low pickup planned, yaw -90 deg, cup height 0.38, approach (-0.16, +0.16, +0.00)
Table arm: clearance move found to [ 1.05 -1.34  1.12]
Table arm: contact placement blocked; checking safe above-bench release
Table arm: no safe bench release found; release approach target: arm/environment, release approach arm/environment, release approach cup tilt
```

Thus **pickup/initial clearance planning succeeded**, but *cup placement did not*. Candidate positions around X=0.18–0.40 were rejected for inverse kinematics, arm/environment collision, cup/sink collision, or cup tilt. The collision checker also printed `tap spout not recognised; standard obstacle collision checks remain active` — this is diagnostic, not evidence the spout collision constraints can be removed.

When working on placement, keep all collision checks and debug reasons. Test the whole path and every moving GP4 link, plus the carried cup against the bench, sink robot, wall, cartons and stove. Do not simply teleport/release the cup in space to mark success. Test contact placement on the bench at a physically valid supported height. Report any unsolved target honestly.

## 6. Requested implementation sequence for Copilot

1. **Inventory** restored repository content and compare it to the GP4 source branch/files. Describe intended edits briefly before coding.
2. **Import only** `GP4_reference/` and `gp4_robot.py` from the GP4 work. Validate reference meshes are present. Keep all existing robot packs.
3. **Replace/adapt the active table-arm logic in `tableArm.py`** using the source `tableArmGP4.py` (maintain relevant API contracts with the existing workcell).
4. **Edit `main.py` directly** to instantiate and run GP4 through `tableArm.py` after the unchanged sink handoff. No `main_GP4.py` or other standalone mains in the final production code.
5. Start with the requested flush-wall mount `(0.70,-2.00,1.00) * SE3.Rx(-pi/2)` and unchanged sink handoff. Verify and report the unreachable geometry. Do **not** claim full operation on this pose.
6. Where practical, make mounting pose and drop-off position simple named constants at the top of the relevant code or near the workcell setup, rather than scattering hard-coded coordinates.
7. Run syntax checks, independently test GP4 FK/IK, and run `main.py` through Swift. Record exactly which stages succeed: spawn, arm motion, sink handoff, GP4 pickup, transport, drop-off, retreat. Confirm GUI and other robots still appear.
8. Provide a **concise list of changed files**, test logs, known limitations, and the exact command for running `main.py`. Do not commit or push without user approval. Do not add co-author trailers.

## 7. Acceptance conditions

- [ ] `main.py` is the only normal workcell entry point, running the same application with GP4 as the table arm.
- [ ] `tableArm.py` is the active GP4 controller; it imports `GP4Robot` and doesn't depend on `tableArmGP4.py`.
- [ ] `gp4_robot.py` and the real GP4 reference meshes are present.
- [ ] The existing sink handoff is exactly `(1.12,-1.32,1.10)`; no changes to Datis's choreography.
- [ ] Louis's stove work and JAKA assets are preserved.
- [ ] GP4 mounting pose is visibly correct for the user's preferred wall placement, or any necessary compromise is explicitly discussed rather than hidden.
- [ ] Failed IK or collision plans are printed as failures, not bypassed.
- [ ] No accidental `.DS_Store` or `.pyc` files are committed.
- [ ] The restored 25-commit ancestry is preserved, and nothing is pushed or co-authored without permission.

## 8. Safe Git reminder for the future

Work from restored `main` on a new branch. **Don't run `git pull` into the old one-commit local `main`** and do not merge/cherry-pick commit `487498d` wholesale. Import its necessary files as content. Never use `git reset --hard`, `git clean`, or history rewrites to solve a file-conflict problem when user work may be uncommitted.

**Practical source locations:** the old saved local folder was `~/Downloads/IndustrialRobotics-main-2-SAFE-BACKUP-20261013`, and another source is the Git reference `gp4-work-backup` at commit `487498d` if still available. The refreshed/restored repository may need the files copied or `git restore --source=... -- GP4_reference gp4_robot.py` from that source reference; only do so after confirming it exists. No external downloading is necessary if the GP4 assets can be retrieved from the backup.
