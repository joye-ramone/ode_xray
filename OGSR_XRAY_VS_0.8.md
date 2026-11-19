# Three-way comparison: `0.5.0` → `xray` versus `0.5.0` → `0.8`

Companion to [OGSR_CHANGES.md](OGSR_CHANGES.md). The question: which `xray` changes are still relevant on the `0.8` branch, and can they be moved there?

## 1. Setup

| Ref | Commit | Note |
|---|---|---|
| base `heads/0.5.0` | `7bac210f` (2004-05-29) | The merge base of both lines. `0.5.0` is also a tag at the same commit. |
| `xray` | `3276c2d6` | 5 snapshot commits, 38 files changed. |
| `heads/0.8` | `08d4cad7` | 386 upstream commits after the base. The only commit after the `0.8` tag (`8272c68f`) is "Close branch 0.8", which changes no content. `0.8` is ambiguous (a tag and a branch), so use `heads/0.8`. |

I also checked how the engine consumes ODE, because that decides what is *needed* and not merely *applicable* (`F:\games\OGSR\OGSR_Engine_private`):
- `3rd_party/Src/ode/ode/src` and `include` are **byte-identical** to the `xray` branch.
- It is built as a **static library** from `contrib/msvc7/ode_default/default.vcxproj`, with `dSINGLE; MSVC; dNODEBUG`. The trimesh/OPCODE sources are **not** compiled. `msvcdefs.def` is referenced only by the DLL configurations.
- `xrPhysics` includes ODE internals directly (`ode/src/objects.h`, `joint.h`, `util.h`, `collision_kernel.h`, `collision_std.h`).
- **`class CPHIsland : public dxWorld`** (`PHIsland.h:49`): every physics island *is* a `dxWorld`. It is never created with `dWorldCreate`, and it manages bodies and joints through `dWorldAddBody/RemoveBody/AddJoint/RemoveJoint`.
- Stepping: `dWorldStep` is used when an island has fewer than 30 joints (`max_joint_allowed_for_exeact_integration`); otherwise `dWorldQuickStep`. **`dWorldStepFast1` is never called.**
- Contacts are created with a **null world** (`dJointCreateContact(nullptr, …)`, `dJointCreateContactSpecial(nullptr, …)`), then attached to an island with `ConnectJoint` → `dWorldAddJoint`. `CPHIsland::Unmerge()` later drops them by cutting its own joint list.
- Direct struct access: `body->pos`, `body->R`, `facc`, `tacc`, `next`, `tome`, `flags`; custom geom classes (`dCylinder`, `dTriList`, `dRayMotions`) that touch `dxGeom` internals; direct calls to `dxStepBody`; `dSetAllocHandler`.

### What 0.8 changed structurally (relevant to porting)

| Area | 0.5 / xray | 0.8 |
|---|---|---|
| Collision sources | one `collision_std.cpp` | split into `box.cpp`, `sphere.cpp`, `capsule.cpp`, `plane.cpp`, `ray.cpp`, `cylinder.cpp`, `convex.cpp`, `heightfield.cpp`, … (`collision_std.cpp` deleted) |
| Geom / body transform | `g->pos`, `g->R`, `b->pos`, `b->R` | `g->final_posr->pos/R` with `recomputePosr()` (geom offsets), `b->posr.pos/R` |
| CCylinder | `dCCylinder*` | renamed `dCapsule*` (compatibility `#define`s exist); a real `dCylinder` class was added |
| Auto-disable | simple thresholds | average-velocity sampling; `dxBody` gained buffers that `dBodyCreate` allocates **from the world's settings** |
| Build / export | `config.h` from the configurator plus `msvcdefs.def` | premake, `build/config-default.h`, `ODE_API` decoration; **no `.def`** |
| Init | none | `dInitODE()` / `dCloseODE()` required |
| API style | `dxJoint*` in signatures | `dJointID` in signatures, cast inside |

A trial `git merge-tree heads/0.8 xray` produced **19 conflicting files**: 2 modify/delete conflicts (`collision_std.cpp`, `msvcdefs.def`) and 17 content conflicts. `ode.cpp` has 13 hunks, `quickstep.cpp` 7, and `joint.cpp`, `timer.cpp` and `include/ode/objects.h` 3 to 4 each. **A branch merge is not the right tool.** The `xray` commits are snapshots, not logical changes, so the port has to be a hand-made patch series. It is feasible, and §4 lays out the plan.

---

## 2. Status of every xray change on 0.8

**Status** says where 0.8 stands. **Port** says what to do and how hard it is:
- *clean*: the same code exists in 0.8 and the patch applies with at most a path change.
- *adapt*: needs renames (`posr`, `ODE_API`, `dJointID`).
- *rework*: needs redesign on top of 0.8.
- *skip*: don't port.

### 2.1 Generic bug fixes (useful to any ODE user)

| # | xray change | Status in 0.8 | Port | Notes |
|---|---|---|---|---|
| F1 | Quadtree `remove()` drops **all** duplicate dirty entries | ✅ **already in 0.8**, identical code and the same "(mg)" comment | skip | xray took this from upstream. |
| F2 | Quadtree `pow((dReal)SPLITS, i)` | ✅ already in 0.8 | skip | |
| F3 | `dJointAddAMotorTorques` axis components | ✅ already in 0.8 ("Important AMotors bugfix", 12/07/04) | skip | |
| F4 | AMotor `angle[]` index clamp | ❌ **still wrong in 0.8** (`anum > 3 → 3` writes out of bounds) | clean, with the correct form | Port as `anum < 3` in the assert and a clamp to `2`. **Don't** port xray's `anum < 2` assert, which rejects the valid axis 2. |
| F5 | `dInternalStepIsland_x2`: check `j1/j2 == -1` **before** `ofs[j1]` | ❌ **still buggy in 0.8** (`step.cpp:1380-1388`) | clean | The block is identical; move the 3 lines. This path is **live in OGSR** through `dWorldStep`. |
| F6 | `stepfast` `MultiplyAdd2_sym_p8p` adds the diagonal twice | ❌ **still buggy in 0.8** (`stepfast.cpp:115-124`) | clean | A genuine upstream bug. It doesn't matter to OGSR, which never uses step-fast. |
| F7 | `testing.cpp` missing `va_end` | ❌ still missing | clean | Trivial. |
| F8 | `char*` → `const char*` (`misc.h`, `mat.h`, `timer.cpp`, `testing.*`) | ❌ still `char*` | clean | Needed for `/permissive-` and C++17 builds. |
| F9 | `timer.cpp` `WIN32` → `_WIN32` | ❌ 0.8 still tests `WIN32` | adapt | 0.8 already handles x86-64 asm through `X86_64_SYSTEM`, which is **better** than xray's `_M_X64` branch (dead on GCC). Port only the `_WIN32` macro, not xray's asm. OGSR defines `WIN32` itself, so this is cosmetic. |
| F10 | `dGeomRaySet` builds an orthonormal `R` (`dRFromZAxis`) | ⚠️ 0.8 normalises the direction but still writes only the Z column | adapt | Use the public `dGeomSetPosition/Rotation` (as xray does); it works with `final_posr`. |
| F11 | `collision_trimesh_internal.h`: remove `using namespace Opcode` | ❌ still present | skip for OGSR | Trimesh isn't built in OGSR. In 0.8, removing it touches many trimesh files. |
| F12 | `error.cpp` `_snprintf`, `_cdecl` | ✅ 0.8 already maps `_snprintf↔snprintf` per platform; `ODE_API` covers the calling convention | skip | |
| F13 | `odemath.cpp` `dCopySign` → `std::copysign` | n/a | skip | Equivalent. |
| F14 | `REAL(-1.0)` → `-1.0` in the ray code | n/a | skip | A regression in literal type. |
| F15 | `dCollideRayCCylinder`: type assert commented out | obsolete | skip | 0.8 has `dCollideRayCapsule` **and** `dCollideRayCylinder`. OGSR has its own `dCylinder` class with its own `dCollideCylRay`. |

### 2.2 X-Ray features (needed by the engine)

| # | xray change | Status in 0.8 | Port | Needed by OGSR? |
|---|---|---|---|---|
| X1 | `dWorldAddBody/RemoveBody/AddJoint/RemoveJoint`; bodies and joints creatable with `world == 0` | ❌ absent; `createJoint`/`dJointInit` assert `w` | **adapt**: 0.8 `dBodyCreate` calls `dBodySetAutoDisableDefaults`, which reads `w->adis` and allocates the average buffers, so it needs a `w == 0` path (default adis, no buffers) | **Yes**: `CPHIsland` is built on it |
| X2 | `static` world parameters in `dxWorld` | ❌ per-world in 0.8 | adapt: same edit to `objects.h` plus the static definitions in `ode.cpp` | **Yes**: every `CPHIsland` is a `dxWorld`, and the global `phWorld` settings (gravity, ERP, CFM in `PHWorld.cpp`) must reach all of them. Keep `sizeof(dxWorld)`/layout in mind, because the engine inherits it. |
| X3 | `dJointAttach` without the same-world check | ❌ check present | adapt: relax to `(!world \|\| !b->world \|\| b->world == world)` instead of deleting it | **Yes**: joints and contacts are worldless |
| X4 | Asserts removed from the `dWorldSet*`/`Get*` functions | n/a | skip or adapt | Needed only if the engine calls them with a null world; with static params, prefer to keep the asserts. |
| X5 | `dJointGroupEmpty` doesn't unlink from the world | ❌ absent (0.8 unlinks any joint whose `world` is set) | **adapt** | **Yes**: contacts keep their island `world` pointer, and `CPHIsland::Unmerge()` has already cut them from the island's list, so stock unlinking would corrupt it. (Corrected: an earlier version said to skip this.) |
| X6 | `dxBodyNoUpdatePos` and `dBodySet/GetNoUpdatePosMode` | ❌ absent | adapt (the flag value `32` is free in 0.8) | **Yes**: `CPHElement::Fix()`. **Fix while porting**: the getter is inverted, and the check is missing in `dInternalStepIsland_x2` (`step.cpp:1674` in 0.8), which is the path `dWorldStep` actually runs. |
| X7 | `dJointCreateContactSpecial` (normal force capped at 1e5) | ❌ absent | clean (the `Vtable` layout is unchanged in 0.8) | **Yes**: actor and camera contacts |
| X8 | `dJointSetFixedQuaternionPos` | ❌ absent | adapt (`dJointID` signature, `ODE_API`) | **Yes** (1 call site). Add an assert for the unsupported two-body case. |
| X9 | `dxProcessIslands` → one island, all bodies re-enabled | ❌ 0.8 has the island search plus the new auto-disable | rework: replace the function body and keep `dInternalHandleAutoDisabling` out | **Yes**: the engine does its own islanding. **Fix while porting**: `return` → `continue` for unattached joints. |
| X10 | Per-body auto-disable removed (fields stubbed) | 0.8 extended auto-disable | **skip**: leave 0.8's auto-disable intact and just default it off (`adis_flag = 0`) | No. The engine never enables ODE auto-disable, and stripping fields from `dxBody` in 0.8 would touch much more code. |
| X11 | `extern "C" dxStepBody` plus export | ❌ | adapt | **Yes**: called directly from `PHElement`, `PHCharacter` and `PHActivationShape`. For a static lib with `util.h` included, linkage matches either way. |
| X12 | `dValid()` and zeroing of NaN/Inf `cforce` in `step.cpp`/`quickstep.cpp` | ❌ absent | adapt: use `dReal&` (not `float&`); `MSVC` is defined by the OGSR project, so the `_fpclass` branch is live | Recommended (stability) |
| X13 | `dWorldCreate` defaults `qs.w = 1.1`, `min_depth = 0.001` | 0.8: 1.3 and 0 | clean | **Yes**: `PHWorld.cpp` sets gravity, ERP and CFM but not these two |
| X14 | Inline fast `dNormalize3` + `dNormalize3_slow` | 0.8 still has only the robust out-of-line `dNormalize3` (the old version is kept in a comment) | adapt (optional) | Performance only. Wrap `__forceinline` in `_MSC_VER`. |
| X15 | QuickStep: no copy of the joint array | 0.8 copies | clean (optional) | Micro-optimisation; requires the X9 caller semantics. |
| X16 | QuickStep: `#define WARM_STARTING 0` | ✅ **0.8 already has warm starting disabled** (`//#define WARM_STARTING 1`) | **skip** | 0.8 achieves what xray intended but didn't. |
| X17 | QuickStep: `std::shuffle` + `random_device`, reshuffle every 4 iterations | 0.8 uses `dRandInt` (int-only, deterministic) | **skip** | Non-deterministic. If a 4-iteration period is wanted, change only the mask. |
| X18 | QuickStep `isnan(lo)` half-guard | — | skip | Covered by X12. |
| X19 | `dBoxBox` extra edge-edge contacts | 0.8 `box.cpp` `dBoxBox` is **identical** to 0.5 | clean (a path change to `box.cpp`), **with fixes** | **Yes** (box stability). Before porting: honour `maxc`, fix `p2`→`p1` in the `pa2/pa3` probes, and guard the `_cos` division (OGSR_CHANGES #1/#2). |
| X20 | `Bounder33` + `StepJointInternal` (3×3 contact LCP) | ❌ absent | clean (new files; `Info2` layout unchanged) | **No**: only reachable from `dWorldStepFast1`, which OGSR never calls. Dead code today. |
| X21 | Other step-fast changes (`NO_ISLANDS`, ≤3-joint routing, `SwapJoints`, no-gravity `facc` branch removed, `rand()`) | ❌ | skip | Unused by OGSR, and two of them are bugs (OGSR_CHANGES #5/#6). Port only F6. |
| X22 | GeomGroup compatibility API | removed upstream on purpose | skip | Engine uses it only in commented-out code. |
| X23 | Hand-written `include/ode/config.h` and the `.def` exports | 0.8 uses `build/config-default.h` and `ODE_API` | rework | For a static lib, copy `config-default.h` to `include/ode/config.h` (`dSINGLE`, trimesh off) and add `ODE_API` to the new declarations. |
| X24 | `lcp`, `testing`, `export-dif`, `dTimerReport` under `#if 0` | — | skip | Size cosmetics only. |
| X25 | Dead `USE_*_MIN_ERR` / `NOISING` code in `joint.cpp`/`stepfast.cpp` | — | skip | |

---

## 3. What 0.8 would bring that xray lacks

Worth having even if you stay on the xray line:
- AMotor fixes (the torque fix is in xray; the angle clamp needs F4).
- The int-only `dRandInt` (deterministic, no double rounding).
- Geom offsets, real cylinder collisions (cylinder-box, cylinder-sphere, cylinder-plane, ray-cylinder), heightfield, convex, the Plane2D and PR joints, `dJointGetUniversalAngles`.
- Better auto-disable ("objects won't sleep in the air"). OGSR doesn't use it.
- The 64-bit OPCODE segfault fix. That fix is only in the trimesh code, which OGSR doesn't build.

For OGSR itself, **none of these is a must-have**. The engine has its own cylinder, trimesh collider, islanding and sleeping logic.

---

## 4. Feasibility and recommended path

### Option A: stay on `xray` (0.5 base) and cherry-pick (recommended)
Low risk, a small diff, and no engine changes.
1. Fix the xray bugs from OGSR_CHANGES.md that are live in OGSR: `dBoxBox` (#1/#2), `dxProcessIslands` `return`→`continue` (#3), the `NoUpdatePos` getter and the missing check in `step.cpp` `_x2` (#4/#7b), and `WARM_STARTING` (#7). Use `#undef` or remove the define to match 0.8, **after** checking in-game that stacking and ragdolls don't get worse. Today warm starting is effectively on.
2. Take from upstream: F4 (AMotor clamp) and F7/F8 (hygiene). F5 is already in xray.
3. Optionally apply F6 and delete X20/X21 (dead in OGSR), or leave step-fast alone.

### Option B: move OGSR to `0.8` and replay the X-Ray patch series
Feasible, but it is a **porting project, not a merge**. Suggested patch series on top of `heads/0.8`:

| Step | Content | Effort |
|---|---|---|
| 1 | Build: static-lib vcxproj, `config.h` from `config-default.h` (`dSINGLE`, no trimesh), `dInitODE()` call in the engine | S |
| 2 | Upstream fixes F4, F5, F6, F7, F8 | S |
| 3 | X1 + X2 + X3: worldless objects, add/remove API, static world params | M (13 conflict hunks in `ode.cpp`; the null-world path in the 0.8 auto-disable init) |
| 4 | X6 (fixed), X7, X8, X11, X13 | S |
| 5 | X9 single-island `dxProcessIslands` (fixed) | S |
| 6 | X12 NaN guards (with `dReal&`) | S |
| 7 | X19 box-box edge contacts (fixed) into `box.cpp` | S |
| 8 | Optional: X14 fast `dNormalize3`, X15 | S |

**The engine side is the expensive part.** xrPhysics uses ODE internals that changed in 0.8:
- `dxBody::pos/R` → `posr.pos/R` (for example `PHElement.cpp:1494`, `PHFracture.cpp:467-474`, `Physics.cpp:351-356`).
- `dxGeom::pos/R/aabb` → `final_posr` + `recomputePosr()` in the custom geom classes (`dcylinder/dCylinder.cpp`, `tri-colliderknoopc/*`, `dRayMotions.cpp`, `ExtendedGeom.h`, `PHMoveStorage.cpp`). There are about 130 `->pos/R/aabb/…` accesses in xrPhysics to audit; most refer to `dContactGeom::pos`, which is unchanged.
- `CPHIsland : public dxWorld`, plus its hand-managed body list (`next`/`tome`), must match 0.8's `dxWorld`. The layout is unchanged except for the statics.
- `dCCylinder` → `dCapsule`, 17 references. The compatibility macros cover the public API, not internal class names.
- `collision_std.h` still declares `dCollideRayBox/Sphere` in 0.8, so `dRayMotions.cpp` keeps working.

Estimate: ODE side 1-2 days; engine side 2-4 days, plus a **physics regression pass**. 0.8 changes the solver defaults and the geom update order, so ragdolls, vehicles, stacking and the actor controller need in-game testing.

### Verdict
- **Moving the fixes is possible.** The generic fixes F4-F8 are trivial on 0.8. The X-Ray features X1-X3, X5-X9, X11-X13 and X19 all port with moderate adaptation. X10, X16, X17 and X20-X22 should **not** be ported: 0.8 already covers them, they are unused by OGSR, or they are buggy.
- **Should OGSR switch?** Not unless there is a concrete need for a 0.8 feature. OGSR doesn't use any of what 0.8 adds (it has its own cylinder, trimesh and islanding), while the switch forces engine changes and a full physics retest. Option A gets every applicable correctness fix for a fraction of the cost.
