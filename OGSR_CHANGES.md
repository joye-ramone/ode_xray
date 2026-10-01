# ODE `xray` branch review: changes compared with `0.5.0`

## Scope

| | |
|---|---|
| Base | `0.5.0` (the branch and the tag both point at `7bac210f` "minor speed improvement"; use `heads/0.5.0` because the name is ambiguous) |
| Head | `xray` @ `3276c2d6` |
| Commits | 5 ("update to xray version" 1 to 5, JR, 2026-09-14) |
| Net diff | 38 files, +2365 / -918 (much of it is whitespace or re-indentation; `git diff -w` is ~25% smaller) |

The branch ports the GSC/X-Ray-modified ODE (the physics engine in S.T.A.L.K.E.R.) on top of stock ODE 0.5. The commits are snapshots, not logical changes. Several things added in commit 1 were removed again in later commits: `contrib/msvc7/ode_default/*` was dropped in v3, the `addBodiesForces_fn` typedef, the "Collider exceed contact buffer" message and about 290 lines of an alternative `SOR_LCP` were dropped in v5, and commented-out code in `joint.cpp` was cleaned up in v4. This review covers the **net** result only.

### Verdict legend

- ✅ **Good**: a correct fix or a harmless improvement.
- ➖ **Neutral**: cosmetic, or a deliberate X-Ray behaviour change that is fine inside the game.
- ⚠️ **Caveat**: works for X-Ray's usage, but changes ODE semantics or is fragile.
- ❌ **Bug**: incorrect code or a real crash or corruption risk.

### How OGSR uses this library (checked in `OGSR_Engine_private` after the first pass)
The engine's copy (`3rd_party/Src/ode`) is byte-identical to `xray`. It is built as a static lib with `dSINGLE; MSVC; dNODEBUG` (asserts are compiled out) and without the trimesh sources. That changes the practical weight of several findings:
- `class CPHIsland : public dxWorld`: each island *is* a world, so the **`static` world params (#9) are required by the design**, not merely a caveat.
- Contacts are created with a null world but are then attached to an island (`ConnectJoint` → `dWorldAddJoint`). `CPHIsland::Unmerge()` drops them by cutting its own joint list, *before* `dJointGroupEmpty` runs. **The xray `dJointGroupEmpty` change (#8) is therefore required**: stock ODE would unlink joints from a list the engine has already rebuilt. (An earlier version of this note said the change was unnecessary; that was wrong.)
- Stepping uses `dWorldStep` (islands with < 30 joints) and `dWorldQuickStep`. **`dWorldStepFast1` is never called**, so `stepfast.cpp`, `Bounder33` and `StepJointInternal` are dead code in OGSR, and #5, #6, #11 and #12 have no runtime effect there.
- **#7b is live**: `CPHElement::Fix()` relies on `NoUpdatePos`, but `dWorldStep` → `_x2` ignores it.
- The main `dCollide` call passes 810 contacts, so #1 doesn't overflow on that path. It is still a library bug for any small-buffer caller.

See [OGSR_XRAY_VS_0.8.md](OGSR_XRAY_VS_0.8.md) for the three-way comparison with the `0.8` branch.

---

## 1. Summary of findings (by priority)

| # | Sev | Location | Issue |
|---|---|---|---|
| 1 | ❌ | `collision_std.cpp:882-1041` (`dBoxBox` edge-edge) | Can write up to **7 contacts while ignoring `maxc`**. This overflows the caller's contact buffer, for example with `dCollide(...,flags=1,...)`. |
| 2 | ❌ | `collision_std.cpp:1022,1032` | The `pa2`/`pa3` probes pass **`p2` (box 2's centre)** together with `R1`/`side1`. It should be `p1` (copy-paste error). |
| 3 | ❌ | `util.cpp:307` (`dxProcessIslands`) | `if (!joint->node[0].body) return;` **skips the whole world step** when one joint is unattached. It should be `continue`. |
| 4 | ❌ | `ode.cpp:807` | `dBodyGetNoUpdatePosMode` returns the **inverted** value (copy-pasted from the gravity getter). |
| 5 | ❌ | `stepfast.cpp:715` | The `else` branch that restores `facc` for **no-gravity bodies** was removed. Their force accumulates across `maxiterations` sub-steps. |
| 6 | ❌ | `stepfast.cpp:529-535,734` | The joint "shuffle" is broken: `SwapJoints` ignores its first argument and rolls its own `rand()`, and the caller has already passed a random index. The order is biased and `rand()` runs twice. |
| 7 | ❌ | `quickstep.cpp:53` | `#define WARM_STARTING 0` does **not** disable warm starting, because every use is `#ifdef WARM_STARTING`. |
| 7b | ❌ | `step.cpp:948`, `stepfast.cpp:837` | `dxBodyNoUpdatePos` is ignored by `dWorldStep` (the live `_x2` path) and by step-fast. Only QuickStep and the dead `_x1` path check it. |
| 8 | ⚠️ | `ode.cpp:1056` (`dJointGroupEmpty`) | Grouped joints are **no longer unlinked from the world**. This is safe only if every grouped joint is created with `world == 0`; otherwise the world's joint list dangles after the group is emptied. |
| 9 | ⚠️ | `objects.h:116-122` | All `dxWorld` parameters (gravity, ERP, CFM, auto-disable, quickstep, contact params) are now **`static`**, so every world shares them. |
| 10 | ⚠️ | `quickstep.cpp:724`, `step.cpp:932` | `float &lf = cforce[...]` hard-codes `float`, so the build **fails with `dDOUBLE`**. |
| 11 | ⚠️ | `Bounder33.cpp:245` | `Lcp33::ToNBn(int)` swaps with `NBn` *before* decrementing it (off by one versus `ToBn`). After a failed trial, the next trial solves for the wrong index. |
| 12 | ⚠️ | `Bounder33.cpp:286` | `float friction` should be `dReal`. There is no guard for `A[0]==0`, and `hi *= friction` gives NaN when `hi==inf` and `friction==0`. |
| 13 | ⚠️ | `quickstep.cpp:314` | `std::random_device`-seeded shuffle, so the simulation is **no longer deterministic** from run to run (previously it used `dRandInt`, which is seedable). |
| 14 | ⚠️ | `joint.cpp:2505,2583` | The AMotor angle index fix over-corrects: `anum < 2` rejects the valid index 2 (`angle[3]`). It should be `anum < 3` with a clamp to 2. |
| 15 | ⚠️ | build | `Bounder33.cpp` and `StepJointInternal.cpp` are not added to `Makefile`. Only the external (OGSR) project builds them. |

Items 1 to 7b should be fixed. Items 8 to 15 should be fixed or at least documented as X-Ray contracts.

---

## 2. Build / configuration / headers

### `include/ode/config.h` (new, 209 lines)
Stock ODE 0.5 generates this header with `configurator.c`. The branch now commits a hand-written one based on newer ODE (fixed-width `dint*` types, `dInfinity`, `EFFICIENT_ALIGNMENT 16`).
- ✅ Committing it makes the MSVC build reproducible without running the configurator.
- ⚠️ It does **not** define `dSINGLE`/`dDOUBLE`, so the consumer project must define one (OGSR does). Add a comment saying so.
- ⚠️ `#include <malloc.h>` is not portable (macOS has no such header). `SHAREDLIBIMPORT/EXPORT` are unconditional `__declspec`, which is MSVC-only. `MMAP_ANONYMOUS` is defined unconditionally.
- ⚠️ `typedef uintptr_t intP;` relies on `<stdint.h>`, but only the non-x86 branch includes it. It works on MSVC through `malloc.h` → `crtdefs.h`, but only by accident.
- ➖ The comment blocks contain stray blank lines between every line, which looks like a CRLF conversion artefact. Clean them up.

### `config/msvcdefs.def`
- ✅ Adds exports for the new X-Ray API: `dWorldAdd/RemoveBody/Joint`, `dBodySet/GetNoUpdatePosMode`, `dJointCreateContactSpecial`, `dJointSetFixedQuaternionPos`, `dGeomMoved`, `dCollideRay*`, `dSet*Handler`, `dxStepBody`, and `dNormalize3_slow`.
- ✅ Consistently removes `dError`/`dDebug`/`dMessage`, `dTest*` and `dWorldExportDIF`, which are now compiled out (see below).
- ⚠️ `dCreateGeomGroup`/`dGeomGroup*` (added in `collision_std.cpp`) are **neither declared in any header nor exported**. If X-Ray uses them, it declares them itself, which only works with static linking.

### `include/ode/error.h` / `error.cpp`
- ➖ `_cdecl` on `dError/dDebug/dMessage` is harmless on x86/x64 MSVC but is **not a keyword on GCC/Clang**, so non-MSVC builds of the header break. Prefer no annotation, or a macro.
- ➖ `extern "C"` was removed from the definitions. Linkage still comes from the header's `extern "C"` block, so this is OK.
- ➖ `snprintf` → `_snprintf` and `#pragma warning(disable:4996)` are MSVC-only. Modern MSVC has `snprintf`, which (unlike `_snprintf`) always null-terminates. The explicit `s[sizeof(s)-1]=0` covers that, so this is safe but was unnecessary.

### `include/ode/misc.h`, `mat.*`, `misc.cpp`, `testing.*`, `timer.cpp` (`char*` → `const char*`)
- ✅ Const-correctness for string literals, needed by modern C++ (`/permissive-`, C++11+).

### `include/ode/odemath.h`: inline `dNormalize3`
- ✅ A fast path (`dRecipSqrt` of the squared length) with a fall-back to the robust `dNormalize3_slow` for tiny vectors (|v|² < 1.19e-5). This is the classic X-Ray optimisation and it is correct.
- ⚠️ `__forceinline` sits in a C-compatible, `extern "C"` public header, so it is MSVC-only. For NaN input it takes the fast path and returns NaN; the original asserted. That is acceptable.

### `include/ode/objects.h`
- ✅ Declares the new public APIs. The whitespace alignment is cosmetic.

### `odecpp*.h`
- ➖ Trailing-whitespace removal only.

### `collision_std.h`
- ✅ `extern "C"` around the `dCollideRay*` prototypes, needed to export them through the `.def`.

### `collision_trimesh_internal.h`: removed `using namespace Opcode;`
- ✅ Good hygiene (no `using` in headers). Check that every trimesh `.cpp` still compiles, because they may have relied on it.

### `util.h`
- ✅ `dValid()` (NaN/Inf/denormal check) and `extern "C" dxStepBody` for export.
- ➖ `#ifdef MSVC` is not a predefined macro (MSVC defines `_MSC_VER`). The OGSR vcxproj defines `MSVC` explicitly, so the `_fpclass` branch is the one used there. Other builds fall back to the equivalent `std::fpclassify`. Keying it on `_MSC_VER` would be more robust.
- ⚠️ `dValid(const float)` together with the callers' `float&` breaks `dDOUBLE` (issue #10).

### `lcp.cpp`, `testing.cpp`, `export-dif.cpp`, `timer.cpp::dTimerReport` wrapped in `#if 0`
- ➖ These are dead code in a game build and shrink the binary.
- ⚠️ `dWorldExportDIF` and `dTimerReport` are still **declared** in the public headers, so any use becomes a link error. `util.cpp`/`stepfast.cpp` call `dTimerReport` under `#ifdef TIMING`, so enabling `TIMING` no longer links.
- ✅ `testing.cpp` adds the missing `va_end(ap)` (a real, if tiny, fix).

### `timer.cpp`
- ✅ `WIN32` → `_WIN32` (the correct predefined macro, which also covers x64), and the chain was restructured into `#if/#elif`. The macOS `mach_absolute_time` branch is taken from newer ODE.
- ❌ (latent) The "x64" Pentium asm branch keys on `_M_X64`, which only MSVC defines, but the code is GCC inline asm that GCC never selects. On LP64 Linux, `unsigned long cc[2]` is 16 bytes, while the asm stores two 32-bit halves at offsets 0 and 4. The branch only matters with `PENTIUM` defined, so it is practically dead. Consider deleting it.

### `odemath.cpp`
- ➖ `dCopySign` → `std::copysign`. Equivalent; `dNormalize3` was renamed to `dNormalize3_slow`.

### `collision_quadtreespace.cpp`
- ✅ `pow((dReal)SPLITS, i)` resolves an ambiguous overload in modern C++.
- ✅ `remove()` now deletes **all** duplicates of a geom from `DirtyList` (it used to stop at the first). This fixes a real dangling-pointer bug, and later ODE versions fixed it the same way.

### Removed `contrib/msvc7/ode_default/*`
- ➖ Added in v1 and removed in v3; the net effect is none.

---

## 3. Collision (`collision_std.cpp`)

### `dGeomRaySet`
- ✅ It now builds a full orthonormal rotation with `dRFromZAxis` and calls `dGeomSetPosition/Rotation`. Previously only the Z column of `R` was written, which left `R` non-orthonormal. This is correct (later ODE does the same) but a little more expensive.

### GeomGroup compatibility API (`dCreateGeomGroup`, `dGeomGroupAdd/Remove/GetNumGeoms/GetGeom/Query`)
- ➖ It emulates the pre-0.5 geom groups on top of a simple space with cleanup disabled. This is fine as a compatibility shim, but it has no header declaration (see §2).

### `dBoxBox` edge-edge contact generation (the largest functional change)
Stock ODE returns **one** contact for an edge-edge case. X-Ray adds up to six more by intersecting the contact edge with the adjacent faces of the other box (`CrossBoxSide44`). This gives much more stable box stacking and resting, which is why GSC added it.
- ❌ **Buffer overflow**: `maxc` is ignored in this branch, so `ret` can reach 7. `dCollideBoxBox` passes `flags & NUMC_MASK` as `maxc`, and callers that request 1 to 6 contacts get memory written past their buffer. Fix: `if (ret >= maxc) goto done;` before each additional contact (and never write into `CONTACT(contact, skip*ret)` unless `ret < maxc`).
- ❌ In the `pa2`/`pa3` probes (lines 1022 and 1032) the box-centre argument is `p2`, while the rotation and extents are box 1's (`R1`, `side1`). Compare with the `pa1` probe, which correctly passes `p1`. The result is contact points computed against the wrong face plane.
- ⚠️ `CrossBoxSide44` divides by `_cos = dot(R1 axis, R2 face normal)`, which can be ~0 for parallel edges and give Inf/NaN positions. Add a `dFabs(_cos) > eps` guard.
- ➖ The unused `pointInBox` and `CrossBoxSide` helpers, the unused `pa1..pb3` pre-declared arrays and the large commented blocks are noise.
- ➖ `fudge_factor = 1.05f` has the same value as before (the comment mentions trying 1.25).

### `REAL(-1.0)` → `-1.0` in the ray helpers
- ➖ This causes double → float truncation warnings. It is harmless, but the change was pointless and should be reverted to keep the literal type consistent.

### `dCollideRayCCylinder`: type assertion commented out
- ⚠️ This implies X-Ray calls it with a non-CCylinder geom (probably its own cylinder class with a compatible layout). That is dangerous if the layout differs. Document the reason.

---

## 4. Objects, world, bodies (`ode.cpp`, `objects.h`)

### Worldless bodies and joints: `dWorldAddBody/RemoveBody/AddJoint/RemoveJoint`, null-world creation
- ✅ This is the design X-Ray relies on: objects are created with `world==0` and attached and detached dynamically (the physics shell activates and deactivates). The implementation is straightforward.
- ⚠️ Every `dAASSERT(w)` in the creation functions and world setters was commented out. Passing `0` to `dWorldSetGravity/ERP/CFM` then works only *because* the fields became `static` (see below). This is intentional but surprising.
- ⚠️ `dJointAttach` no longer checks that the joint and bodies are in the same world. Required for the design, but it removes a safety net.
- ✅ `dJointDestroy` now guards `j->world` before unlinking, which is correct for worldless joints.

### `dxWorld` fields made `static`
- ⚠️ All worlds share gravity, ERP, CFM, auto-disable, quickstep and contact parameters. `dWorldCreate` still writes these fields, so **creating a second world resets the settings of the first**. This is fine for X-Ray, which has one physics world, but it is a fundamental ODE semantics change. The static initialisers in `ode.cpp` (CFM 1.136e-6, ERP 0.545, gravity (0,-1,0)) are then overwritten by `dWorldCreate` anyway, except CFM, ERP and gravity, which `dWorldCreate` sets to ODE defaults. Check which values the game actually expects.

### Defaults in `dWorldCreate`
- ➖ `qs.w = 1.1` (was 1.3) and `contactp.min_depth = 0.001` (was 0). These are deliberate X-Ray tuning values; a less aggressive SOR is more stable for stacked objects.

### Per-body auto-disable removed
- ➖ The `adis`, `adis_timeleft` and `adis_stepsleft` fields are commented out. The body getters return 0, the setters are no-ops, `dBodySetAutoDisableDefaults` forces the flag off, and `dInternalHandleAutoDisabling` is emptied. X-Ray implements its own sleeping logic, so this is acceptable, but the public API now silently lies. Consider `dDebug`/a comment in `objects.h` stating that auto-disable is unsupported.
- ➖ The comment typo "accululators" was introduced (cosmetic).

### `dxBodyNoUpdatePos` flag and `dBodySet/GetNoUpdatePosMode`
- ✅ The idea is to let a body take part in the solve without being integrated (used for kinematic or "frozen" objects). Velocity still accumulates while the position is frozen, which is expected semantics.
- ❌ `dBodyGetNoUpdatePosMode` returns `(flags & dxBodyNoUpdatePos) == 0`, which is **true when the mode is OFF**. It should be `!= 0`.
- ❌ Only `quickstep.cpp` and `dInternalStepIsland_x1` (`step.cpp:501`) check the flag. `_x1` is **dead code**: with `COMPARE_METHODS` off, `dInternalStepIsland` always calls `_x2`, and `_x2` (`step.cpp:948`) calls `dxStepBody` unconditionally. `stepfast.cpp:837` (`moveAndRotateBody`) doesn't check it either. So the flag works **only with `dWorldQuickStep`**. Add the check to `step.cpp:948` and `stepfast.cpp:837`.

### `dJointGroupEmpty` no longer unlinks joints from the world
- ⚠️ Correct **only if** grouped joints (contacts) are always created with `world == 0`, which is how X-Ray creates contacts. If any code does `dJointCreateContact(world, group, ...)`, `world->firstjoint` points into freed group memory after the next `dJointGroupEmpty`, and the step crashes. Safer: keep the `removeObjectFromList` under `if (jlist[i]->world)` exactly as before. It costs nothing for worldless joints.

### `dJointCreateContactSpecial`
- ✅ A contact whose normal force is capped at `1e5` (`hi[0]`). It is used for "soft" character or ragdoll contacts. ➖ `finit_big_force` is a `float` constant (fine).

---

## 5. Joints (`joint.cpp`)

- ➖ `min_stop_err`, `min_ball_err`, `hinge_min_err_exis_par`, `stop_early_reaction` and `add_min_err` are only used under `USE_*_MIN_ERR` macros, **none of which are defined**, so they are dead code and produce unused-variable warnings. The hinge `er1/er2` refactor is behaviour-preserving.
- ✅ `dJointAddAMotorTorques`: fixes a real ODE 0.5 bug (it used component `[0]` of axes 1 and 2 for all three components). Upstream fixed it the same way.
- ⚠️ `dJointSet/GetAMotorAngle`: the original `anum < 3` / clamp to `3` was off by one (`angle[3]` is out of bounds when `anum==3`). The new code clamps to 2 (correct) but asserts `anum < 2`, which **rejects the valid index 2**. Use `anum < 3`.
- ⚠️ `dJointSetFixedQuaternionPos`: implemented only for a fixed joint attached to a single body; the two-body branch is a **silent no-op**. Either implement it or `dUASSERT` that `node[1].body == 0`. The `ofs[4]` loop inside the commented code would also read `pos[3]`.
- ✅ `contactSpecialGetInfo2` / `__dcontact_special_vtable`: a clean extension that reuses the contact init and `info1`.

---

## 6. Steppers

### `util.cpp`: `dxProcessIslands` replaced ("no need Island collecting! @slipch")
The new version feeds **all bodies and joints of the world as one island** and re-enables every body each step.
- ➖ This is fine for X-Ray, which manages sleeping itself and usually steps small per-object worlds or islands. For `dWorldStep` (the big LCP, O(m³)) a single large island is **much** slower than island splitting. Make sure the game only uses `dWorldQuickStep` or step-fast on large worlds.
- ❌ `if (!joint->node[0].body) return;`: one unattached joint in the world silently **skips the whole step** (the stepper is never called and no error is reported). Change it to `continue;`.
- ⚠️ `ALLOCA(world->nj/nb * sizeof(ptr))` is unbounded stack use, the same as the original.

### `step.cpp`
- ✅ `dInternalStepIsland_x2`: the `j1==-1 || j2==-1` check was moved **before** `ofs[j1]`/`ofs[j2]` are read, which fixes an out-of-bounds read (`ofs[-1]`). This is the upstream fix.
- ✅ Invalid (NaN/Inf/denormal) constraint forces are zeroed before they are applied, a defensive measure against explosions.
- ⚠️ `float &` binding (issue #10).
- ➖ The signature changed from `dxJoint * const *` to `dxJoint **` (the steppers may reorder the caller's array; see quickstep).

### `quickstep.cpp`
- ❌ `#define WARM_STARTING 0` has no effect, because all uses are `#ifdef`. Warm starting is **still on** (λ *= 0.9 carried over). Decide which is intended: delete the define, or change the uses to `#if WARM_STARTING`. Note that X-Ray's contact joints are recreated every frame, so warm starting only helps persistent joints, and the source comment says it "hurts with high-friction contacts".
- ⚠️ The random constraint reorder now uses `std::shuffle` with a `thread_local std::mt19937` seeded from `std::random_device`, and it reshuffles every 4 iterations (was 8). Physics is **non-reproducible between runs** (demo record and debugging), and seeding `random_device` per thread costs a syscall. Seed it deterministically (a fixed seed, or `dRandGetSeed()`) unless non-determinism is wanted.
- ⚠️ `if (new_lambda < lo[index] && !isnan(lo[index]))` guards only `lo`. `hi` can be NaN too (`hi = hicopy * lambda[findex]`), and NaN `lambda` propagates anyway. This half-guard gives false confidence. It is better to sanitise `lambda[findex]` or rely on the `dValid` cforce check.
- ✅ Per-body NaN or Inf sanitising of `cforce`, with the joints' λ reset when anything was invalid.
- ⚠️ `float &` (issue #10). The λ reset loop hard-codes 6 rows (fine, `lambda[6]`).
- ➖ The joint array is no longer copied, so the stepper now **mutates the caller's joint array** (compaction of `m==0` joints). This is safe with the new `dxProcessIslands`, which builds a fresh array each step, but any other caller must know it.
- ➖ `static __cdecl int compare_index_error` is only compiled with `REORDER_CONSTRAINTS` (off). `__cdecl` placement is MSVC-specific.
- ➖ `multiply_J_invM_JT` commented out (unused); the `b` local pointer refactor is cosmetic; the `DEBUG_VALID` asserts are off by default.

### `stepfast.cpp` (the main X-Ray stepper, `dWorldStepFast1`)
- ✅ `MultiplyAdd2_sym_p8p`: fixes a **real ODE 0.5 bug**. For `j == i`, `aa` and `ad` alias the diagonal element, so the diagonal was added **twice** whenever a joint had two bodies. It is now added once.
- ✅ `NO_ISLANDS` enabled, consistent with the new "one island" philosophy.
- ✅ Contact joints with `m == 3` go to the new specialised solver (`dInternalStepJointContact` + `dSolveLCP33`), which is much cheaper than the generic `dSolveLCP` for the most common constraint. The deliberate fall-through to `default:` for other `m` is correct.
- ✅ `findex` is reset to -1 every iteration. Harmless and slightly more robust.
- ✅ The dead `FAST_FACTOR` LCP replacement was removed (the Russian comment says it lives in source control).
- ⚠️ Removing the `else if (body[1])` branches in `dInternalStepFast` (A and rhs) is only valid because the "disabled body → null" substitution was also commented out, so `bodyPair[0]` is null only for a detached joint, which `dJointAttach` never produces. If the disabled-body code is ever restored, `A` and `rhs` are used **uninitialised**. Add `dIASSERT(body[0])`.
- ❌ The `else` branch that restores `facc` for bodies with `dxBodyNoGravity` was removed. For such bodies `facc` is not reset from `saveFacc` at the start of each sub-iteration, so it keeps the previous sub-step's (already `*ministep`-scaled) value plus the constraint forces. Wrong dynamics for any gravity-less body when `maxiterations > 1`. Restore the branch.
- ❌ `SwapJoints(i, j, ...)`: the parameter `i` is ignored. The function swaps `joints[j]` with `joints[rand() % (j+1)]`, but the caller passes `j = r` (already random), so the loop is not a Fisher-Yates shuffle, the order is biased, and `rand()` is called twice per joint. Fix: `SwapJoints(j, r, ...)` should just swap indices `j` and `r`.
- ⚠️ `std::rand()` replaces `dRandInt`: global, non-thread-safe state that the game can reseed elsewhere, and the result is no longer controllable through `dRandSetSeed`.
- ➖ Islands with ≤3 joints (with `NO_ISLANDS`, the whole world) are routed to the exact `dInternalStepIsland` instead of the iterative solver. That is sensible for small systems: more accurate at negligible cost.
- ➖ `NOISING` (random jitter of forces) is dead code (not defined).
- ⚠️ The island version (compiled only without `NO_ISLANDS`, so currently dead) caps joints at `MAXJ_ALLOC = 2000` and **silently drops** the excess. It also has an unused `quit:` label and a disabled "attached enabled joint not tagged" check. Remove this code or make it correct.
- ➖ `dInternalHandleAutoDisabling` calls were removed (it is a no-op now anyway).

### `Bounder33.cpp/.h`: 3×3 box-bounded LCP for contacts (new)
It solves the normal and two friction rows by inverting the 3×3 `A` and then clamping variables iteratively (lines, then planes).
- ✅ It is much faster than the general Dantzig solver for m=3, and it matches the original X-Ray implementation.
- ⚠️ `ToNBn(int i)` does `Swap(i, NBn); NBn--;`. It should be `NBn--; Swap(i, NBn);` to mirror `ToBn`. It also resets `w[i]` using a *position* index as a *variable* index. After one failed "line" trial, the `index[]` permutation and the clamped `x` of the previous variable are left inconsistent, so the next trial effectively re-tests the old variable. The fallback `SolveForPlanes` usually still converges, so the effect is quality, not a crash. Verify against the original GSC source before changing it.
- ⚠️ `FillA` has no singularity check (the `VERIFY` is commented out). It relies on CFM > 0 on the diagonal, which ODE always adds, but a zero-mass or zero-CFM contact gives Inf.
- ⚠️ `dSolveLCP33`: `float friction` should be `dReal`. It divides by `A[0]` without a guard, ignores `n`, `w` and `nub`, and assumes the **fixed contact layout** (row 0 normal, rows 1 and 2 friction referencing row 0) whenever `findex != NULL`, which is always true in stepfast. It is correct for `contactGetInfo2` output. For `mu == dInfinity` it scales infinite bounds, and `friction == 0` then gives `inf*0 = NaN`, so clamp `friction` or check `dInfinity`.
- ➖ Unused members (`savedNbn`, `bounds`, `tw`, `BoundAll`, `EI`, `ToNBn()`, `CheckState`). The header doesn't include `<ode/common.h>` itself, and the files have no trailing newline.

### `StepJointInternal.cpp/.h` (new)
A copy of `dInternalStepFast`, specialised for a single 3-row contact, with unrolled 3×8 products.
- ✅ The maths matches `dInternalStepFast` (J·M⁻¹·Jᵀ + CFM/h, rhs = c/h − J·(v/h + M⁻¹f)).
- ⚠️ `info`/`Jinfo` are passed **by value** (Info2 is small, so this is fine, but inconsistent). `world`, `GI` and `nub` are unused. The header needs `joint.h` included before it.
- ⚠️ It assumes `body[0] != NULL` (see the stepfast note) and exactly 3 rows (guarded by the caller's `m==3`).

---

## 7. Recommendations

1. **Fix before shipping**: #1 (box-box `maxc` overflow), #2 (`p2`→`p1`), #3 (`return`→`continue`), #4 (inverted getter), #5 (no-gravity `facc` restore), #6 (shuffle), #7 (`WARM_STARTING` intent), #7b (`NoUpdatePos` in every stepper).
2. **Harden**: restore the world unlink in `dJointGroupEmpty` (it is free for worldless joints), use `dReal&` instead of `float&`, guard `_cos`/`A[0]`/`friction` against 0, and fix the AMotor assert to `< 3`.
3. **Document X-Ray contracts** in `objects.h`/`README`: a single global world parameter set, no auto-disable, contacts created with `world == 0`, joints always attached with `node[0].body != 0`, and steppers mutating the joint array.
4. **Build hygiene**: add the new sources to `Makefile`; remove the dead `USE_*_MIN_ERR`, `NOISING`, island-mode and `#if 0` code, or put it behind a single documented switch; guard the MSVC-only keywords (`_cdecl`, `__forceinline`, `__declspec`) with `_MSC_VER`.
5. **Determinism**: replace `std::random_device`/`std::rand()` with the seedable `dRandInt` (or a fixed-seed engine) so physics is reproducible for debugging.

---

## 8. Cleanup and fixes applied (2026-09-30, uncommitted)

Verified by building the library with the engine's flags (VS 2026 v145, C++23, `/permissive-`, AVX2, `dSINGLE; MSVC; dNODEBUG`) and a regression harness. The harness drives the library the way xrPhysics does: worldless joints plus `dWorldAddJoint`, and contacts dropped the way `CPHIsland::Unmerge()` drops them.

### Step 1: removed xray customisations the engine does not use
- Deleted `Bounder33.cpp/.h` and `StepJointInternal.cpp/.h`, and restored stock `stepfast.cpp`. The engine never calls `dWorldStepFast1`. Only xray's real fix to `MultiplyAdd2_sym_p8p` (diagonal added twice) was kept. **The engine's `default.vcxproj` must drop the two deleted `.cpp` files** when this is synced into `3rd_party/Src/ode`.
- `collision_std.cpp`: removed the GeomGroup API (unused), the unused `pointInBox`/`CrossBoxSide` helpers and commented-out code. Restored the `REAL(±1.0)` literals and the CCylinder assert in `dCollideRayCCylinder`.
- `joint.cpp`: removed the dead `USE_*_MIN_ERR` code (its macros were never defined).
- `util.cpp`: replaced the commented-out old island code with a short explanation. `quickstep.cpp`: dropped `__cdecl` from `compare_index_error` (dead). `error.cpp`/`objects.h`: small leftovers.
- Kept everything the engine relies on, including `dJointGroupEmpty` (see the note at the top).
- Result: `dWorldStep`, contact-special, ray and box-box outputs are **bit-identical** to before.

### Step 2: fixed xray's own bugs
| Fix | Location |
|---|---|
| Edge-edge box contacts honour `maxc` (no buffer overflow); the `pa2`/`pa3` probes use box 1's centre; parallel edge/face rejected (no division by ~0) | `collision_std.cpp` |
| An unattached joint no longer skips the whole island step (`return` → `continue`) | `util.cpp` |
| `dBodyGetNoUpdatePosMode` returned the inverted value | `ode.cpp` |
| `dxBodyNoUpdatePos` honoured by `dWorldStep` (`_x2`), so `CPHElement::Fix()` now works in small islands | `step.cpp` |
| `float&` → `dReal&`, `dValid(dReal)` (the build works with `dDOUBLE`) | `step.cpp`, `quickstep.cpp`, `util.h` |
| QuickStep constraint shuffle uses a fixed seed (reproducible, no `random_device`) | `quickstep.cpp` |
| `WARM_STARTING` made explicit (`1`), behaviour unchanged, with a comment on how to really disable it | `quickstep.cpp` |
| AMotor angle assert `anum < 3` (valid index 2 accepted) | `joint.cpp` |
| `dJointSetFixedQuaternionPos`: dead code removed; the unsupported two-body case asserts | `joint.cpp` |
| `_cdecl` removed from `dError/dDebug/dMessage` (not needed; non-portable) | `error.h`, `error.cpp` |
| `dValid` keyed on `_MSC_VER` as well as `MSVC` | `util.h` |

### Step 3: backported from upstream 0.8
| Upstream | Fix | Engine impact |
|---|---|---|
| `80e17f1e` + `510da6c2` | QuickStep joint feedback reports each joint's own force (`J'·λ`) instead of the total force on the body. Plus X-Ray: feedback is zeroed if the step's solution was rejected as NaN | **High**: PHFracture, PHCapture and movement activation now get the same per-joint forces under QuickStep (islands ≥ 30 joints) as they already did under `dWorldStep`. In the harness, two joints holding 1 kg reported 9.81 N each; now 4.905 N each. |
| `fe60f645` | A zero-velocity motor (joint friction) at the **high** stop pushed the joint away from the limit | **Medium**: ragdoll/joint friction at upper limits. Needs in-game check. |
| `59febbe9` | `dParamLoStop`/`HiStop` no longer silently ignored when a new range doesn't overlap the old one | `CPHJoint::SetLimitsActive`, car wheels |
| `d4c9f495` (Jacobian part) | Slider: body 1's angular Jacobian rows were never set | slider joints (`PHJoint.cpp`) |
| `3ad9d649` | AMotor user mode: a `rel = 1` axis was overwritten by the global-axis branch | none (the engine uses euler mode) |
| `246d7bb5` | `last_lambda` only allocated/copied when `REORDER_CONSTRAINTS` is on | small speed-up |

Considered and **not** ported: `dRandInt` rewrite (not used on live paths), capsule-box denormal fix (the engine has no capsules), `dGeomBoxPointDepth` (unused), QuickStep row-order optimisation `8a921d02` (changes behaviour), assert-only patches, auto-disable changes (not used).

**Recommended in-game checks** for the behaviour changes: breakable objects (fracture thresholds), grabbing objects (capture), ragdoll limbs resting at joint limits, fixed/animated physics elements, box stacks, doors and car suspension.

## 9. API additions for the engine

- `dGeomTransformGetFinalPos(g)` (2026-10-01): returns the cached final position of a geom transform (`dxGeomTransform::final_pos`, set by `computeAABB`). OGSR `PHMoveStorage.cpp` used to read it by re-declaring the private struct from `collision_transform.cpp` (an ODR violation that breaks silently if the layout changes). `collision.h`, `collision_transform.cpp`, `msvcdefs.def`.
