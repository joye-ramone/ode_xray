// Runtime argument validation (OGSR, include/ode/error.h dSetValidation): works with dNODEBUG, off by default

#ifndef _ODE_VALIDATE_H_
#define _ODE_VALIDATE_H_

#include <ode/common.h>
#include <ode/error.h>
#include <ode/mass.h>
#include <float.h>

extern int dxValidation;

// reports a rejected argument (d_ERR_VALIDATE through the message handler) and counts it
void dxValidateFail (const char *fn, const char *what, ...);

inline bool dxFinite (dReal x) { return _finite (x) != 0; }
inline bool dxFinite3 (dReal x, dReal y, dReal z) { return dxFinite (x) && dxFinite (y) && dxFinite (z); }
inline bool dxFinite3 (const dReal *v) { return dxFinite3 (v[0], v[1], v[2]); }
inline bool dxFinite4 (const dReal *v) { return dxFinite3 (v) && dxFinite (v[3]); }
inline bool dxFiniteR (const dReal *R) { return dxFinite3 (R) && dxFinite3 (R + 4) && dxFinite3 (R + 8); }
// a direction that can be normalized
inline bool dxAxis3 (dReal x, dReal y, dReal z) { return dxFinite3 (x, y, z) && x * x + y * y + z * z > REAL(1e-12); }

// mass > 0, finite, inertia positive definite, centre of mass consistent with it (mass.cpp)
int dxMassValid (const dMass *m);

// return from a void setter when validation is on and cond fails
#define dVALIDATE(cond, fn, ...) do { if (dxValidation && !(cond)) { dxValidateFail (fn, __VA_ARGS__); return; } } while (0)
// the same in a function that returns a value
#define dVALIDATE_RET(cond, ret, fn, ...) do { if (dxValidation && !(cond)) { dxValidateFail (fn, __VA_ARGS__); return ret; } } while (0)
// report only (the function goes on: its output is not a value given to a body)
#define dVALIDATE_REPORT(cond, fn, ...) do { if (dxValidation && !(cond)) dxValidateFail (fn, __VA_ARGS__); } while (0)

#endif
