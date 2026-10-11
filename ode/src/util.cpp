/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include "ode/ode.h"
#include "objects.h"
#include "joint.h"
#include "util.h"

#define ALLOCA dALLOCA16

//****************************************************************************
// Auto disabling

void dInternalHandleAutoDisabling (dxWorld *world, dReal stepsize)
{
	// X-Ray: ODE auto-disabling is not used, the engine handles sleeping itself
	// (the per-body auto-disable fields were removed from dxBody).
}


//****************************************************************************
// body rotation

// return sin(x)/x. this has a singularity at 0 so special handling is needed
// for small arguments.

static inline dReal sinc (dReal x)
{
  // if |x| < 1e-4 then use a taylor series expansion. this two term expansion
  // is actually accurate to one LS bit within this range if double precision
  // is being used - so don't worry!
  if (dFabs(x) < 1.0e-4) return REAL(1.0) - x*x*REAL(0.166666666666666666667);
  else return dSin(x)/x;
}


// given a body b, apply its linear and angular rotation over the time
// interval h, thereby adjusting its position and orientation.

void dxStepBody (dxBody *b, dReal h)
{
  int j;
#ifdef DEBUG_VALID
dIASSERT(dValid(b->avel[0])&&dValid(b->avel[1])&&dValid(b->avel[2]));
#endif
  // handle linear velocity
  for (j=0; j<3; j++) b->pos[j] += h * b->lvel[j];

  if (b->flags & dxBodyFlagFiniteRotation) {
    dVector3 irv;	// infitesimal rotation vector
    dQuaternion q;	// quaternion for finite rotation

    if (b->flags & dxBodyFlagFiniteRotationAxis) {
      // split the angular velocity vector into a component along the finite
      // rotation axis, and a component orthogonal to it.
      dVector3 frv;		// finite rotation vector
      dReal k = dDOT (b->finite_rot_axis,b->avel);
      frv[0] = b->finite_rot_axis[0] * k;
      frv[1] = b->finite_rot_axis[1] * k;
      frv[2] = b->finite_rot_axis[2] * k;
      irv[0] = b->avel[0] - frv[0];
      irv[1] = b->avel[1] - frv[1];
      irv[2] = b->avel[2] - frv[2];

      // make a rotation quaternion q that corresponds to frv * h.
      // compare this with the full-finite-rotation case below.
      h *= REAL(0.5);
      dReal theta = k * h;
      q[0] = dCos(theta);
      dReal s = sinc(theta) * h;
      q[1] = frv[0] * s;
      q[2] = frv[1] * s;
      q[3] = frv[2] * s;
    }
    else {
      // make a rotation quaternion q that corresponds to w * h
      dReal wlen = dSqrt (b->avel[0]*b->avel[0] + b->avel[1]*b->avel[1] +
			  b->avel[2]*b->avel[2]);
      h *= REAL(0.5);
      dReal theta = wlen * h;
      q[0] = dCos(theta);
      dReal s = sinc(theta) * h;
      q[1] = b->avel[0] * s;
      q[2] = b->avel[1] * s;
      q[3] = b->avel[2] * s;
    }

    // do the finite rotation
    dQuaternion q2;
    dQMultiply0 (q2,q,b->q);
    for (j=0; j<4; j++) b->q[j] = q2[j];

    // do the infitesimal rotation if required
    if (b->flags & dxBodyFlagFiniteRotationAxis) {
      dReal dq[4];
      dWtoDQ (irv,b->q,dq);
      for (j=0; j<4; j++) b->q[j] += h * dq[j];
    }
  }
  else {
    // the normal way - do an infitesimal rotation
    dReal dq[4];
    dWtoDQ (b->avel,b->q,dq);
    for (j=0; j<4; j++) b->q[j] += h * dq[j];
  }

  // normalize the quaternion and convert it to a rotation matrix
  dNormalize4 (b->q);
  dQtoR (b->q,b->R);

  // notify all attached geoms that this body has moved
  for (dxGeom *geom = b->geom; geom; geom = dGeomGetBodyNext (geom))
    dGeomMoved (geom);

#ifdef DEBUG_VALID
dIASSERT(dValid(b->avel[0])&&dValid(b->avel[1])&&dValid(b->avel[2]));
#endif
}

//****************************************************************************
// island processing

// X-Ray: the engine (CPHIsland) already builds the islands and gives every
// island its own dxWorld, so the whole world is stepped as a single island.
// bodies are never auto-disabled here; the engine handles sleeping itself.
void dxProcessIslands (dxWorld *world, dReal stepsize, dstepper_fn_t stepper)
{
	// nothing to do if no bodies
	if (world->nb <= 0)
		return;

#	ifdef TIMING
	dTimerStart ("creating joint and body arrays");
#	endif
	dxBody **bodies, *body;
	dxJoint **joints, *joint;
	joints = (dxJoint **) ALLOCA (world->nj * sizeof (dxJoint *));
	bodies = (dxBody **) ALLOCA (world->nb * sizeof (dxBody *));

	int nj = 0;
    for (joint = world->firstjoint; joint; joint = (dxJoint*)joint->next)
    {
        if (!joint->node[0].body)
            continue;		// unattached joint: not part of the island

        joints[nj++] = joint;
    }

	int nb = 0;
	for (body = world->firstbody; body; body = (dxBody *) body->next)
	{
		body->flags &= ~dxBodyDisabled;
		bodies[nb++] = body;

	}

	// now do something with body and joint lists
	stepper (world,bodies,nb,joints,nj,stepsize);
//stepper (world,body,bcount,joint,jcount,stepsize);
#	ifdef TIMING
	dTimerEnd ();
	dTimerReport (stdout, 1);
#	endif
}