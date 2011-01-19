/**
 * Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <btBulletDynamicsCommon.h>

extern "C"
{

  btRigidBody *newRigidBody(double mass, btMotionState *motionState, btCollisionShape *collisionShape)
  {
    btVector3 intertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, intertia);
    return new btRigidBody(mass, motionState, collisionShape, intertia);
  }

  void deleteRigidBody(btRigidBody *body)
  {
    delete body;
  }

  void getTotalForce(btRigidBody *body, double *forceVector)
  {
    btVector3 forceVectorBt = body->getTotalForce();
    forceVector[0] = forceVectorBt.x();
    forceVector[1] = forceVectorBt.y();
    forceVector[2] = forceVectorBt.z();
  }

  void getTotalTorque(btRigidBody *body, double *torqueVector)
  {
    btVector3 torqueVectorBt = body->getTotalTorque();
    torqueVector[0] = torqueVectorBt.x();
    torqueVector[1] = torqueVectorBt.y();
    torqueVector[2] = torqueVectorBt.z();
  }

  void applyForce(btRigidBody *body, double *force, double *rel_pos)
  {
    btVector3 force_vec(force[0], force[1], force[2]);
    btVector3 rel_pos_vec(rel_pos[0], rel_pos[1], rel_pos[2]);
    body->applyForce(force_vec, rel_pos_vec);
  }

  void applyCentralForce(btRigidBody *body, double *force)
  {
    btVector3 force_vec(force[0], force[1], force[2]);
    body->applyCentralForce(force_vec);
  }

  void applyTorque(btRigidBody *body, double *torque)
  {
    btVector3 torque_vec(torque[0], torque[1], torque[2]);
    body->applyTorque(torque_vec);
  }

  void clearForces(btRigidBody *body)
  {
    body->clearForces();
  }

  btMotionState *getMotionState(btRigidBody *body)
  {
    return body->getMotionState();
  }

  void setMotionState(btRigidBody *body, btMotionState *motionState)
  {
    body->setMotionState(motionState);
  }

  void setActivationState(btRigidBody *body, int newState)
  {
    body->setActivationState(newState);
  }

  int getActivationState(btRigidBody *body)
  {
    return body->getActivationState();
  }

  void setCollisionFlags(btRigidBody *body, int flags)
  {
    body->setCollisionFlags(flags);
  }

  int getCollisionFlags(btRigidBody *body)
  {
    return body->getCollisionFlags();
  }

  btCollisionShape *getCollisionShape(btRigidBody *body)
  {
    return body->getCollisionShape();
  }
}
