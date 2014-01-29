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

struct DynamicsWorldHandle
{
  btBroadphaseInterface *broadphase;
  btCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  btConstraintSolver *solver;
  btDynamicsWorld *dynamicsWorld;
};

double bulletWorldScalingFactor = 1;
  
extern "C"
{
  DynamicsWorldHandle *newDiscreteDynamicsWorld(double *gravityVector)
  {
    DynamicsWorldHandle *handle = new DynamicsWorldHandle;
    handle->broadphase = new btDbvtBroadphase();
    handle->collisionConfiguration = new btDefaultCollisionConfiguration();
    handle->dispatcher = new btCollisionDispatcher(handle->collisionConfiguration);
    handle->solver = new btSequentialImpulseConstraintSolver;
    handle->dynamicsWorld = new btDiscreteDynamicsWorld(handle->dispatcher,
                                                        handle->broadphase,
                                                        handle->solver,
                                                        handle->collisionConfiguration);
    handle->dynamicsWorld->setGravity(
      btVector3(
        gravityVector[0] * bulletWorldScalingFactor,
        gravityVector[1] * bulletWorldScalingFactor,
        gravityVector[2] * bulletWorldScalingFactor));
    
    return handle;
  }

  void deleteDiscreteDynamicsWorld(DynamicsWorldHandle *handle)
  {
    delete handle->dynamicsWorld;
    delete handle->solver;
    delete handle->dispatcher;
    delete handle->collisionConfiguration;
    delete handle->broadphase;
    delete handle;
  }

  void getGravity(DynamicsWorldHandle *handle, double *gravity)
  {
    btVector3 gravity_vec = handle->dynamicsWorld->getGravity();
    gravity[0] = gravity_vec.x() * bulletWorldScalingFactor;
    gravity[1] = gravity_vec.y() * bulletWorldScalingFactor;
    gravity[2] = gravity_vec.z() * bulletWorldScalingFactor;
  }

  void setGravity(DynamicsWorldHandle *handle, double *gravity)
  {
    handle->dynamicsWorld->setGravity(
      btVector3(gravity[0] * bulletWorldScalingFactor, gravity[1] * bulletWorldScalingFactor, gravity[2] * bulletWorldScalingFactor));
  }

  void stepSimulation(DynamicsWorldHandle *handle, double timeStep)
  {
    handle->dynamicsWorld->stepSimulation(timeStep);
  }

  void addConstraint(DynamicsWorldHandle *handle, btTypedConstraint *constraint, bool disableCollisions)
  {
    handle->dynamicsWorld->addConstraint(constraint, disableCollisions);
  }

  void removeConstraint(DynamicsWorldHandle *handle, btTypedConstraint *constraint)
  {
    handle->dynamicsWorld->removeConstraint(constraint);
  }

  int getNumConstraints(DynamicsWorldHandle *handle)
  {
    return handle->dynamicsWorld->getNumConstraints();
  }

  btTypedConstraint *getConstraint(DynamicsWorldHandle *handle, int index)
  {
    return handle->dynamicsWorld->getConstraint(index);
  }

  void addRigidBody(DynamicsWorldHandle *handle, btRigidBody *body)
  {
    handle->dynamicsWorld->addRigidBody(body);
  }

  void addRigidBodyWithMask(DynamicsWorldHandle *handle, btRigidBody *body, short group, short mask)
  {
    btDiscreteDynamicsWorld *world = dynamic_cast<btDiscreteDynamicsWorld *>(handle->dynamicsWorld);

    if(world)
      world->addRigidBody(body, group, mask);
    else
      addRigidBody(handle, body);
  }

  void removeRigidBody(DynamicsWorldHandle *handle, btRigidBody *body)
  {
    handle->dynamicsWorld->removeRigidBody(body);
  }

  int getNumRigidBodies(const DynamicsWorldHandle *handle)
  {
    return handle->dynamicsWorld->getNumCollisionObjects();
  }

  btRigidBody *getRigidBody(const DynamicsWorldHandle *handle, int index)
  {
    return btRigidBody::upcast(handle->dynamicsWorld->getCollisionObjectArray()[index]);
  }

  void setDebugDrawer(DynamicsWorldHandle *handle, btIDebugDraw *debugDrawer)
  {
    btDiscreteDynamicsWorld *handle_ =
      dynamic_cast<btDiscreteDynamicsWorld *>(handle->dynamicsWorld);

    if(handle_)
      handle_->setDebugDrawer(debugDrawer);
  }

  btIDebugDraw *getDebugDrawer(DynamicsWorldHandle *handle)
  {
    btDiscreteDynamicsWorld *handle_ =
      dynamic_cast<btDiscreteDynamicsWorld *>(handle->dynamicsWorld);

    if(handle_)
      return handle_->getDebugDrawer();
    else
      return NULL;
  }

  void debugDrawWorld(DynamicsWorldHandle *handle)
  {
    btDiscreteDynamicsWorld *handle_ =
      dynamic_cast<btDiscreteDynamicsWorld *>(handle->dynamicsWorld);

    if(handle_)
      handle_->debugDrawWorld();
  }

  void serializeWorld(DynamicsWorldHandle *handle, btSerializer *serializer)
  {
    handle->dynamicsWorld->serialize(serializer);
  }

  /** Collisions **/

  void performDiscreteCollisionDetection(DynamicsWorldHandle *handle)
  {
    handle->dynamicsWorld->performDiscreteCollisionDetection();
  }
  
  int getNumManifolds(DynamicsWorldHandle *handle)
  {
    return handle->dispatcher->getNumManifolds();
  }

  btPersistentManifold *getManifoldByIndex(DynamicsWorldHandle *handle, int index)
  {
    return handle->dispatcher->getManifoldByIndexInternal(index);
  }

  const btCollisionObject *manifoldGetBody0(btPersistentManifold *manifold)
  {
    return reinterpret_cast<const btCollisionObject *>(manifold->getBody0());
  }

  const btCollisionObject *manifoldGetBody1(btPersistentManifold *manifold)
  {
    return reinterpret_cast<const btCollisionObject *>(manifold->getBody1());
  }

  int manifoldGetNumContactPoints(btPersistentManifold *manifold)
  {
    return manifold->getNumContacts();
  }

  void manifoldGetContactPoint0(btPersistentManifold *manifold, int index, double *point)
  {
    btManifoldPoint &pt = manifold->getContactPoint(index);

    const btVector3 &point_on_a_vec = pt.getPositionWorldOnA();
    point[0] = point_on_a_vec.x() / bulletWorldScalingFactor;
    point[1] = point_on_a_vec.y() / bulletWorldScalingFactor;
    point[2] = point_on_a_vec.z() / bulletWorldScalingFactor;
  }

  void manifoldGetContactPoint1(btPersistentManifold *manifold, int index, double *point)
  {
    btManifoldPoint &pt = manifold->getContactPoint(index);
        
    const btVector3 &point_on_b_vec = pt.getPositionWorldOnB();
    point[0] = point_on_b_vec.x() / bulletWorldScalingFactor;
    point[1] = point_on_b_vec.y() / bulletWorldScalingFactor;
    point[2] = point_on_b_vec.z() / bulletWorldScalingFactor;
  }
  
}
