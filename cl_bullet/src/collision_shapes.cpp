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

#include <btBulletCollisionCommon.h>

extern "C"
{

  void deleteCollisionShape(btCollisionShape *ptr)
  {
    delete ptr;
  }
  
  btCollisionShape *newBoxShape(const double *half_extents)
  {
    return new btBoxShape(btVector3(half_extents[0], half_extents[1], half_extents[2]));
  }

  bool isBoxShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btBoxShape *>(ptr) != NULL;
  }

  btCollisionShape *newStaticPlaneShape(const double *normal, double constant)
  {
    return new btStaticPlaneShape(btVector3(normal[0], normal[1], normal[2]), constant);
  }

  bool isStaticPlaneShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btStaticPlaneShape *>(ptr) != NULL;
  }

  btCollisionShape *newSphereShape(double radius)
  {
    return new btSphereShape(radius);
  }

  bool isSphereShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btSphereShape *>(ptr) != NULL;
  }

  btCollisionShape *newCylinderShape(const double *half_extents)
  {
    return new btCylinderShape(btVector3(half_extents[0], half_extents[1], half_extents[2]));
  }

  bool isCylinderShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btCylinderShape *>(ptr) != NULL;
  }
  
  btCollisionShape *newConeShape(double radius, double height)
  {
    return new btConeShape(radius, height);
  }

  bool isConeShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btConeShape *>(ptr) != NULL;
  }

  bool newCompoundShape()
  {
    return new btCompoundShape();
  }

  bool isCompoundShape(const btCompoundShape *ptr)
  {
    return dynamic_cast<const btCompoundShape *>(ptr) != NULL;
  }

  void addChildShape(btCompoundShape *parent,
    const double *position, const double *orientation,
    btCollisionShape *shape)
  {
    parent->addChildShape(
      btTransform(
        btQuaternion(orientation[0],
          orientation[1],
          orientation[2],
          orientation[3]),
        btVector3(position[0],
          position[1],
          position[2])),
      shape);
  }

  btCollisionShape *newConvexHullShape(const double *points, int num_points)
  {
    return new btConvexHullShape(points, num_points, sizeof(double) * 3);
  }

  bool isConvexHullShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btConvexHullShape *>(ptr) != NULL;
  }

  void addPoint(btConvexHullShape *shape, const double *point)
  {
    shape->addPoint(btVector3(point[0], point[1], point[2]));
  }

}
