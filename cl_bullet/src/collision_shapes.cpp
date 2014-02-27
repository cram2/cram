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

#include <stdio.h>

#include <btBulletCollisionCommon.h>

extern double bulletWorldScalingFactor;

extern "C"
{

  void deleteCollisionShape(btCollisionShape *ptr)
  {
    delete ptr;
  }

  int getShapeType(btCollisionShape *ptr)
  {
    return ptr->getShapeType();
  }
  
  btCollisionShape *newBoxShape(const double *half_extents)
  {
    return new btBoxShape(btVector3(half_extents[0] * bulletWorldScalingFactor,
        half_extents[1] * bulletWorldScalingFactor,
        half_extents[2] * bulletWorldScalingFactor));
  }

  bool isBoxShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btBoxShape *>(ptr) != NULL;
  }

  void getBoxHalfExtents(const btBoxShape *box, double *half_extents)
  {
    btVector3 extents = box->getHalfExtentsWithMargin() / bulletWorldScalingFactor;
    half_extents[0] = extents.x();
    half_extents[1] = extents.y();
    half_extents[2] = extents.z();
  }

  btCollisionShape *newStaticPlaneShape(const double *normal, double constant)
  {
    return new btStaticPlaneShape(btVector3(normal[0], normal[1], normal[2]), constant);
  }

  bool isStaticPlaneShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btStaticPlaneShape *>(ptr) != NULL;
  }

  void getPlaneNormal(const btStaticPlaneShape *plane, double *normal)
  {
    btVector3 normal_vec = plane->getPlaneNormal();
    normal[0] = normal_vec.x();
    normal[1] = normal_vec.y();
    normal[2] = normal_vec.z();
  }

  double getPlaneConstant(const btStaticPlaneShape *plane)
  {
    return plane->getPlaneConstant();
  }
  
  btCollisionShape *newSphereShape(double radius)
  {
    return new btSphereShape(radius * bulletWorldScalingFactor);
  }

  bool isSphereShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btSphereShape *>(ptr) != NULL;
  }

  double getSphereRadius(const btSphereShape *sphere)
  {
    return sphere->getRadius() / bulletWorldScalingFactor;
  }

  btCollisionShape *newCylinderShape(const double *half_extents)
  {
    return new btCylinderShapeZ(
      btVector3(
        half_extents[0] * bulletWorldScalingFactor,
        half_extents[1] * bulletWorldScalingFactor,
        half_extents[2] * bulletWorldScalingFactor));
  }

  bool isCylinderShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btCylinderShape *>(ptr) != NULL;
  }

  void getCylinderHalfExtents(const btCylinderShape *cylinder, double *half_extents)
  {
    btVector3 extents = cylinder->getHalfExtentsWithMargin() / bulletWorldScalingFactor;
    half_extents[0] = extents.x();
    half_extents[1] = extents.y();
    half_extents[2] = extents.z();
  }
  
  btCollisionShape *newConeShape(double radius, double height)
  {
    return new btConeShapeZ(
      radius * bulletWorldScalingFactor,
      height * bulletWorldScalingFactor);
  }

  bool isConeShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btConeShape *>(ptr) != NULL;
  }

  double getConeRadius(const btConeShape *cone)
  {
    return cone->getRadius() / bulletWorldScalingFactor;
  }

  double getConeHeight(const btConeShape *cone)
  {
    return cone->getHeight() / bulletWorldScalingFactor;
  }

  btCollisionShape *newCompoundShape()
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
        btVector3(position[0] * bulletWorldScalingFactor,
          position[1] * bulletWorldScalingFactor,
          position[2] * bulletWorldScalingFactor)),
      shape);
  }

  int getNumChildShapes(btCompoundShape *shape)
  {
    return shape->getNumChildShapes();
  }

  const btCollisionShape *getChildShape(const btCompoundShape *shape, int index)
  {
    return shape->getChildShape(index);
  }

  void getChildTransform(const btCompoundShape *shape, int index, double *transform)
  {
    const btTransform &trans = shape->getChildTransform(index);
    const btVector3 &pos = trans.getOrigin() / bulletWorldScalingFactor;
    btQuaternion rot = trans.getRotation();
    transform[0] = pos.x();
    transform[1] = pos.y();
    transform[2] = pos.z();
    transform[3] = rot.x();
    transform[4] = rot.y();
    transform[5] = rot.z();
    transform[6] = rot.w();
  }

  btCollisionShape *newConvexHullShape(const double *points, int num_points)
  {
    btScalar *scaled_points = new btScalar[num_points*3];
    for(int i=0; i<num_points*3; i++)
      scaled_points[i] = static_cast<btScalar>(points[i] * bulletWorldScalingFactor);
    btCollisionShape *shape = new btConvexHullShape(scaled_points, num_points, 3 * sizeof(btScalar));
    delete scaled_points;
    return shape;
  }

  bool isConvexHullShape(const btCollisionShape *ptr)
  {
    return dynamic_cast<const btConvexHullShape *>(ptr) != NULL;
  }

  void addPoint(btConvexHullShape *shape, const double *point)
  {
    shape->addPoint(btVector3(point[0] * bulletWorldScalingFactor,
        point[1] * bulletWorldScalingFactor, point[2] * bulletWorldScalingFactor));
  }

  int convexHullGetNumPoints(const btConvexHullShape *shape)
  {
    return shape->getNumPoints();
  }

  void getPoint(const btConvexHullShape *shape, int index, double *point)
  {
    const btVector3 &pt = shape->getUnscaledPoints()[index] / bulletWorldScalingFactor;
    point[0] = pt.x();
    point[1] = pt.y();
    point[2] = pt.z();
  }
}
