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

#include <LinearMath/btDefaultMotionState.h>

extern "C"
{

  btDefaultMotionState *newMotionState(const double *position, const double *orientation)
  {
    return new btDefaultMotionState(btTransform(btQuaternion(orientation[0],
                                                             orientation[1],
                                                             orientation[2],
                                                             orientation[3]),
                                                btVector3(position[0],
                                                          position[1],
                                                          position[2])));
  }

  void deleteMotionState(btDefaultMotionState *motionState)
  {
    delete motionState;
  }

  void setCenterOfMass(btDefaultMotionState *motionState, const double *centerOfMass)
  {
    motionState->m_centerOfMassOffset = btTransform(btQuaternion(0, 0, 0, 1),
                                                    btVector3(centerOfMass[0],
                                                              centerOfMass[1],
                                                              centerOfMass[2]));
  }

  void getWorldTransform(btMotionState *motionState, double *position, double *orientation)
  {
    btTransform result;

    motionState->getWorldTransform(result);
    position[0] = result.getOrigin().x();
    position[1] = result.getOrigin().y();
    position[2] = result.getOrigin().z();
    orientation[0] = result.getRotation().x();
    orientation[1] = result.getRotation().y();
    orientation[2] = result.getRotation().z();
    orientation[3] = result.getRotation().w();
  }
  
}
