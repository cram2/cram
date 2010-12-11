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

#include <LinearMath/btIDebugDraw.h>

class CLBulletDebugDraw : public btIDebugDraw
{
public:
  struct Callbacks
  {
    void (*drawLine)(const double */*from*/, const double */*to*/, const double */*color*/, void *);
    void (*drawSphere)(const double */*p*/, double /*radius*/, const double */*color*/, void *);
    void (*drawTriangle)(const double */*v0*/, const double */*v1*/, const double */*v2*/,
                         const double */*color*/, double /*alpha*/, void *);
    void (*drawBox)(const double */*boxMin*/, const double */*boxMax*/,
                    const double */*color*/, void *);
    void (*drawAabb)(const double */*from*/, const double */*to*/, const double */*color*/, void *);
    void (*drawTransform)(const double */*transform*/, double /*orthoLen*/, void *);
    void (*drawArc)(const double */*center*/, const double */*normal*/,
                    const double */*axis*/, double /*radiusA*/, double /*radiusB*/,
                    double /*minAngle*/, double /*maxAngle*/, const double */*color*/,
                    bool /*drawSect*/, double /*stepDegrees*/, void *);
    void (*drawSpherePatch)(const double */*center*/, const double */*up*/, const double */*axis*/,
                            double /*radius*/, double /*minTh*/, double /*maxTh*/, double /*minPs*/,
                            double /*maxPs*/, const double */*color*/,
                            double /*stepDegrees*/, void *);
    void (*drawContactPoint)(const double */*PointOnB*/, const double */*normalOnB*/, double /*distance*/,
                             int /*lifeTime*/, const double */*color*/, void *);
    void (*reportErrorWarning)(const char */*warningString*/, void *);
    void (*draw3dText)(const double */*location*/, const char */*string*/, void *); 

    Callbacks()
      : drawLine(0), drawSphere(0), drawTriangle(0), drawBox(0),
        drawAabb(0), drawTransform(0), drawArc(0), drawSpherePatch(0),
        drawContactPoint(0), reportErrorWarning(0), draw3dText(0) {}
  };


  CLBulletDebugDraw(void *arg=0)
    : debug_mode_(DBG_NoDebug),
      arg_(arg) {}

  CLBulletDebugDraw(const Callbacks &callbacks, void *arg=0)
    : debug_mode_(DBG_NoDebug),
      callbacks_(callbacks),
      arg_(arg) {}
    
  virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
  {
    double from_[3] = {from.x(), from.y(), from.z()};
    double to_[3] = {to.x(), to.y(), to.z()};
    double color_[3] = {color.x(), color.y(), color.z()};
    
    if(callbacks_.drawLine)
      (*callbacks_.drawLine)(from_, to_, color_, arg_);
  }

  virtual void drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
  {
    double p_[3] = {p.x(), p.y(), p.z()};
    double color_[3] = {color.x(), color.y(), color.z()};

    if(callbacks_.drawSphere)
      (*callbacks_.drawSphere)(p_, radius, color_, arg_);
    else
      btIDebugDraw::drawSphere(p, radius, color);
  }

  virtual void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar alpha)
  {
    double v0_[3] = {v0.x(), v0.y(), v0.z()};
    double v1_[3] = {v1.x(), v1.y(), v1.z()};
    double v2_[3] = {v2.x(), v2.y(), v2.z()};
    double color_[3] = {color.x(), color.y(), color.z()};
    

    if(callbacks_.drawTriangle)
      (callbacks_.drawTriangle)(v0_, v1_, v2_, color_, alpha, arg_);
    else
      btIDebugDraw::drawTriangle(v0, v1, v2, color, alpha);
  }

  virtual void drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color)
  {
    double boxMin_[3] = {boxMin.x(), boxMin.y(), boxMin.z()}; 
    double boxMax_[3] = {boxMax.x(), boxMax.y(), boxMax.z()};
    double color_[3] = {color.x(), color.y(), color.z()};
    
    if(callbacks_.drawBox)
      (*callbacks_.drawBox)(boxMin_, boxMax_, color_, arg_);
    else
      btIDebugDraw::drawBox(boxMin, boxMax, color);
  }

  virtual void drawAabb(const btVector3& from,const btVector3& to,const btVector3& color)
  {
    double from_[3] = {from.x(), from.y(), from.z()};
    double to_[3] = {to.x(), to.y(), to.z()};
    double color_[3] = {color.x(), color.y(), color.z()};
    
    if(callbacks_.drawAabb)
      (*callbacks_.drawAabb)(from_, to_, color_, arg_);
    else
      btIDebugDraw::drawAabb(from, to, color);
  }

  virtual void drawTransform(const btTransform& transform, btScalar orthoLen)
  {
    btVector3 origin = transform.getOrigin();
    btQuaternion rotation = transform.getRotation();
    double transform_[7] = {origin.x(), origin.y(), origin.z(),
                            rotation.x(), rotation.y(), rotation.z(),
                            rotation.w()};
    
    if(callbacks_.drawTransform)
      (*callbacks_.drawTransform)(transform_, orthoLen, arg_);
    else
      btIDebugDraw::drawTransform(transform, orthoLen);
  }

  virtual void drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, 
                       const btVector3& color, bool drawSect, btScalar stepDegrees = btScalar(10.f))
  {
    double center_[3] = {center.x(), center.y(), center.z()};
    double normal_[3] = {normal.x(), normal.y(), normal.z()};
    double axis_[3] = {axis.x(), axis.y(), axis.z()};
    double color_[3] = {color.x(), color.y(), color.z()};

    if(callbacks_.drawArc)
      (*callbacks_.drawArc)(center_, normal_, axis_, radiusA, radiusB, minAngle, maxAngle, color_, drawSect, stepDegrees, arg_);
    else
      btIDebugDraw::drawArc(center, normal, axis, radiusA, radiusB, minAngle, maxAngle, color, drawSect, stepDegrees);
  }

  virtual void drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis, btScalar radius, 
                               btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3& color, btScalar stepDegrees = btScalar(10.f))
  {
    double center_[3] = {center.x(), center.y(), center.z()};
    double up_[3] = {up.x(), up.y(), up.z()};
    double axis_[3] = {axis.x(), axis.y(), axis.z()};
    double color_[3] = {color.x(), color.y(), color.z()};
    
    if(callbacks_.drawSpherePatch)
      (*callbacks_.drawSpherePatch)(center_, up_, axis_, radius, minTh, maxTh, minPs, maxPs, color_, stepDegrees, arg_);
    else
      btIDebugDraw::drawSpherePatch(center, up, axis, radius, minTh, maxTh, minPs, maxPs, color, stepDegrees);
  }

  virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
  {
    double PointOnB_[3] = {PointOnB.x(), PointOnB.y(), PointOnB.z()};
    double normalOnB_[3] = {normalOnB.x(), normalOnB.y(), normalOnB.z()};
    double color_[3] = {color.x(), color.y(), color.z()};

    if(callbacks_.drawContactPoint)
      (*callbacks_.drawContactPoint)(PointOnB_, normalOnB_, distance, lifeTime, color_, arg_);
  }
  
  virtual void reportErrorWarning(const char* warningString)
  {
    if(callbacks_.reportErrorWarning)
      (*callbacks_.reportErrorWarning)(warningString, arg_);
  }

  void draw3dText(const btVector3& location,const char* textString)
  {
    double location_[3] = {location.x(), location.y(), location.z()};

    if(callbacks_.draw3dText)
      (*callbacks_.draw3dText)(location_, textString, arg_);
  }

  virtual void setDebugMode(int debugMode)
  {
    debug_mode_ = debugMode;
  }

  virtual int getDebugMode() const
  {
    return debug_mode_;
  }

  void setCallbacks(const Callbacks &callbacks)
  {
    callbacks_ = callbacks;
  }

  Callbacks &getCallbacks()
  {
    return callbacks_;
  }

private:
  int debug_mode_;
  Callbacks callbacks_;
  void *arg_;
};

extern "C"
{
  
  CLBulletDebugDraw *newCLBulletDebugDraw(CLBulletDebugDraw::Callbacks *callbacks, void *arg)
  {
    if(callbacks)
      return new CLBulletDebugDraw(*callbacks, arg);
    else
      return new CLBulletDebugDraw(arg);
  }

  void deleteCLBulletDebugDraw(CLBulletDebugDraw *ptr)
  {
    delete ptr;
  }

  void setDebugMode(CLBulletDebugDraw *draw, int debugMode)
  {
    draw->setDebugMode(debugMode);
  }

  int getDebugMode(CLBulletDebugDraw *draw)
  {
    return draw->getDebugMode();
  }

  void setCallbacks(CLBulletDebugDraw *draw, CLBulletDebugDraw::Callbacks *callbacks)
  {
    draw->setCallbacks(*callbacks);
  }

  CLBulletDebugDraw::Callbacks *getCallbacks(CLBulletDebugDraw *draw)
  {
    return &draw->getCallbacks();
  }

}
