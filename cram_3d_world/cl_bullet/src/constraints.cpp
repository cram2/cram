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

extern double bulletWorldScalingFactor;

extern "C"
{

  void deleteConstraint(btTypedConstraint *constraint)
  {
    delete constraint;
  }

  /**
   * Point2PointConstraint
   */
  btTypedConstraint *newPoint2PointConstraint(
    btRigidBody *rbA, btRigidBody *rbB,
    const double *pivotInA, const double *pivotInB)
  {
    return new btPoint2PointConstraint(*rbA, *rbB,
      btVector3(pivotInA[0] * bulletWorldScalingFactor,
        pivotInA[1] * bulletWorldScalingFactor,
        pivotInA[2] * bulletWorldScalingFactor),
      btVector3(pivotInB[0] * bulletWorldScalingFactor,
        pivotInB[1] * bulletWorldScalingFactor,
        pivotInB[2] * bulletWorldScalingFactor));
  }

  bool isPoint2PointConstraint(const btTypedConstraint *ptr)
  {
    return dynamic_cast<const btPoint2PointConstraint *>(ptr) != NULL;
  }

  /**
   * HingeConstraint
   */
  btTypedConstraint *newHingeConstraint(
    btRigidBody *rbA, btRigidBody *rbB,
    const double *frameInA, const double *frameInB)
  {
    btTransform transInA(
      btQuaternion(frameInA[3], frameInA[4], frameInA[5], frameInA[6]),
      btVector3(frameInA[0] * bulletWorldScalingFactor,
        frameInA[1] * bulletWorldScalingFactor,
        frameInA[2] * bulletWorldScalingFactor));
    btTransform transInB(
      btQuaternion(frameInB[3], frameInB[4], frameInB[5], frameInB[6]),
      btVector3(frameInB[0] * bulletWorldScalingFactor,
        frameInB[1] * bulletWorldScalingFactor,
        frameInB[2] * bulletWorldScalingFactor));
    
    return new btHingeConstraint(*rbA, *rbB, transInA, transInB);
  }

  bool isHingeConstraint(const btTypedConstraint *ptr)
  {
    return dynamic_cast<const btHingeConstraint *>(ptr) != NULL;
  }

  void setAngularOnly(btHingeConstraint *hinge, bool angularOnly)
  {
    hinge->setAngularOnly(angularOnly);
  }

  bool getAngularOnly(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getAngularOnly();
  }

  void enableAngularMotor(btHingeConstraint *hinge, bool enableMotor,
    double targetVelocity, double maxMotorImpulse)
  {
    hinge->enableAngularMotor(enableMotor, targetVelocity * bulletWorldScalingFactor, maxMotorImpulse);
  }

  void enableMotor(btHingeConstraint *hinge, bool enableMotor)
  {
    hinge->enableMotor(enableMotor);
  }

  bool getEnableMotor(btHingeConstraint *hinge)
  {
    return hinge->getEnableAngularMotor();
  }

  void setMaxMotorImpulse(btHingeConstraint *hinge, double maxMotorImpulse)
  {
    hinge->setMaxMotorImpulse(maxMotorImpulse * bulletWorldScalingFactor);
  }

  double getMaxMotorImpulse(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getMaxMotorImpulse();
  }

  double getMotorTargetVelocity(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getMotorTargetVelosity() / bulletWorldScalingFactor;
  }

  void setMotorTarget(btHingeConstraint *hinge, double targetAngle, double dt)
  {
    hinge->setMotorTarget(targetAngle, dt);
  }

  void setLimit(btHingeConstraint *hinge, double low, double high)
  {
    hinge->setLimit(low, high);
  }
  
  void setLimitComplex(btHingeConstraint *hinge,
    double low, double high,
    double softness, double biasFactor, double relaxationFactor)
  {
    hinge->setLimit(low, high, softness, biasFactor, relaxationFactor);
  }

  double getHingeAngle(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getHingeAngle();
  }

  double getLowerLimit(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getLowerLimit();
  }

  double getUpperLimit(/*const*/ btHingeConstraint *hinge)
  {
    return hinge->getUpperLimit();
  }

  /**
   * SliderConstraint
   */
  btTypedConstraint *newSliderConstraint(
    btRigidBody *rbA, btRigidBody *rbB,
    const double *frameInA, const double *frameInB)
  {
    return new btSliderConstraint(*rbA, *rbB,
      btTransform(
        btQuaternion(frameInA[3], frameInA[4], frameInA[5], frameInA[6]),
        btVector3(frameInA[0]  * bulletWorldScalingFactor,
          frameInA[1] * bulletWorldScalingFactor, frameInA[2] * bulletWorldScalingFactor)),
      btTransform(
        btQuaternion(frameInB[3], frameInB[4], frameInB[5], frameInB[6]),
        btVector3(frameInB[0] * bulletWorldScalingFactor,
          frameInB[1] * bulletWorldScalingFactor, frameInB[2] * bulletWorldScalingFactor)),
      false);
  }

  bool isSliderConstraint(const btTypedConstraint *ptr)
  {
    return dynamic_cast<const btSliderConstraint *>(ptr) != NULL;
  }

  double getLowerLinLimit(/*const*/ btSliderConstraint *slider)
  {
    return slider->getLowerLinLimit();
  }

  void setLowerLinLimit(btSliderConstraint *slider, double lowerLimit)
  {
    slider->setLowerLinLimit(lowerLimit);
  }

  double getUpperLinLimit(/*const*/ btSliderConstraint *slider)
  {
    return slider->getUpperLinLimit();
  }

  void setUpperLinLimit(btSliderConstraint *slider, double lowerLimit)
  {
    slider->setUpperLinLimit(lowerLimit);
  }

  double getLowerAngLimit(btSliderConstraint *slider)
  {
    return slider->getLowerAngLimit();
  }
  
  void setLowerAngLimit(btSliderConstraint *slider, double lowerLimit)
  {
    slider->setLowerAngLimit(lowerLimit);
  }

  double getUpperAngLimit(/*const*/ btSliderConstraint *slider)
  {
    return slider->getUpperAngLimit();
  }

  void setUpperAngLimit(btSliderConstraint *slider, double lowerLimit)
  {
    slider->setUpperAngLimit(lowerLimit);
  }

	double getSoftnessDirLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessDirLin();
  }
  
	double getRestitutionDirLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionDirLin();
  }
  
	double getDampingDirLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingDirLin();
  }
  
	double getSoftnessDirAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessDirAng();
  }
  
	double getRestitutionDirAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionDirAng();
  }
  
	double getDampingDirAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingDirAng();
  }
  
	double getSoftnessLimLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessLimLin();
  }
  
	double getRestitutionLimLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionLimLin();
  }
  
	double getDampingLimLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingLimLin();
  }
  
	double getSoftnessLimAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessLimAng();
  }
  
	double getRestitutionLimAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionLimAng();
  }
  
	double getDampingLimAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingLimAng();
  }
  
	double getSoftnessOrthoLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessOrthoLin();
  }
  
	double getRestitutionOrthoLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionOrthoLin();
  }
  
	double getDampingOrthoLin(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingOrthoLin();
  }
  
	double getSoftnessOrthoAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getSoftnessOrthoAng();
  }
  
	double getRestitutionOrthoAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getRestitutionOrthoAng();
  }
  
	double getDampingOrthoAng(/*const*/ btSliderConstraint *slider)
  {
    return slider->getDampingOrthoAng();
  }

	void setSoftnessDirLin(btSliderConstraint *slider, double softnessDirLin)
  {
    slider->setSoftnessDirLin(softnessDirLin);
  }
  
	void setRestitutionDirLin(btSliderConstraint *slider, double restitutionDirLin)
  {
    slider->setRestitutionDirLin(restitutionDirLin);
  }
  
	void setDampingDirLin(btSliderConstraint *slider, double dampingDirLin)
  {
    slider->setDampingDirLin(dampingDirLin);
  }
  
	void setSoftnessDirAng(btSliderConstraint *slider, double softnessDirAng)
  {
    slider->setSoftnessDirAng(softnessDirAng);
  }
  
	void setRestitutionDirAng(btSliderConstraint *slider, double restitutionDirAng)
  {
    slider->setRestitutionDirAng(restitutionDirAng);
  }
  
	void setDampingDirAng(btSliderConstraint *slider, double dampingDirAng)
  {
    slider->setDampingDirAng(dampingDirAng);
  }
  
	void setSoftnessLimLin(btSliderConstraint *slider, double softnessLimLin)
  {
    slider->setSoftnessLimLin(softnessLimLin);
  }
  
	void setRestitutionLimLin(btSliderConstraint *slider, double restitutionLimLin)
  {
    slider->setRestitutionLimLin(restitutionLimLin);
  }
  
	void setDampingLimLin(btSliderConstraint *slider, double dampingLimLin)
  {
    slider->setDampingLimLin(dampingLimLin);
  }
  
	void setSoftnessLimAng(btSliderConstraint *slider, double softnessLimAng)
  {
    slider->setSoftnessLimAng(softnessLimAng);
  }
  
	void setRestitutionLimAng(btSliderConstraint *slider, double restitutionLimAng)
  {
    slider->setRestitutionLimAng(restitutionLimAng);
  }
  
	void setDampingLimAng(btSliderConstraint *slider, double dampingLimAng)
  {
    slider->setDampingLimAng(dampingLimAng);
  }
  
	void setSoftnessOrthoLin(btSliderConstraint *slider, double softnessOrthoLin)
  {
    slider->setSoftnessOrthoLin(softnessOrthoLin);
  }
  
	void setRestitutionOrthoLin(btSliderConstraint *slider, double restitutionOrthoLin)
  {
    slider->setRestitutionOrthoLin(restitutionOrthoLin);
  }
  
	void setDampingOrthoLin(btSliderConstraint *slider, double dampingOrthoLin)
  {
    slider->setDampingOrthoLin(dampingOrthoLin);
  }
  
	void setSoftnessOrthoAng(btSliderConstraint *slider, double softnessOrthoAng)
  {
    slider->setSoftnessOrthoAng(softnessOrthoAng);
  }
  
	void setRestitutionOrthoAng(btSliderConstraint *slider, double restitutionOrthoAng)
  {
    slider->setRestitutionOrthoAng(restitutionOrthoAng);
  }
  
	void setDampingOrthoAng(btSliderConstraint *slider, double dampingOrthoAng)
  {
    slider->setDampingOrthoAng(dampingOrthoAng);
  }
  
  void setPoweredLinMotor(btSliderConstraint *slider, bool onOff)
  {
    slider->setPoweredLinMotor(onOff);
  }

  bool getPoweredLinMotor(/*const*/ btSliderConstraint *slider)
  {
    return slider->getPoweredLinMotor();
  }

	void setTargetLinMotorVelocity(btSliderConstraint *slider, double targetLinMotorVelocity)
  {
    slider->setTargetLinMotorVelocity(targetLinMotorVelocity * bulletWorldScalingFactor);
  }
  
	double getTargetLinMotorVelocity(/*const*/ btSliderConstraint *slider)
  {
    return slider->getTargetLinMotorVelocity() / bulletWorldScalingFactor;
  }
  
  void setMaxLinMotorForce(btSliderConstraint *slider, double maxLinMotorForce)
  {
    slider->setMaxLinMotorForce(maxLinMotorForce * bulletWorldScalingFactor);
  }
  
	double getMaxLinMotorForce(/*const*/ btSliderConstraint *slider)
  {
    return slider->getMaxLinMotorForce() / bulletWorldScalingFactor;
  }
  
	void setPoweredAngMotor(btSliderConstraint *slider, bool onOff)
  {
    slider->setPoweredAngMotor(onOff);
  }
  
	bool getPoweredAngMotor(/*const*/ btSliderConstraint *slider)
  {
    return slider->getPoweredAngMotor();
  }
  
	void setTargetAngMotorVelocity(btSliderConstraint *slider, double targetAngMotorVelocity)
  {
    slider->setTargetAngMotorVelocity(targetAngMotorVelocity);
  }
  
	double getTargetAngMotorVelocity(/*const*/ btSliderConstraint *slider)
  {
    return slider->getTargetAngMotorVelocity();
  }
  
	void setMaxAngMotorForce(btSliderConstraint *slider, double maxAngMotorForce)
  {
    slider->setMaxAngMotorForce(maxAngMotorForce);
  }
  
	double getMaxAngMotorForce(/*const*/ btSliderConstraint *slider)
  {
    return slider->getMaxAngMotorForce();
  }
  
	double getLinearPos(/*const*/ btSliderConstraint *slider)
  {
    return slider->getLinearPos() / bulletWorldScalingFactor;
  }
  
}
