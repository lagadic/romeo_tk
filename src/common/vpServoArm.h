#ifndef __vpServoArm_h__
#define __vpServoArm_h__

#include <visp/vpCameraParameters.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpServo.h>
#include <visp/vpServoDisplay.h>

class vpServoArm
{
public:
  vpServo m_task; // Visual servoing task for the hand

protected:
  vpFeatureTranslation m_t;
  vpFeatureThetaU m_tu;
  double m_lambda;

public:
  vpServoArm();
  virtual ~vpServoArm() {
    m_task.kill();
  }

  vpColVector computeControlLaw()
  {
    return ( m_task.computeControlLaw() );
  }
  vpColVector computeControlLaw(double servo_time_init)
  {
    return ( m_task.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init) );
  }
  vpMatrix getTaskJacobian() {return m_task.getTaskJacobian();}
  vpMatrix getTaskJacobianPseudoInverse() {return m_task.getTaskJacobianPseudoInverse();}
  vpColVector getError(){return m_task.getError();}

  void set_eJe(const vpMatrix &eJe) { m_task.set_eJe(eJe);}
  void set_cVf(const vpVelocityTwistMatrix &cVf) { m_task.set_cVf(cVf);}
  void set_fVe(const vpVelocityTwistMatrix &fVe) { m_task.set_fVe(fVe);}
  void setCurrentFeature(const vpHomogeneousMatrix &cdMc)
  {
    m_t.buildFrom(cdMc) ;
    m_tu.buildFrom(cdMc) ;
  }
  void setLambda(double lambda) {m_task.setLambda(lambda);}
  void setLambda(const vpAdaptiveGain &lambda) {m_task.setLambda(lambda);}

};

#endif

