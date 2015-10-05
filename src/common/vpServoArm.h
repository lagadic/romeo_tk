#ifndef __vpServoArm_h__
#define __vpServoArm_h__

#include <visp/vpCameraParameters.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpServo.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpGenericFeature.h>

class vpServoArm
{
public:
  vpServo m_task; // Visual servoing task for the hand

  /*!
    \enum vpServoArmType
    Type of visual servoing implemented.
   */
  typedef enum {
    /*! Use Translation(cdMc) and ThetaU (vpFeatureThetaU::cdRc) features
        Here we controll all the 6DOF*/
    vs6dof,
    /*! Use Translation(cdMc) and ThetaUx and ThetaUy (vpFeatureThetaU::cdRc) features.
        Here we controll 5DOF: Used to grasp cylindrical object*/
    vs5dof_cyl,
    /*! Use Translation(cdMc) and a alignment z axis features.
        Here we controll 5DOF (the interaction matrix has ranq = 5): Used to grasp
        cylindrical object*/
    vs6dof_cyl
  } vpServoArmType;

protected:
  vpFeatureTranslation m_t;
  vpFeatureThetaU m_tu;
  vpGenericFeature m_axis;
  double m_lambda;
  vpServoArmType m_type;

public:

  vpServoArm(vpServoArmType n = vpServoArm::vs6dof);
  virtual ~vpServoArm() {
    m_task.kill();
  }

  vpColVector computeControlLaw()
  {
    m_task.print();
    return ( m_task.computeControlLaw() );
  }
  vpColVector computeControlLaw(double servo_time_init)
  {
    return ( m_task.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init) );
  }
  vpMatrix getTaskJacobian() {return m_task.getTaskJacobian();}
  vpMatrix getTaskJacobianPseudoInverse() {return m_task.getTaskJacobianPseudoInverse();}
  vpServoArm::vpServoArmType getServoArmType(){return m_type;}

  vpColVector getError(){return m_task.getError();}

  void set_eJe(const vpMatrix &eJe) { m_task.set_eJe(eJe);}
  void set_cVf(const vpVelocityTwistMatrix &cVf) { m_task.set_cVf(cVf);}
  void set_fVe(const vpVelocityTwistMatrix &fVe) { m_task.set_fVe(fVe);}
  void setCurrentFeature(const vpHomogeneousMatrix &cdMc)
  {
    m_t.buildFrom(cdMc) ;
    m_tu.buildFrom(cdMc) ;
  }
  void setCurrentFeature(const vpHomogeneousMatrix &cdMc, const vpColVector &z_c, const vpColVector &z_d);
  void setLambda(double lambda) {m_task.setLambda(lambda);}
  void setLambda(const vpAdaptiveGain &lambda) {m_task.setLambda(lambda);}

};

#endif

