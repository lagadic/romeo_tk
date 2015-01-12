#ifndef __vpServoHead_h__
#define __vpServoHead_h__

#include <visp/vpCameraParameters.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpServoDisplay.h>


class vpServoHead
{
public:
  vpServo m_task_head; // Visual servoing task
protected:
  vpFeaturePoint m_sd; //The desired point feature.

  //Set the desired features x and y
  double m_xd;
  double m_yd;
  //Set the depth of the point in the camera frame.
  double m_Zd;
  vpFeaturePoint m_s; //The current point feature.
  //Set the current features x and y
  double m_x; //You have to compute the value of x.
  double m_y; //You have to compute the value of y.
  double m_Z; //You have to compute the value of Z.
  vpCameraParameters m_cam;

public:
  vpServoHead();
  virtual ~vpServoHead()
  {
    m_task_head.kill();
  }
  vpColVector computeControlLaw()
  {
    return ( m_task_head.computeControlLaw() );
  }
  vpColVector computeControlLaw(double servo_time_init)
  {
    return ( m_task_head.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init) );
  }
  void set_eJe(const vpMatrix &eJe) { m_task_head.set_eJe(eJe);}
  void set_cVe(const vpVelocityTwistMatrix &cVe) { m_task_head.set_cVe(cVe);}
  void setCameraParameters(const vpCameraParameters &cam) {m_cam = cam;}
  void setCurrentFeature(const vpImagePoint &ip)
  {
    vpPixelMeterConversion::convertPoint(m_cam, ip, m_x, m_y);
    m_s.buildFrom(m_x, m_y, m_Z);
  }
  void setDesiredFeature(const vpImagePoint &ip)
  {
    vpPixelMeterConversion::convertPoint(m_cam, ip, m_xd, m_yd);
    m_sd.buildFrom(m_xd, m_yd, m_Zd);
  }
  void setLambda(double lambda) {m_task_head.setLambda(lambda);}
  void setLambda(const vpAdaptiveGain &lambda) {m_task_head.setLambda(lambda);}
};

#endif
