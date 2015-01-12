
#include <vpServoHead.h>


vpServoHead::vpServoHead(): m_xd(0), m_yd(0), m_Zd(0.8), m_x(0), m_y(0), m_Z(0.8),
  m_cam()
{
  //Set the point feature thanks to the current parameters.
  m_s.buildFrom(m_x, m_y, m_Z);

  //Set the point feature thanks to the desired parameters.
  m_sd.buildFrom(m_xd, m_yd, m_Zd);

  //In this case the parameter Z is not necessary because the interaction matrix is computed
  //with the desired visual feature.
  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  m_task_head.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  m_task_head.setInteractionMatrixType(vpServo::DESIRED);
  // Add the 2D point feature to the task
  m_task_head.addFeature(m_s, m_sd);

  //vpAdaptiveGain lambda_head(2, 0.1, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
  //tm_ask_head.setLambda(lambda_head);
  m_task_head.setLambda(0.2);
}
