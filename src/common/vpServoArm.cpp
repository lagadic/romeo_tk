
#include <vpServoArm.h>


vpServoArm::vpServoArm() : m_t(vpFeatureTranslation::cdMc), m_tu(vpFeatureThetaU::cdRc),
  m_lambda(0.1)
{
  // We want to see a point on a point
  m_task.addFeature(m_t) ;   // 3D translation
  m_task.addFeature(m_tu) ; // 3D rotation
  m_task.setServo(vpServo::EYETOHAND_L_cVe_eJe);// EYETOHAND_L_cVf_fVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
  m_task.setLambda(m_lambda);
  //  // Set the proportional gain
  //  vpAdaptiveGain  lambda;
  //  lambda.initStandard(2, 0.2, 50);
  //  m_task.setLambda(lambda) ;
}

