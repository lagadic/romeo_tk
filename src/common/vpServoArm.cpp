#include <vpServoArm.h>


vpServoArm::vpServoArm(vpServoArmType n) : m_t(), m_tu(),
  m_lambda(0.1)
{

  if (n == vpServoArm::vs6dof)
  {

    m_t.setFeatureTranslationType(vpFeatureTranslation::cdMc);
    m_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);

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
else if (n == vpServoArm::vs5dof_cyl)
  {
    m_t.setFeatureTranslationType(vpFeatureTranslation::cdMc);
    m_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);

    // We want to see a point on a point
    m_task.addFeature(m_t) ;   // 3D translation
    m_task.addFeature(m_tu, vpFeatureThetaU::selectTUx() | vpFeatureThetaU::selectTUy()) ; // 3D rotation
    m_task.setServo(vpServo::EYETOHAND_L_cVe_eJe);// EYETOHAND_L_cVf_fVe_eJe);
    // Interaction matrix is computed with the desired visual features sd
    m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    m_task.setLambda(m_lambda);


  }


}

