#include <vpServoArm.h>


vpServoArm::vpServoArm(vpServoArmType n) : m_t(), m_tu(), m_axis(3),
  m_lambda(0.1), m_type(n)
{

  if (m_type == vpServoArm::vs6dof)
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
  else if (m_type == vpServoArm::vs5dof_cyl)
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
  else if (m_type == vpServoArm::vs6dof_cyl)
  {
    m_t.setFeatureTranslationType(vpFeatureTranslation::cdMc);

    // We want to see a point on a point
    m_task.addFeature(m_t) ;   // 3D translation
    m_task.addFeature(m_axis); // z axis

    m_task.setServo(vpServo::EYETOHAND_L_cVe_eJe);// EYETOHAND_L_cVf_fVe_eJe);
    // Interaction matrix is computed with the desired visual features sd
    m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    m_task.setLambda(m_lambda);

  }


}

void vpServoArm::setCurrentFeature(const vpHomogeneousMatrix &cdMc, const vpColVector &z_c, const vpColVector &z_d)
{
  m_t.buildFrom(cdMc) ;
  vpColVector e;
  e = vpColVector::crossProd(z_c,z_d);
  //std::cout << "e" << std::endl << e << std::endl;
  m_axis.set_s(e);
 // m_axis.setError(e);
  vpMatrix L_axis(3,3);
  L_axis =  -vpColVector::skew(z_d) * vpColVector::skew(z_c);
  vpMatrix Ls(3,6);
  Ls.insert(-L_axis,0,3);
//    std::cout << "L_axis" << std::endl << L_axis << std::endl;
//    std::cout << "Ls" << std::endl << Ls << std::endl;
  m_axis.setInteractionMatrix(Ls);

}
