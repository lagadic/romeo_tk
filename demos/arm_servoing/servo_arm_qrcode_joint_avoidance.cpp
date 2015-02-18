/**
 *
 * I want to test the joint avoidance limit using the new operator. We learn a desired position from the qrcode and after we perform a visual servoing to the
 * desired position.
 *
 */

#include <iostream>
#include <string>
#include <list>
#include <iterator>

// Aldebaran includes.
#include <alproxies/altexttospeechproxy.h>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpPlot.h>
#include <visp/vpPoint.h>


#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>

#include <vpQRCodeTracker.h>
#include <vpServoArm.h>
#include <vpRomeoTkConfig.h>

using namespace AL;


double sigmoidFunction(const vpColVector & e)
{
  double e0 = 0.1;
  double e1 = 0.9;
  double sig = 0.0;

  double norm_e = e.euclideanNorm() ;

  if (norm_e > e1)
    sig = 1.0;
  else if (e0 <= norm_e && norm_e <= e1 )
    sig = 1.0 / (1.0 + exp(-12.0 * ( (norm_e-e0)/((e1-e0))) + 6.0 ) );
  else
    sig = 0.0;

  return sig;
}

vpMatrix computeP(const vpColVector & e, const vpMatrix & J, const vpMatrix & J_pinv,const int & n)
{

  vpMatrix P(n,n);

  vpMatrix I(n,n);
  I.setIdentity();

  vpMatrix P_e(n,n);
  P_e =  I - J_pinv * J; // vpMatrix 	I_WpW in Visp vpServo

  double pp = (e.transpose() * J * J.transpose() * e)[0];

  vpMatrix  ee_t(n,n);
  ee_t =  e * e.t();


  vpMatrix P_norm_e(n,n);
  P_norm_e = I - (1.0 / pp ) * J.transpose() * ee_t * J;

  P = sigmoidFunction(e) * P_norm_e + (1 - sigmoidFunction(e)) * P_e;

  //
  std::cout << "P: " << std::endl << P << std::endl;

  return P;

}



vpColVector computeQsec(const vpMatrix &P, const vpColVector &jointMin, const vpColVector &jointMax,  const vpColVector & q , const vpColVector & q1, const double & ro,const double & ro1 )
{


  double lambda = 0.8;
  double lambda_l = 0.0;

  int n = q.size();
  vpColVector q2 (n);


  // Computation of gi ([nx1] vector) and lambda_l ([nx1] vector)
  vpMatrix g(n,n);
  vpColVector q2_i(n);

  vpColVector q_l0_min(n);
  vpColVector q_l0_max(n);
  vpColVector q_l1_min(n);
  vpColVector q_l1_max(n);


  for(unsigned int i = 0; i < n; i++)
  {
    double qmin = jointMin[i];
    double qmax = jointMax[i];

    q_l0_min[i] = qmin + ro *(qmax - qmin);
    q_l0_max[i] = qmax - ro *(qmax - qmin);

    q_l1_min[i] =  q_l0_min[i] - ro * ro1 * (qmax - qmin);
    q_l1_max[i] =  q_l0_max[i] + ro * ro1 * (qmax - qmin);

    if (q[i] < q_l0_min[i] )
      g[i][i] = -1;
    else if (q[i] > q_l0_max[i] )
      g[i][i] = 1;
    else
      g[i][i]= 0;

  }

  //  std::cout << "------------" << std::endl;
  //  std::cout << "g: " << std::endl << g << std::endl;
  // std::cout << "------------" << std::endl;

  // std::cout << "***************************" << std::endl;
  for(unsigned int i = 0; i < n; i++)
  {
    //    std::cout << "ITERATION : " << std::endl << i << std::endl;

    if (q[i] > q_l0_min[i] && q[i] < q_l0_max[i])
    {
      // std::cout << "---- caso 4 : zero "  << std::endl;
      q2_i = 0 * q2_i;
    }

    else
    {
      vpColVector Pg_i(n);
      Pg_i = (P * g.getCol(i));
      double b = ( vpMath::abs(q1[i]) )/( vpMath::abs( Pg_i[i] ) );

      if (q[i] < q_l1_min[i] || q[i] > q_l1_max[i] )
      {
        q2_i = - (1 + lambda) * b * Pg_i;
        //std::cout << "---- caso 1 "  << std::endl;
        // std::cout << "b: " << std::endl << b << std::endl;
        // std::cout << "Pg_i: " << std::endl << Pg_i << std::endl;
      }

      else
      {
        if (q[i] >= q_l0_max[i] && q[i] <= q_l1_max[i] )
        {
          //std::cout << "---- caso 2"  << std::endl;
          lambda_l = 1 / (1 + exp(-12 *( (q[i] - q_l0_max[i]) / (q_l1_min[i] - q_l0_max[i])  ) + 6 ) );
        }
        else if (q[i] >= q_l1_min[i] && q[i] < q_l0_min[i])
        {
          lambda_l = 1 / (1 + exp(-12 *( (q[i] - q_l0_min[i]) / (q_l1_min[i] - q_l0_min[i])  ) + 6 ) );
          //std::cout << "---- caso 3 "  << std::endl;
        }

        q2_i = - lambda_l * (1 + lambda)* b * Pg_i;
      }

    }
    q2 = q2 + q2_i;

    // std::cout << "q2_i: " << std::endl << q2_i << std::endl;

  }

  std::cout << "***************************" << std::endl;

  return q2;

}




vpColVector computeQdotLimitAvoidance(const vpColVector & e, const vpMatrix & J, const vpMatrix & J_pinv,const vpColVector & jointMin,const vpColVector & jointMax, const vpColVector & q , const vpColVector & q1,const double ro,const double ro1)
{
  int n = q.size();
  // Computation Projector operator P
  vpMatrix P(n,n);
  P = computeP(e, J, J_pinv, n);

  // Computation secondary task (q2)
  vpColVector q_sec(n);

  q_sec = computeQsec(P, jointMin, jointMax, q, q1, ro, ro1 );


  return q_sec;
}





void printPose(const std::string &text, const vpHomogeneousMatrix &cMo)
{
  vpTranslationVector t;
  cMo.extract(t);
  vpRotationMatrix R;
  cMo.extract(R);
  vpThetaUVector tu(R);

  std::cout << text;
  for (unsigned int i=0; i < 3; i++)
    std::cout << t[i] << " ";
  for (unsigned int i=0; i < 3; i++)
    std::cout << vpMath::deg(tu[i]) << " ";
  std::cout << std::endl;
}

int main(int argc, const char* argv[])
{



  //  // TO TEST

  //  int n = 4;
  //  double k = 0.0;
  //  vpMatrix J(n,n);
  //  for (unsigned int i = 0; i < n; i++)
  //    for (unsigned int j = 0; j < n; j++)
  //    {
  //      J[i][j] = k;
  //      k = k +1.0;
  //    }
  //  vpColVector e(4);

  //  for (unsigned int i = 0; i < n; i++)
  //    e[i] = double(i);
  //  //e[i] = 0.0001;


  //  vpColVector jointMin(n) ;
  //  for (unsigned int i = 0; i < n; i++)
  //    jointMin[i] = -0.5;


  //  vpColVector jointMax(n) ;
  //  for (unsigned int i = 0; i < n; i++)
  //    jointMax[i] = 0.3;

  //  vpColVector q(n) ;
  //  for (unsigned int i = 0; i < n; i++)
  //    q[i] = 0.1 * i;

  //  vpColVector q1(n) ;
  //  for (unsigned int i = 0; i < n; i++)
  //    q1[i] = 0.2 * i;

  //  std::cout << "J: " << std::endl << J << std::endl;
  //  std::cout << "e: " << std::endl << e << std::endl;
  //  std::cout << "jointMin: " << std::endl << jointMin << std::endl;
  //  std::cout << "jointMax: " << std::endl << jointMax << std::endl;
  //  std::cout << "q: " << std::endl << q << std::endl;
  //  std::cout << "q1: " << std::endl << q1 << std::endl;

  //  vpColVector q2 (n);
  //  q2 =  computeQdotLimitAvoidance(e, J, J.pseudoInverse(),jointMin, jointMax, q, q1);

  //  std::cout << "q2: " << std::endl << q2 << std::endl;

  //  return 0;






  std::string opt_ip = "198.18.0.1";;
  bool opt_plotter_arm = false;
  bool opt_plotter_qrcode_pose = false;
  bool opt_learn = false;
  bool opt_plotter_q_sec_arm = false;
  bool opt_plotter_q = false;

  std::string learned_filename = "learned_cdMo.xml";
  std::string learned_transform_name = "cdMo";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--learn")
      opt_learn = true;
    else if (std::string(argv[i]) == "--plot-arm")
      opt_plotter_arm = true;
    else if (std::string(argv[i]) == "--plot-qrcode-pose")
      opt_plotter_qrcode_pose = true;
    else if (std::string(argv[i]) == "--plot-q-sec-arm")
      opt_plotter_q_sec_arm = true;
    else if (std::string(argv[i]) == "--plot-q")
      opt_plotter_q = true;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn] [--plot-arm] [--plot-qrcode-pose] [--plot-q-sec-arm] [--plot-q] [--help]" << std::endl;
      return 0;
    }
  }

  // Check if the desired position was learned
  if (!opt_learn) {
    if (! vpIoTools::checkFilename(learned_filename)) {
      std::cout << "\nError: You should first learn the desired position using [--learn] option." << std::endl;
      std::cout << "\nRun: \"" << argv[0] << " --help\" to get all the options.\n" <<  std::endl;
      return 0;
    }
  }


  /** Open the grabber for the acquisition of the images from the robot*/
  vpNaoqiGrabber g;
  g.setFramerate(15);
  g.setCamera(0);
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.open();

  vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithoutDistortion);
  std::cout << "Camera parameters: " << cam << std::endl;

  /** Create a new istance NaoqiRobot*/
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  robot.open();

  /** Initialization Visp Image, display and camera paramenters*/
  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "Right camera view");

  // Initialize constant transformations
  vpHomogeneousMatrix eMc = g.get_eMc();

  // Initialize the qrcode tracker
  bool status_qrcode_tracker;
  vpHomogeneousMatrix cMo_qrcode;
  vpQRCodeTracker qrcode_tracker;
  qrcode_tracker.setCameraParameters(cam);
  qrcode_tracker.setQRCodeSize(0.035);


  // Constant transformation Target Frame to LArm end-effector (LWristPitch)
  vpHomogeneousMatrix oMe_LArm;

  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  std::string name_transform = "qrcode_M_e_LArm";
  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  if (pm.parse(oMe_LArm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
    return 0;
  }
  else
    std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_LArm << std::endl;

  bool grasp_servo_converged = false;
  vpServoArm servo_larm; // Initialize arm servoing
  std::vector<std::string> jointNames_larm =  robot.getBodyNames("LArm");
  jointNames_larm.pop_back(); // Delete last joints LHand, that we don't consider in the servo
  vpVelocityTwistMatrix oVe_LArm(oMe_LArm);

  int numJoints = jointNames_larm.size();

  // Initialize the joint avoidance scheme from the joint limits
  vpColVector jointMin = robot.getJointMin("LArm");
  vpColVector jointMax = robot.getJointMax("LArm");
  // Vector secondary task
  vpColVector q2 (numJoints);

  // Vector of joint positions
  vpColVector q;

  //Vector data for plotting
  vpColVector data(13);
  vpColVector Qmiddle(numJoints);

  // Joints limits and paramenters
  std::cout << "Joint limits arm: " << std::endl;
  for (unsigned int i=0; i< numJoints; i++)
  {
    Qmiddle[i] = ( jointMin[i] + jointMax[i]) /2.;
    std::cout << " Joint " << i << " " << jointNames_larm[i]
                 << ": min=" << vpMath::deg(jointMin[i])
                 << " max=" << vpMath::deg(jointMax[i]) << std::endl;
  }
  double ro = 0.1;
  double ro1 = 0.5;


  // Initialize arm open loop servoing
  double servo_time_init = 0;

  //Set the stiffness
  robot.setStiffness(jointNames_larm, 1.f);
  vpColVector q_dot_larm(numJoints, 0);

  // Common
  vpMouseButton::vpMouseButtonType button;
  unsigned long loop_iter = 0;

  // Plotter
  vpPlot *plotter_arm;
  if (opt_plotter_arm) {
    plotter_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+80, I.display->getWindowYPosition(), "Visual servoing");
    plotter_arm->initGraph(0, 6); // visual features
    plotter_arm->initGraph(1, q_dot_larm.size()); // d_dot
    plotter_arm->setTitle(0, "Visual features error");
    plotter_arm->setTitle(1, "joint velocities");
    plotter_arm->setLegend(0, 0, "tx");
    plotter_arm->setLegend(0, 1, "ty");
    plotter_arm->setLegend(0, 2, "tz");
    plotter_arm->setLegend(0, 3, "tux");
    plotter_arm->setLegend(0, 4, "tuy");
    plotter_arm->setLegend(0, 5, "tuz");

  }
  vpPlot *plotter_qrcode_pose;
  if (opt_plotter_qrcode_pose) {
    plotter_qrcode_pose = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Qrcode pose");
    plotter_qrcode_pose->initGraph(0, 3); // translations
    plotter_qrcode_pose->initGraph(1, 3); // rotations
    plotter_qrcode_pose->setTitle(0, "Pose translation");
    plotter_qrcode_pose->setTitle(1, "Pose theta u");
    plotter_qrcode_pose->setLegend(0, 0, "tx");
    plotter_qrcode_pose->setLegend(0, 1, "ty");
    plotter_qrcode_pose->setLegend(0, 2, "tz");
    plotter_qrcode_pose->setLegend(1, 0, "tux");
    plotter_qrcode_pose->setLegend(1, 1, "tuy");
    plotter_qrcode_pose->setLegend(1, 2, "tuz");
  }

  vpPlot *plotter_q_sec_arm;
  if (opt_plotter_q_sec_arm) {
    plotter_q_sec_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+30, "Secondary Task");
    plotter_q_sec_arm->initGraph(0, 7); // translations
    plotter_q_sec_arm->initGraph(1, 7); // rotations

    plotter_q_sec_arm->setTitle(0, "DQ2 values");
    plotter_q_sec_arm->setTitle(1, "DQ tot values");

    plotter_q_sec_arm->setLegend(0, 0, "LshouderPitch");
    plotter_q_sec_arm->setLegend(0, 1, "LShoulderYaw");
    plotter_q_sec_arm->setLegend(0, 2, "LElbowRoll");
    plotter_q_sec_arm->setLegend(0, 3, "LElbowYaw");
    plotter_q_sec_arm->setLegend(0, 4, "LWristRoll");
    plotter_q_sec_arm->setLegend(0, 5, "LWristYaw");
    plotter_q_sec_arm->setLegend(0, 6, "LWristPitch");

    plotter_q_sec_arm->setLegend(1, 0, "LshouderPitch");
    plotter_q_sec_arm->setLegend(1, 1, "LShoulderYaw");
    plotter_q_sec_arm->setLegend(1, 2, "LElbowRoll");
    plotter_q_sec_arm->setLegend(1, 3, "LElbowYaw");
    plotter_q_sec_arm->setLegend(1, 4, "LWristRoll");
    plotter_q_sec_arm->setLegend(1, 5, "LWristYaw");
    plotter_q_sec_arm->setLegend(1, 6, "LWristPitch");

  }


  vpPlot *plotter_q;
  if (opt_plotter_q) {

    plotter_q = new vpPlot(1, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+60, "Values of q and limits");
    plotter_q->initGraph(0, 13);

    plotter_q->setTitle(0, "Q1 values");

    plotter_q->setLegend(0, 0, "LshouderPitch");
    plotter_q->setLegend(0, 1, "LShoulderYaw");
    plotter_q->setLegend(0, 2, "LElbowRoll");
    plotter_q->setLegend(0, 3, "LElbowYaw");
    plotter_q->setLegend(0, 4, "LWristRoll");
    plotter_q->setLegend(0, 5, "LWristYaw");
    plotter_q->setLegend(0, 6, "LWristPitch");

    plotter_q->setLegend(0, 7, "Low Limits");
    plotter_q->setLegend(0, 8, "Upper Limits");
    plotter_q->setLegend(0, 9, "l0 min");
    plotter_q->setLegend(0, 10, "l0 max");
    plotter_q->setLegend(0, 11, "l1 min");
    plotter_q->setLegend(0, 12, "l1 max");



  }




  vpHomogeneousMatrix cdMo_learned;

  if (! opt_learn) {
    vpXmlParserHomogeneousMatrix pm;
    if (pm.parse(cdMo_learned, learned_filename, learned_transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << learned_transform_name<< "." << std::endl;
      return false;
    }
    else
      std::cout << "Homogeneous matrix " << learned_transform_name <<": " << std::endl << cdMo_learned << std::endl;
  }

  std::vector<float> head_pose, larm_pose, larm_new_pose;
  AL::ALValue LShoulderYaw_limits;
  float shoulderYaw_pos ;


  // Move the head in the default position

  AL::ALValue names_head       = AL::ALValue::array("HeadPitch","HeadRoll", "NeckPitch", "NeckYaw");
  AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(15), vpMath::rad(0), vpMath::rad(10), vpMath::rad(6));
  float fractionMaxSpeed  = 0.2f;
  robot.getProxy()->setStiffnesses(names_head, AL::ALValue::array(1.0f, 1.0f, 1.0f, 1.0f));
  qi::os::sleep(1.0f);
  robot.getProxy()->setAngles(names_head, angles_head, fractionMaxSpeed);
  qi::os::sleep(2.0f);


  while(1) {
    double loop_time_start = vpTime::measureTimeMs();
    g.acquire(I);
    vpDisplay::display(I);

    vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Right click to quit", vpColor::red);

    bool click_done = vpDisplay::getClick(I, button, false);

    // track qrcode
    status_qrcode_tracker = qrcode_tracker.track(I);

    if (status_qrcode_tracker) { // display the tracking results
      cMo_qrcode = qrcode_tracker.get_cMo();
      printPose("cMo qrcode: ", cMo_qrcode);
      // The qrcode frame is only displayed when PBVS is active or learning
      vpDisplay::displayFrame(I, cMo_qrcode, cam, 0.04, vpColor::none, 3);
      vpDisplay::displayPolygon(I, qrcode_tracker.getCorners(), vpColor::green, 2);
    }

    // learn the desired position
    if (status_qrcode_tracker && opt_learn) {
      vpDisplay::displayText(I, 10, 10, "Left click to learn desired position", vpColor::red);
      if (click_done && button == vpMouseButton::button1) {
        vpXmlParserHomogeneousMatrix p; // Create a XML parser
        cdMo_learned = qrcode_tracker.get_cMo();

        if (p.save(cdMo_learned, learned_filename, learned_transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
        {
          std::cout << "Cannot save the homogeneous matrix cdMo" << std::endl;
          return false;
        }
        printPose("Learned pose: ", cdMo_learned);
        return 0;
      }
    }

    // Visual servo of the head centering teabox and qrcode
    if (status_qrcode_tracker && !opt_learn) {
      static bool first_time = true;
      if (first_time) {
        std::cout << "-- Start visual servoing of the arm" << std::endl;
        servo_time_init = vpTime::measureTimeSecond();
        first_time = false;
      }

      // Servo arm
      if (! grasp_servo_converged) {
        vpMatrix oJo = oVe_LArm * robot.get_eJe("LArm");
        //servo_larm.setLambda(0.2);
        vpAdaptiveGain lambda(1.6, .6, 20);

        servo_larm.set_eJe(oJo);
        vpHomogeneousMatrix torsoMHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));
        vpVelocityTwistMatrix cVtorso( (torsoMHeadRoll * eMc).inverse());
        servo_larm.set_cVf( cVtorso );
        vpHomogeneousMatrix torsoMLWristPitch(robot.getProxy()->getTransform("LWristPitch", 0, true));
        vpVelocityTwistMatrix torsoVo(torsoMLWristPitch * oMe_LArm.inverse());
        servo_larm.set_fVe( torsoVo );


        vpHomogeneousMatrix cdMc = cdMo_learned * /*oMh_Tea_Box_grasp * */ cMo_qrcode.inverse() ;
        printPose("cdMc: ", cdMc);
        servo_larm.setCurrentFeature(cdMc) ;

        vpDisplay::displayFrame(I, cdMo_learned /* * oMh_Tea_Box_grasp */, cam, 0.025, vpColor::none, 2);

        q_dot_larm = servo_larm.computeControlLaw(servo_time_init);
        std::cout << "Vel arm: " << q_dot_larm.t() << std::endl;

        vpColVector e = servo_larm.getError();
        vpMatrix TaskJac = servo_larm.getTaskJacobian();
        vpMatrix TaskJacPseudoInv = servo_larm.getTaskJacobianPseudoInverse();

        q = robot.getPosition(jointNames_larm);

        q2 = computeQdotLimitAvoidance(e, TaskJac, TaskJacPseudoInv, jointMin, jointMax, q, q_dot_larm,ro,ro1);

        std::cout << "q2: " << std::endl << q2 << std::endl;
        robot.setVelocity(jointNames_larm, q_dot_larm +q2);
        //robot.setVelocity(jointNames_larm, q_dot_larm);

        //        vpDisplay::flush(I);
        //        vpDisplay::getClick(I);
      }
    }
    else if(! status_qrcode_tracker) {
      robot.stop(jointNames_larm);
    }

    if (click_done && button == vpMouseButton::button3) { // Quit the loop
      robot.stop(jointNames_larm);
      break;
    }

    double loop_time = vpTime::measureTimeMs() - loop_time_start;

    if (opt_plotter_arm && ! opt_learn) {
      plotter_arm->plot(0, loop_iter, servo_larm.m_task.getError());
      plotter_arm->plot(1, loop_iter, q_dot_larm);
    }

    if (opt_plotter_qrcode_pose && status_qrcode_tracker) {
      vpPoseVector p(cMo_qrcode);
      vpColVector cto(3);
      vpColVector cthetauo(3);
      for(size_t i=0; i<3; i++) {
        cto[i] = p[i];
        cthetauo[i] = vpMath::deg(p[i+3]);
      }

      plotter_qrcode_pose->plot(0, loop_iter, cto);
      plotter_qrcode_pose->plot(1, loop_iter, cthetauo);
    }

    if (opt_plotter_q_sec_arm  && status_qrcode_tracker)
    {
      plotter_q_sec_arm->plot(0,loop_iter,q2);
      plotter_q_sec_arm->plot(1,loop_iter,q_dot_larm + q2);

    }

    if (opt_plotter_q  && status_qrcode_tracker)
    {

      // q normalized between (entre -1 et 1)
      for (unsigned int i=0 ; i < numJoints ; i++) {
        data[i] = (q[i] - Qmiddle[i]) ;
        data[i] /= (jointMax[i] - jointMin[i]) ;
        data[i]*=2 ;
      }



      data[numJoints] = -1.0;
      data[numJoints+1] = 1.0;

      data[numJoints+2] = -1 + ro * 2;
      data[numJoints+3] = 1 - ro * 2;

      data[numJoints+4] =  data[numJoints+2] - ro * ro1 * 2;
      data[numJoints+5] =  data[numJoints+3] + ro * ro1 * 2;

      plotter_q->plot(0,0,loop_iter,data[0]);
      plotter_q->plot(0,1,loop_iter,data[1]);
      plotter_q->plot(0,2,loop_iter,data[2]);
      plotter_q->plot(0,3,loop_iter,data[3]);
      plotter_q->plot(0,4,loop_iter,data[4]);
      plotter_q->plot(0,5,loop_iter,data[5]);
      plotter_q->plot(0,6,loop_iter,data[6]);

      plotter_q->plot(0,7,loop_iter,data[7]);
      plotter_q->plot(0,8,loop_iter,data[8]);

      plotter_q->plot(0,9,loop_iter,data[9]);
      plotter_q->plot(0,10,loop_iter,data[10]);
      plotter_q->plot(0,11,loop_iter,data[11]);
      plotter_q->plot(0,12,loop_iter,data[12]);


    }


    vpDisplay::flush(I) ;
    //std::cout << "Loop time: " << vpTime::measureTimeMs() - loop_time_start << std::endl;

    loop_iter ++;
  }


  while(1)
  {

    bool click_done = vpDisplay::getClick(I, button, false);

    if (click_done && button == vpMouseButton::button3) { // Quit the loop
      break;
    }

  }


  if (opt_plotter_arm)
    delete plotter_arm;
  if (opt_plotter_qrcode_pose)
    delete plotter_qrcode_pose;
  if (opt_plotter_q)
    delete plotter_q;
  if (opt_plotter_q_sec_arm)
    delete plotter_q_sec_arm;



  return 0;
}

