/**
 *
 * This example demonstrates how to get images from the robot remotely, how
 * to track a blob using all the four joints of the Romeo Head;
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



// Todo
// - thread when romeo motion blocking
// - check if the arm is in rest position. If not, stop the demo, or better if possible
// - set the desired position of the point used for IBVS depending on the object.cao file (Fabien)
// - add copy constructor and operator in vpServo (Fabien)
// - continue to servo the head always. Requires to have a non blocking open loop motion of the arm
// - add getTranslationVector() to vpHomogeneousMatrix()
// - modify moveToDesiredLHandPosition() to check if the file and transform exists at the beginning of the demo

using namespace AL;


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
  std::string opt_ip = "198.18.0.1";;
  bool opt_plotter_arm = false;
  bool opt_plotter_qrcode_pose = false;
  bool opt_learn = false;

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
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn] [--plot-arm] [--plot-qrcode-pose] [--help]" << std::endl;
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

  // Initialize arm open loop servoing
  bool arm_moved = false;
  double servo_time_init = 0;

  //Set the stiffness
  robot.setStiffness(jointNames_larm, 1.f);
  vpColVector q_dot_larm(jointNames_larm.size(), 0);

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


  while(1) {
    double loop_time_start = vpTime::measureTimeMs();
    g.acquire(I);
    vpDisplay::display(I);

    vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Right click to quit", vpColor::red);

    bool click_done = vpDisplay::getClick(I, button, false);

    if (click_done && button == vpMouseButton::button1) {
      std::vector<float> LArmShoulderElbowYaw_pos;
      LArmShoulderElbowYaw_pos.push_back(shoulderYaw_pos);
      LArmShoulderElbowYaw_pos.push_back(vpMath::rad(-40.));
      std::vector<std::string> LArmShoulderElbowYaw_name;
      LArmShoulderElbowYaw_name.push_back("LShoulderYaw");
      LArmShoulderElbowYaw_name.push_back("LElbowYaw");

     robot.getProxy()->setAngles(LArmShoulderElbowYaw_name, LArmShoulderElbowYaw_pos, 0.08);

      click_done = false;
    }


#if 0

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
        servo_larm.setLambda(0.1);
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
        robot.setVelocity(jointNames_larm, q_dot_larm);

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
#endif

    vpDisplay::flush(I) ;
    //std::cout << "Loop time: " << vpTime::measureTimeMs() - loop_time_start << std::endl;

    loop_iter ++;
  }

  if (opt_plotter_arm)
    delete plotter_arm;
  if (opt_plotter_qrcode_pose)
    delete plotter_qrcode_pose;

  return 0;
}

