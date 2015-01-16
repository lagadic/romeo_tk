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

#include <vpFaceTracker.h>
#include <vpQRCodeTracker.h>
#include <vpServoHead.h>
#include <vpServoArm.h>
#include <vpCartesianDisplacement.h>
#include <vpRomeoTkConfig.h>

#define SAVE 0

#define USE_PLOTTER
#define L 0.015


// Todo
// - thread when romeo motion blocking
// - check if the arm is in rest position. If not, stop the demo, or better if possible
// - set the desired position of the point used for IBVS depending on the object.cao file (Fabien)
// - add copy constructor and operator in vpServo (Fabien)
// - continue to servo the head always. Requires to have a non blocking open loop motion of the arm
// - add getTranslationVector() to vpHomogeneousMatrix()
// - modify moveToDesiredLHandPosition() to check if the file and transform exists at the beginning of the demo

using namespace AL;

typedef enum {
  Acquisition,
  InitTeaBoxTracking,
  WaitTeaBoxTracking,
  TeaBoxTracking,
  MoveArm,
  LearnDesiredLHandOpenLoopPosition,
  LearnDesiredLHandGraspPosition,
  MoveToDesiredLHandPosition,
  WaitGrasping, // wait for a click to start grasping
  Grasping,
  TakeTea,
  Interaction
} StateTeaboxTracker_t;


/*!
  Move LArm in position to start a grapsing demo avoiding the table.
*/
void moveLArmFromOrToRestPosition(const vpNaoqiRobot &robot, bool up)
{
  try
  {
    AL::ALValue pos0 = AL::ALValue::array(0.12116464972496033, 0.2008720338344574, -0.3022586703300476, -2.055604934692383, 1.1719822883605957, -0.6304683089256287);
    AL::ALValue time0 = 2.0f;
    AL::ALValue pos1 = AL::ALValue::array(0.2852230966091156, 0.3805413246154785, -0.17208018898963928, -1.4664039611816406, 0.28257742524147034, 0.17258954048156738);
    AL::ALValue time1 = 3.5f;
    AL::ALValue pos2 = AL::ALValue::array(0.3599576950073242, 0.3060062527656555, 0.01953596994280815, -1.1513646841049194, -0.18644022941589355, -0.1889418214559555);
    AL::ALValue time2 = 5.5f;

    AL::ALValue path;
    if (up) {
      path.arrayPush(pos0);
      path.arrayPush(pos1);
      path.arrayPush(pos2);
    }
    else {
      path.arrayPush(pos2);
      path.arrayPush(pos1);
      path.arrayPush(pos0);
    }

    AL::ALValue times;
    times.arrayPush(time0);
    times.arrayPush(time1);
    times.arrayPush(time2);

    AL::ALValue chainName  = AL::ALValue::array ("LArm");
    AL::ALValue space      = AL::ALValue::array (0); // Torso
    AL::ALValue axisMask   =  AL::ALValue::array (63);

    robot.getProxy()->positionInterpolations(chainName, space, path, axisMask, times);

  }
  catch(const std::exception&)
  {
    throw vpRobotException (vpRobotException::badValue,
                            "servo apply the motion");
  }

  return;
}

bool learnDesiredLHandOpenLoopPosition(const vpNaoqiRobot &robot, const vpHomogeneousMatrix &cMo,
                                       const vpHomogeneousMatrix &eMc, const std::string &out_filename,
                                       const std::string &transform_name, vpHomogeneousMatrix &oMh_sens)
{
  // Trasformation from Torso to Hand sensor
  vpHomogeneousMatrix tMh_sens (robot.getProxy()->getTransform("LArm",0,true));

  // Trasformation from Torso to HeadRoll
  vpHomogeneousMatrix torsoMHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));

  vpHomogeneousMatrix torsoMlcam = torsoMHeadRoll * eMc;

  // Now we can compute the transformation between oMh
  oMh_sens = cMo.inverse() * torsoMlcam.inverse() * tMh_sens;

  //std::cout << "Learned target offset oMh_sens:" << std::endl << oMh_sens <<std::endl ;

  vpXmlParserHomogeneousMatrix p_; // Create a XML parser

  if (p_.save(oMh_sens, out_filename, transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
  {
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
    return false;
  }
  return true;
}

bool learnDesiredLHandGraspingPosition(const vpNaoqiRobot &robot, const vpHomogeneousMatrix &cMo_teabox,
                                       const vpHomogeneousMatrix &cMo_qrcode, const std::string &out_filename,
                                       const std::string &transform_name, vpHomogeneousMatrix &oMh_grasp)
{
  oMh_grasp = cMo_teabox.inverse()*cMo_qrcode;

  vpXmlParserHomogeneousMatrix p_; // Create a XML parser

  if (p_.save(oMh_grasp, out_filename, transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
  {
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
    return false;
  }
  return true;
}


bool moveToDesiredLHandPosition(const vpNaoqiRobot &robot, const vpHomogeneousMatrix &cMo,
                                const vpHomogeneousMatrix &eMc, const std::string &in_filename, const std::string &transform_name)
{
  // Load transformation between teabox and desired position of the hand (from sensors) to initializate the tracker
  vpHomogeneousMatrix oMe_lhand;

  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  //  char filename_[FILENAME_MAX];
  //  sprintf(filename_, "%s", VISP_NAOQI_GENERAL_M_FILE);

  if (pm.parse(oMe_lhand, in_filename, transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the Homogeneous matrix named " << transform_name<< "." << std::endl;
    return false;
  }
  else
    std::cout << "Homogeneous matrix " << transform_name <<": " << std::endl << oMe_lhand << std::endl;

  vpHomogeneousMatrix torsoMHeadRoll_(robot.getProxy()->getTransform("HeadRoll", 0, true));
  vpHomogeneousMatrix torsoMlcam_visp_init = torsoMHeadRoll_ * eMc;

  vpHomogeneousMatrix tMh_desired;
  tMh_desired = (oMe_lhand.inverse() * cMo.inverse() * torsoMlcam_visp_init.inverse()).inverse();

  {
    // Hack: TODO remove when naoqi fixed
    vpHomogeneousMatrix Mhack;
    Mhack[1][3] = -0.025; // add Y - 0.025 offset
    tMh_desired = tMh_desired * Mhack;
  }

  std::vector<float> tMh_desired_;
  tMh_desired.convert(tMh_desired_);

  float velocity = 0.2;
  int axis_mask = 63; // Control position and orientation
  robot.getProxy()->setTransform("LArm", 0, tMh_desired_, velocity, axis_mask);
  return true;
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
  std::string opt_ip = "198.18.0.1";;
  //std::string opt_model = std::string(ROMEOTK_DATA_FOLDER) + "/teabox/teabox";
  std::string opt_model = std::string(ROMEOTK_DATA_FOLDER) + "/milkbox/milkbox";
  std::string opt_face_cascade_name = std::string(ROMEOTK_DATA_FOLDER) + "/face/haarcascade_frontalface_alt.xml";

  bool opt_learn_open_loop_position = false;
  bool opt_learn_grasp_position = false;
  bool opt_plotter_time = false;
  bool opt_plotter_arm = false;
  bool opt_plotter_qrcode_pose = false;
  bool opt_interaction = true;
  bool opt_language_english = false;
  bool opt_record_video = false;
  StateTeaboxTracker_t state_teabox_tracker = Acquisition; //TakeTea; //Acquisition; //TakeTea;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--model")
      opt_model = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--learn-open-loop-position")
      opt_learn_open_loop_position = true;
    else if (std::string(argv[i]) == "--learn-grasp-position")
      opt_learn_grasp_position = true;
    else if (std::string(argv[i]) == "--plot-time")
      opt_plotter_time = true;
    else if (std::string(argv[i]) == "--plot-arm")
      opt_plotter_arm = true;
    else if (std::string(argv[i]) == "--plot-qrcode-pose")
      opt_plotter_qrcode_pose = true;
    else if (std::string(argv[i]) == "--no-interaction")
      opt_interaction = false;
    else if (std::string(argv[i]) == "--opt-language-english")
      opt_language_english = true;
    else if (std::string(argv[i]) == "--opt-record-video")
      opt_record_video = true;
    else if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--model <path to mbt cao model>]" << std::endl;
      std::cout << "       [--haar <haarcascade xml filename>] [--no-interaction] [--learn-open-loop-position] " << std::endl;
      std::cout << "       [--learn-grasp-position] [--plot-time] [--plot-arm] [--plot-qrcode-pose] "<< std::endl;
      std::cout << "       [--opt-language-english] [--opt-record-video] [--help]" << std::endl;
      return 0;
    }
  }

  if (opt_interaction && ! vpIoTools::checkFilename(opt_face_cascade_name)) {
    std::cout << "Error: the file " << opt_face_cascade_name <<  " doesn't exist." << std::endl;
    std::cout << "Use --haar <haarcascade xml filename> or --no-interaction to disable face detection " << std::endl;
    return 0;
  }

  vpFaceTracker *face_tracker;
  if (opt_interaction) {
    face_tracker = new vpFaceTracker();
    face_tracker->setFaceCascade(opt_face_cascade_name);
  }

  // Open Proxy for the speech
  AL::ALTextToSpeechProxy tts(opt_ip, 9559);
  if (opt_language_english == false)
    tts.setLanguage("French");
  else
    tts.setLanguage("English");

  std::string phraseToSay = "Hello!";

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

  // Initialize constant parameters
  std::string learned_oMh_filename = "oMh_Tea_Box_offset.xml"; // This file contains the following two transf. matrix:
  std::string name_oMh_open_loop =  "oMh_Tea_Box_open_loop"; // Offset position Hand w.r.t the object (Open loop)
  std::string name_oMh_grasp =  "oMh_Tea_Box_grasp"; // Offset position Hand w.r.t the object to grasp it (Close loop)

  // Initialize the model based tracker
  vpMbEdgeKltTracker teabox_tracker;
  teabox_tracker.loadConfigFile(opt_model + ".xml");
  teabox_tracker.setOgreVisibilityTest(false);
  teabox_tracker.loadModel(opt_model + ".cao");
  teabox_tracker.setDisplayFeatures(true);
  teabox_tracker.setCameraParameters(cam);
  bool status_teabox_tracker = false; // false if the tea box tracker fails
  vpHomogeneousMatrix cMo_teabox;

  // Initialize the qrcode tracker
  bool status_qrcode_tracker;
  vpHomogeneousMatrix cMo_qrcode;
  vpQRCodeTracker qrcode_tracker;
  qrcode_tracker.setCameraParameters(cam);
  qrcode_tracker.setQRCodeSize(0.035);

  // Initialize head servoing
  vpServoHead servo_head;
  servo_head.setCameraParameters(cam);
  bool teabox_servo_converged = false;
  double servo_time_init = 0;
  double servo_head_time_init = 0;
  double servo_arm_time_init = 0;
  std::vector<std::string> jointNames_head =  robot.getBodyNames("Head");

  // Constant transformation Target Frame to LArm end-effector (LWristPitch)
  vpHomogeneousMatrix oMe_LArm;
  vpHomogeneousMatrix cdMo_learned;

  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  std::string name_transform = "qrcode_M_e_LArm";
  {
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMe_LArm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
      return 0;
    }
    else
      std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_LArm << std::endl;
  }

  vpHomogeneousMatrix oMh_Tea_Box_grasp;
  if (! opt_learn_grasp_position && ! opt_learn_open_loop_position) {
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMh_Tea_Box_grasp, learned_oMh_filename, name_oMh_grasp) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << name_oMh_grasp<< "." << std::endl;
      return 0;
    }
    else
      std::cout << "Homogeneous matrix " << name_oMh_grasp <<": " << std::endl << oMh_Tea_Box_grasp << std::endl;
  }

  bool grasp_servo_converged = false;
  vpServoArm servo_larm; // Initialize arm servoing
  std::vector<std::string> jointNames_larm =  robot.getBodyNames("LArm");
  jointNames_larm.pop_back(); // Delete last joints LHand, that we don't consider in the servo
  vpVelocityTwistMatrix oVe_LArm(oMe_LArm);
  vpHomogeneousMatrix cdMc;

  // Initialize arm open loop servoing
  bool arm_moved = false;

  //Set the stiffness
  robot.setStiffness(jointNames_head, 1.f);
  robot.setStiffness(jointNames_larm, 1.f);
  vpColVector q_dot_head(jointNames_head.size(), 0);
  vpColVector q_dot_larm(jointNames_larm.size(), 0);

  // Common
  vpMouseButton::vpMouseButtonType button;
  unsigned long loop_iter = 0;

  // Plotter
  vpPlot *plotter_arm;
  if (opt_plotter_arm) {
    plotter_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+80, I.display->getWindowYPosition(), "Loop time");
    plotter_arm->initGraph(0, 6); // visual features
    plotter_arm->initGraph(1, q_dot_larm.size()); // d_dot
    plotter_arm->setTitle(0, "Visual features error");
    plotter_arm->setTitle(1, "joint velocities");
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
  vpPlot *plotter_time;
  if (opt_plotter_time) {
    plotter_time = new vpPlot(1, I.getHeight(), I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Loop time");
    plotter_time->initGraph(0, 1);
  }


  while(1) {
    double loop_time_start = vpTime::measureTimeMs();
    //std::cout << "Loop iteration: " << loop_iter << std::endl;
    g.acquire(I);
    vpDisplay::display(I);

    vpImageIo::write(I, "milkbox.ppm");

    if (! opt_record_video)
      vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Right click to quit", vpColor::red);

    bool click_done = vpDisplay::getClick(I, button, false);

    // track teabox
    if(state_teabox_tracker == Acquisition) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Click to start the teabox initialization", vpColor::red);

      // Move the head in the default position
    {
      //AL::ALValue names_head       = AL::ALValue::array("HeadPitch","HeadRoll", "NeckPitch", "NeckYaw");
      AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(-23), vpMath::rad(19.2), vpMath::rad(9), vpMath::rad(0) );
      float fractionMaxSpeed  = 0.1f;
      robot.getProxy()->setAngles(jointNames_head, angles_head, fractionMaxSpeed);
     }



      if (click_done && button == vpMouseButton::button1) {
        state_teabox_tracker = InitTeaBoxTracking;
        click_done = false;
      }
    }

    if (state_teabox_tracker == InitTeaBoxTracking) {
      teabox_tracker.initClick(I, opt_model + ".init", true);
      state_teabox_tracker = WaitTeaBoxTracking;
    }

    if (state_teabox_tracker == WaitTeaBoxTracking) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Click to start the teabox tracking", vpColor::red);
      if (click_done && button == vpMouseButton::button1) {
        state_teabox_tracker = TeaBoxTracking;
        click_done = false;
      }
    }

    if (state_teabox_tracker == TeaBoxTracking || state_teabox_tracker == MoveToDesiredLHandPosition
        || state_teabox_tracker == LearnDesiredLHandOpenLoopPosition || state_teabox_tracker == LearnDesiredLHandGraspPosition
        || state_teabox_tracker == WaitGrasping || state_teabox_tracker == Grasping ) {
      try {
        teabox_tracker.track(I);
        teabox_tracker.getPose(cMo_teabox);
        //printPose("cMo teabox: ", cMo_teabox);
        teabox_tracker.display(I, cMo_teabox, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo_teabox, cam, 0.025, vpColor::none, 3);
        if (opt_learn_open_loop_position)
          state_teabox_tracker = LearnDesiredLHandOpenLoopPosition;
        else if (opt_learn_grasp_position)
          state_teabox_tracker = LearnDesiredLHandGraspPosition;
        else if (! arm_moved)
          state_teabox_tracker = MoveToDesiredLHandPosition;

        status_teabox_tracker = true;
      }
      catch(...) {
        status_teabox_tracker = false;
        state_teabox_tracker = Acquisition;
      }
    }

    // track qrcode
    //    if (state_teabox_tracker == TakeTea)
    //      qrcode_tracker.setForceDetection(true);
    //    else
    //      qrcode_tracker.setForceDetection(false);
    status_qrcode_tracker = qrcode_tracker.track(I);

    if (status_qrcode_tracker) { // display the tracking results
      cMo_qrcode = qrcode_tracker.get_cMo();
      //printPose("cMo qrcode: ", cMo_qrcode);
      // The qrcode frame is only displayed when PBVS is active or learning
      if (state_teabox_tracker == LearnDesiredLHandGraspPosition
          || state_teabox_tracker == MoveToDesiredLHandPosition
          || state_teabox_tracker == WaitGrasping
          || state_teabox_tracker == Grasping) {
        vpDisplay::displayFrame(I, cMo_qrcode, cam, 0.04, vpColor::none, 3);
      }
      vpDisplay::displayPolygon(I, qrcode_tracker.getCorners(), vpColor::green, 2);
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


    // Servo head for teabox
    if (! opt_learn_grasp_position && ! opt_learn_open_loop_position && status_teabox_tracker && ! teabox_servo_converged) {
      static int cpt_iter_servo_head = 0;
      static bool first_time = true;
      if (first_time) {
        servo_time_init = vpTime::measureTimeSecond();
        first_time = false;
      }
      vpImagePoint teabox_cog_cur;
      vpPoint P;
      P.setWorldCoordinates(0.065/2, 0.045/2, -0.1565/2);
      P.project(cMo_teabox);
      double u=0, v=0;
      vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), u, v);
      teabox_cog_cur.set_uv(u, v);

      vpAdaptiveGain lambda(1., .3, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
      servo_head.setLambda(lambda);

      vpMatrix eJe_head = robot.get_eJe("Head");
      servo_head.set_eJe(eJe_head);
      servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

      servo_head.setCurrentFeature(teabox_cog_cur);
      vpImagePoint teabox_cog_des( I.getHeight()*5/8, I.getWidth()*7/8 );
      servo_head.setDesiredFeature( teabox_cog_des );
      //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::blue, 3);
      vpDisplay::displayCross(I, teabox_cog_cur, 15, vpColor::blue, 3); // current feature
      vpDisplay::displayCross(I, teabox_cog_des, 15, vpColor::green, 3); // desired feature

      q_dot_head = servo_head.computeControlLaw(servo_time_init);
      robot.setVelocity(jointNames_head, q_dot_head);

      if (cpt_iter_servo_head > 100) {
        if (opt_record_video)
          vpDisplay::displayText(I, vpImagePoint(10,10), "Click to continue", vpColor::red);
        else
          vpDisplay::displayText(I, vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);

      }
      if ( sqrt(servo_head.m_task_head.getError().sumSquare())*cam.get_px() < 10. || (click_done && button == vpMouseButton::button1 && cpt_iter_servo_head > 100) )
      {
        robot.stop(jointNames_head);
        teabox_servo_converged = true;
      }
      cpt_iter_servo_head ++;
      if (click_done && button == vpMouseButton::button1)
        click_done = false;
    }

    if (state_teabox_tracker == LearnDesiredLHandOpenLoopPosition) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Put the left arm in learning position", vpColor::red);
      vpDisplay::displayText(I, vpImagePoint(25,10), "and left click to learn the pose", vpColor::red);
      if (click_done && button == vpMouseButton::button1) {
        vpHomogeneousMatrix teaboxMh_offset;
        if (learnDesiredLHandOpenLoopPosition(robot, cMo_teabox, eMc, learned_oMh_filename, name_oMh_open_loop, teaboxMh_offset)) {
          printPose("The learned open loop pose: ", teaboxMh_offset);
          std::cout << "is saved in " << learned_oMh_filename << std::endl;
          return 0;
        }
      }
    }

    if (state_teabox_tracker == LearnDesiredLHandGraspPosition && status_qrcode_tracker && status_teabox_tracker) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Put the left arm in grasping position", vpColor::red);
      vpDisplay::displayText(I, vpImagePoint(25,10), "and left click to learn the pose", vpColor::red);
      if (click_done && button == vpMouseButton::button1) {
        vpHomogeneousMatrix teaboxMh_grasp;
        if (learnDesiredLHandGraspingPosition(robot, cMo_teabox, cMo_qrcode, learned_oMh_filename, name_oMh_grasp, teaboxMh_grasp)) {
          printPose("The learned grasping pose: ", teaboxMh_grasp);
          std::cout << "is saved in " << learned_oMh_filename << std::endl;
          return 0;
        }
      }

    }


    if (state_teabox_tracker == MoveToDesiredLHandPosition && status_teabox_tracker && teabox_servo_converged) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to move left arm from rest position open loop", vpColor::red);
      vpDisplay::displayText(I, vpImagePoint(25,10), "Middle click to move left arm from current position in open loop", vpColor::red);
      if (click_done) {
        switch(button) {
        case vpMouseButton::button1:
          moveLArmFromOrToRestPosition(robot, true); // move up
        case vpMouseButton::button2:
          if (! moveToDesiredLHandPosition(robot, cMo_teabox, eMc, learned_oMh_filename, name_oMh_open_loop)) {
            std::cout << "Cannot move to the open loop position, maybe " << name_oMh_open_loop << " doesn't exits in " << learned_oMh_filename << std::endl;
            return 0;
          }
          click_done = false;
          break;
        default:
          break;
        }
        arm_moved = true;
        state_teabox_tracker = WaitGrasping;
      }
    }

    if (state_teabox_tracker == WaitGrasping && status_qrcode_tracker && status_teabox_tracker) {
      vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to start grasping", vpColor::red);
      vpDisplay::displayFrame(I, cMo_teabox * oMh_Tea_Box_grasp , cam, 0.04, vpColor::none, 2);

      if (click_done && button == vpMouseButton::button1) {
        state_teabox_tracker = Grasping;
        click_done = false;
      }
    }

    // Visual servo of the head centering teabox and qrcode
    if (state_teabox_tracker == Grasping) {
      if (status_qrcode_tracker && status_teabox_tracker) {
        // servo head to center qrcode and teabox
        static int cpt_iter_servo_grasp = 0;
        if (1) {
          static bool first_time_head_servo = true;
          if (first_time_head_servo) {
            std::cout << "-- Start visual servoing of the head" << std::endl;
            servo_head_time_init = vpTime::measureTimeSecond();
            first_time_head_servo = false;
          }

          vpImagePoint teabox_cog_cur;
          vpImagePoint qrcode_cog_cur;
          vpPoint P;
          P.setWorldCoordinates(0.065/2, 0.045/2, -0.1565/2);
          P.project(cMo_teabox);
          double u=0, v=0;
          vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), u, v);
          teabox_cog_cur.set_uv(u, v);
          qrcode_cog_cur = qrcode_tracker.getCog();

          vpMatrix eJe_head = robot.get_eJe("Head");
          servo_head.set_eJe(eJe_head);
          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

          vpAdaptiveGain lambda(0.8, .1, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
          servo_head.setLambda(lambda);

          servo_head.setCurrentFeature( (teabox_cog_cur + qrcode_cog_cur)/2 );
          servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*5/8, I.getWidth()/2) );
          vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::yellow, 3);

          vpColVector q_dot_head = servo_head.computeControlLaw(servo_head_time_init);
          robot.setVelocity(jointNames_head, q_dot_head);
        }

        // Servo arm
        if (! grasp_servo_converged) {
          vpMatrix oJo = oVe_LArm * robot.get_eJe("LArm");
          //servo_larm.setLambda(0.15);
          vpAdaptiveGain lambda(0.3, .2, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
          servo_larm.setLambda(lambda);
          servo_larm.set_eJe(oJo);
          vpHomogeneousMatrix torsoMHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));
          vpVelocityTwistMatrix cVtorso( (torsoMHeadRoll * eMc).inverse());
          servo_larm.set_cVf( cVtorso );
          vpHomogeneousMatrix torsoMLWristPitch(robot.getProxy()->getTransform("LWristPitch", 0, true));
          vpVelocityTwistMatrix torsoVo(torsoMLWristPitch * oMe_LArm.inverse());
          servo_larm.set_fVe( torsoVo );

          // Compute the desired position of the hand taking into account the off-set
          cdMc = cMo_teabox * oMh_Tea_Box_grasp * cMo_qrcode.inverse() ;
          printPose("cdMc: ", cdMc);
          servo_larm.setCurrentFeature(cdMc) ;

          vpDisplay::displayFrame(I, cMo_teabox * oMh_Tea_Box_grasp , cam, 0.025, vpColor::red, 2);

          static bool first_time_arm_servo = true;
          if (first_time_arm_servo) {
            std::cout << "-- Start visual servoing of the head" << std::endl;
            servo_arm_time_init = vpTime::measureTimeSecond();
            first_time_arm_servo = false;
          }

          q_dot_larm = servo_larm.computeControlLaw(servo_arm_time_init);

          robot.setVelocity(jointNames_larm, q_dot_larm);

          if (opt_plotter_arm) {
            plotter_arm->plot(0, cpt_iter_servo_grasp, servo_larm.m_task.getError());
            plotter_arm->plot(1, cpt_iter_servo_grasp, q_dot_larm);
          }

          if (cpt_iter_servo_grasp > 100) {
            if (opt_record_video)
              vpDisplay::displayText(I, vpImagePoint(10,10), "Click to continue", vpColor::red);
            else
              vpDisplay::displayText(I, vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);
          }

          vpTranslationVector t_error_grasp = cdMc.getTranslationVector();
          vpRotationMatrix R_error_grasp;
          cdMc.extract(R_error_grasp);
          vpThetaUVector tu_error_grasp;
          tu_error_grasp.buildFrom(R_error_grasp);
          double theta_error_grasp;
          vpColVector u_error_grasp;
          tu_error_grasp.extract(theta_error_grasp, u_error_grasp);

          if ( (sqrt(t_error_grasp.sumSquare()) < 0.005) && (theta_error_grasp < vpMath::rad(3)) || (click_done && button == vpMouseButton::button1 && cpt_iter_servo_grasp > 150) )
          {
            robot.stop(jointNames_larm);
            state_teabox_tracker = TakeTea;
            grasp_servo_converged = true;
            //            if (click_done && button == vpMouseButton::button1 && cpt_iter_servo_grasp > 150) {
            //              click_done = false;
            //            }
          }
        }
        cpt_iter_servo_grasp ++;
      }
      else {
        // state grasping but one of the tracker fails
        robot.stop(jointNames_larm);
      }
    }

    if (state_teabox_tracker == TakeTea)
    {
      typedef enum {
        CloseHand,
        OpenHand,
        WaitLiftTeabox,
        LiftTeabox,
        WaitDeposeTeabox,
        DeposeTeabox,
        WaitPullOutHand,
        PullOutHandUp,
        PullOutHandLeft,
        MoveArmToRestPosition,
        StopServo,
        Finished
      } GraspStatus_t;
      static GraspStatus_t grasp_status = CloseHand;

      std::string nameChain_larm = "LArm";

      static bool first_time = true;
      if (first_time) {
        // stop head servo
        robot.stop(jointNames_head);
        first_time = false;
      }

      // servo head to center qrcode
      if (grasp_status >= LiftTeabox  && grasp_status != Finished) {
        if (status_qrcode_tracker) {
          static bool first_time = true;
          if (first_time) {
            servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
          }

          vpImagePoint qrcode_cog_cur;
          qrcode_cog_cur = qrcode_tracker.getCog();

          vpMatrix eJe_head = robot.get_eJe("Head");
          servo_head.set_eJe(eJe_head);
          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );
          //servo_head.setLambda(0.4);
          static vpAdaptiveGain lambda(2, 0.4, 20); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
          //static vpAdaptiveGain lambda(1.5, 0.2, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
          servo_head.setLambda(lambda);
          servo_head.setCurrentFeature( qrcode_cog_cur );
          servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*6/8, I.getWidth()/2) );
          vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::yellow, 3);

          vpColVector q_dot_head = servo_head.computeControlLaw(servo_time_init);
          robot.setVelocity(jointNames_head, q_dot_head);
        }
        else {
          robot.stop(jointNames_head);
        }
      }

      switch(grasp_status) {
      case CloseHand: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to close the hand", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          // Close hand
          robot.getProxy()->setStiffnesses("LHand", 1.0f);
          AL::ALValue angle = 0.15;
          robot.getProxy()->setAngles("LHand", angle, 0.15);
          if (opt_language_english == false)
            phraseToSay = "Je vais attraper la boite.";
          else
            phraseToSay = "I will grasp the box";
          tts.post.say(phraseToSay);
          click_done = false;
          grasp_status = WaitLiftTeabox;
        }
        break;
      }
        //      case LiftTeabox: { // Naoqi bug that doesn't allow velocity control of the head and cartesian position control of the arm in parallel
        //        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to lift the teabox", vpColor::red);
        //        if (click_done && button == vpMouseButton::button1) {
        //          // Open loop upward motion of the hand

        //          std::vector<float> handPos = robot.getProxy()->getPosition(nameChain_larm, 0, false);
        //          handPos[2] =  handPos[2] + 0.07;
        //          robot.getProxy()->setPositions(nameChain_larm, 0, handPos, 0.05,7);
        //          click_done = false;
        //          grasp_status = DeposeTeabox;
        //        }
        //        break;
        //      }
      case WaitLiftTeabox: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to lift the teabox", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          grasp_status = LiftTeabox;
        }
        break;
      }
      case LiftTeabox: {
        // Open loop upward motion of the hand
        vpColVector cart_delta_pos(6, 0);
        cart_delta_pos[2] = -0.12;
        double delta_t = 5;

        static vpCartesianDisplacement moveCartesian;
        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_LArm)) {
          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian.getJointNames());
          if (! opt_interaction)
            grasp_status = WaitDeposeTeabox;
          else {
            robot.stop(jointNames_head);
            state_teabox_tracker = Interaction;
          }
        }
        break;
      }
        //      case DeposeTeabox: {
        //        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to depose the teabox", vpColor::red);
        //        if (click_done && button == vpMouseButton::button1) {
        //          // Open loop downward motion of the hand
        //          std::vector<float> handPos = robot.getProxy()->getPosition(nameChain_larm, 0, false);
        //          handPos[2] =  handPos[2] - 0.06;
        //          robot.getProxy()->setPositions(nameChain_larm, 0, handPos, 0.05, 7);
        //          click_done = false;
        //          grasp_status = OpenHand;
        //        }
        //        break;
        //      }


      case WaitDeposeTeabox: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to depose the teabox", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          grasp_status = DeposeTeabox;
        }
        break;
      }

      case DeposeTeabox: {
        // Open loop upward motion of the hand
        vpColVector cart_delta_pos(6, 0);
        cart_delta_pos[2] = 0.12;
        double delta_t = 5;

        static vpCartesianDisplacement moveCartesian;
        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_LArm)) {
          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian.getJointNames());
          grasp_status = OpenHand;
        }
        break;
      }

      case OpenHand: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to open the hand", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          double angle = 1.0f;
          robot.getProxy()->setAngles("LHand", angle, 1.);
          click_done = false;
          grasp_status = WaitPullOutHand;
        }
        break;
      }

      case WaitPullOutHand: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to pull out the hand", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          grasp_status = PullOutHandUp;
        }
        break;
      }

      case PullOutHandUp: {
        // Open loop upward motion of the hand
        vpColVector cart_delta_pos(6, 0);
        cart_delta_pos[2] = 0.05;
        double delta_t = 2;

        static vpCartesianDisplacement moveCartesian;
        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_LArm)) {
          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian.getJointNames());
          grasp_status = PullOutHandLeft;
        }
        break;
      }
      case PullOutHandLeft: {
        // Open loop upward motion of the hand
        vpColVector cart_delta_pos(6, 0);
        cart_delta_pos[0] = 0.08;
        double delta_t = 3;

        static vpCartesianDisplacement moveCartesian;
        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_LArm)) {
          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian.getJointNames());
          grasp_status = StopServo;
        }
        break;
      }

      case StopServo: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to stop servo", vpColor::red);
        if (click_done && button == vpMouseButton::button1) { // Quit the loop
          robot.stop(jointNames_head);
          robot.stop(jointNames_larm);
          grasp_status = Finished;
          click_done = false;
        }
        break;
      }
      }
    }

    // Begin interaction
    if (state_teabox_tracker == Interaction) {
      typedef enum {
        MoveArmToRestPosition,
        WaitMoveHeadToZero,
        MoveHeadToZero,
        WaitHeadInZero,
        HeadFollowFace,
        MoveArm,
        ReleaseTea,
        WaitForEnd
      } InteractionStatus_t;
      static InteractionStatus_t interaction_status = WaitMoveHeadToZero;
      static vpColVector head_pos(jointNames_head.size(), 0);
      bool face_found = false;

      // Detect and track the largest face
      if (interaction_status >= HeadFollowFace ) {
        face_found = face_tracker->track(I);

        vpImagePoint head_cog_cur;
        vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);

        if (face_found) {
          vpDisplay::displayRectangle(I, face_tracker->getFace(), vpColor::red, false, 4);

          bool face_centered = false;
          static bool first_time = true;
          if (first_time) {
            servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
          }

          head_cog_cur = face_tracker->getFace().getCenter();

          vpAdaptiveGain lambda(2.5, 1., 30); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
          servo_head.setLambda(lambda);
          servo_head.set_eJe( robot.get_eJe("Head") );
          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

          servo_head.setCurrentFeature(head_cog_cur);
          servo_head.setDesiredFeature(head_cog_des);
          if (opt_record_video)
            vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 2);
          else
            vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 1);

          q_dot_head = servo_head.computeControlLaw(servo_time_init);
          robot.setVelocity(jointNames_head, q_dot_head);

          // Compute the distance in pixel between the target and the center of the image
          double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);
          if (distance < 0.03*I.getWidth() /*&& !face_centered*/) { // 3 % of the image witdh

            face_centered = true;
            //robot.stop(jointNames_head);
            //interaction_status = MoveArm;

          }
          else if (distance > 0.20*I.getWidth()) // 20 % of the image witdh
            face_centered = false;
        }
        else {
          robot.stop(jointNames_head);
        }

      }

      switch (interaction_status) {

      case WaitMoveHeadToZero: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to move head for face detection", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          interaction_status = MoveHeadToZero;
        }
        break;
      }
      case MoveHeadToZero: {
        head_pos = 0;
        head_pos[1] = vpMath::rad(-10.); // NeckPitch
        head_pos[2] = vpMath::rad(-14.); // HeadPitch
        robot.setPosition(jointNames_head, head_pos, 0.06);
        interaction_status = WaitHeadInZero;
        break;
      }

      case WaitHeadInZero: {
        vpColVector head_pos_mes = robot.getPosition(jointNames_head);
        if (sqrt((head_pos_mes-head_pos).sumSquare()) < vpMath::rad(4)) {
          interaction_status = HeadFollowFace;
        }
        break;
      }
      case HeadFollowFace: {
        vpDisplay::displayText(I, 10, 10, "Left click to move the arm" ,vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          robot.stop(jointNames_head);
          interaction_status = MoveArm;
        }
        break;
      }
      case MoveArm: {
        std::vector<float> head_pose;
        AL::ALValue LShoulderYaw_limits;
        float shoulderYaw_pos ;

        head_pose = robot.getProxy()->getPosition("Head", 0, true); // Position Head w.r.t the torso
        shoulderYaw_pos = head_pose[5]; //rotation arround z
        LShoulderYaw_limits = robot.getProxy()->getLimits("LShoulderYaw");

        float offset = vpMath::rad(5);
        if (shoulderYaw_pos < (float)LShoulderYaw_limits[0][0])
          shoulderYaw_pos = (float)LShoulderYaw_limits[0][0] + offset;
        else if(shoulderYaw_pos > (float)LShoulderYaw_limits[0][1])
          shoulderYaw_pos = (float)LShoulderYaw_limits[0][1] - offset;

        std::vector<float> LArmShoulderElbow_pos;
        LArmShoulderElbow_pos.push_back(vpMath::rad(+10.));
        LArmShoulderElbow_pos.push_back(shoulderYaw_pos);
        LArmShoulderElbow_pos.push_back(vpMath::rad(-40.));
        LArmShoulderElbow_pos.push_back(vpMath::rad(-90.));

        std::vector<std::string> LArmShoulderElbowYaw_name;
        LArmShoulderElbowYaw_name.push_back("LShoulderPitch");
        LArmShoulderElbowYaw_name.push_back("LShoulderYaw");
        LArmShoulderElbowYaw_name.push_back("LElbowYaw");
        LArmShoulderElbowYaw_name.push_back("LElbowRoll");

        robot.getProxy()->setAngles(LArmShoulderElbowYaw_name, LArmShoulderElbow_pos, 0.08);

        interaction_status = ReleaseTea;

        break;
      }

      case ReleaseTea: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to release the tea", vpColor::red);
        if (click_done && button == vpMouseButton::button1) {
          if (opt_language_english == false)
            phraseToSay = "Je vais te donner la boite";
          else
            phraseToSay = "I will give you the box";
          tts.post.say(phraseToSay);
          double angle = 1.0f;
          robot.getProxy()->setAngles("LHand", angle, 1.);
          click_done = false;
          robot.stop(jointNames_head);
          interaction_status = MoveArmToRestPosition;
        }
        break;
      }
      case MoveArmToRestPosition: {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to move arm to rest", vpColor::red);
        if (click_done && button == vpMouseButton::button1)  {
          moveLArmFromOrToRestPosition(robot, false); // move down to rest
          click_done = false;
          interaction_status = WaitForEnd;
        }

      case WaitForEnd: {
          //std::cout <<" Waiting for the end" << std::endl;
          break;
        }

        }
      }
    }
    // End interaction

    if (click_done && button == vpMouseButton::button3) { // Quit the loop
      robot.stop(jointNames_head);
      robot.stop(jointNames_larm);
      break;
    }

    double loop_time = vpTime::measureTimeMs() - loop_time_start;
    if (opt_plotter_time)
      plotter_time->plot(0, 0, loop_iter, loop_time);

    vpDisplay::flush(I) ;
    //std::cout << "Loop time: " << vpTime::measureTimeMs() - loop_time_start << std::endl;

    loop_iter ++;
  }

  if (opt_interaction)
    delete face_tracker;

  if (opt_plotter_time)
    delete plotter_time;

  if (opt_plotter_arm)
    delete plotter_arm;

  if (opt_plotter_qrcode_pose)
    delete plotter_qrcode_pose;

  return 0;
}

