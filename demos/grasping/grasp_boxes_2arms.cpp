#include <iostream>
#include <string>
#include <list>
#include <iterator>
#include <time.h>

// Aldebaran includes.
#include <alproxies/altexttospeechproxy.h>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/altransform.h>

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
#include <vpMbLocalization.h>
#include <vpCartesianDisplacement.h>
#include <vpRomeoTkConfig.h>
#include <vpColorDetection.h>
#include <vpJointLimitAvoidance.h>

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
  HeadToZero,
  WaitHeadToZero,
  TrackColorObject,
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
  WaitTakeTea,
  Interaction
} StateTeaboxTracker_t;

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


const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}


/*!
  Check the validity of the pose of the box.
*/

bool checkValiditycMo(vpHomogeneousMatrix cMo)
{

  double x = cMo[0][3];
  double y = cMo[1][3];
  double z = cMo[2][3];

  //std::cout << "x: " << x <<". Limits: -0.40 > y > 0.40" << std::endl;
  //std::cout << "y: " << y <<". Limits: -0.50 > y > 0.50" << std::endl;
  //std::cout << "z: " << z <<". Limits: 0.10 > y > 0.40" << std::endl;
  if (z < 0.10 || z > 0.40
      || x < - 0.20 || x > 0.10
      || y < -0.10 || y > 0.10 )
    return false;
  else
    return true;
}

void moveLArmFromRestPosition (const vpNaoqiRobot &robot, const std::vector<float> &handMbox_desired, const std::string chain_name)
{

  try
  {

    vpPoseVector handMbox_desired_ (handMbox_desired);
    AL::ALValue pos3;
    pos3.arraySetSize(6);

    // Transform in Yall Pitch and Yaw
    AL::Math::Rotation3D rot;
    AL::Math::Transform transform(handMbox_desired);
    rot = AL::Math::rotation3DFromTransform(transform);


    for (unsigned int i = 0; i<3;i++)
    {
      pos3[i] = handMbox_desired_[i];
      pos3[i+3] = rot.toVector()[i];
    }
    AL::ALValue pos1;
    AL::ALValue pos2;

    if (chain_name == "RArm")
    {
      pos1 = AL::ALValue::array(0.38404589891433716, -0.23612679541110992, -0.09724850952625275, 1.4714961051940918, 0.5567980408668518, 0.2787119448184967);
      pos2 = AL::ALValue::array(0.39801979064941406, -0.20118434727191925, 0.17352993786334991, 1.471331238746643, -0.24805442988872528, 0.6248168349266052);
    }
    else
    {
      pos1 = AL::ALValue::array(0.2852230966091156, 0.3805413246154785, -0.17208018898963928, -1.4664039611816406, 0.28257742524147034, 0.17258954048156738);
      pos2 = AL::ALValue::array(0.3599576950073242, 0.3060062527656555, 0.01953596994280815, -1.1513646841049194, -0.18644022941589355, -0.1889418214559555);
    }

    AL::ALValue time1 = 1.5f;
    AL::ALValue time2 = 3.0f;
    AL::ALValue time3 = 5.0f;

    AL::ALValue path;
    path.arrayPush(pos1);
    path.arrayPush(pos2);
    path.arrayPush(pos3);


    AL::ALValue times;
    times.arrayPush(time1);
    times.arrayPush(time2);
    times.arrayPush(time3);

    AL::ALValue chainName  = AL::ALValue::array (chain_name);
    AL::ALValue space      = AL::ALValue::array (0); // Torso
    AL::ALValue axisMask   = AL::ALValue::array (63);

    robot.getProxy()->post.positionInterpolations(chainName, space, path, axisMask, times);

  }
  catch(const std::exception&)
  {
    throw vpRobotException (vpRobotException::badValue,
                            "servo apply the motion");
  }

  return;

}

/*!
  Move LArm to rest position avoiding the table.
*/
void moveLArmToRestPosition(const vpNaoqiRobot &robot,  const std::string chain_name)
{
  try
  {
    AL::ALValue pos0;
    AL::ALValue pos1;
    AL::ALValue pos2;

    if (chain_name == "RArm")
    {
      pos0 = AL::ALValue::array(0.10328315198421478, -0.2016201913356781, -0.3066698908805847, 1.8711119890213013, 0.843141496181488, 0.4650134742259979);
      pos1 = AL::ALValue::array(0.38404589891433716, -0.23612679541110992, -0.09724850952625275, 1.4714961051940918, 0.5567980408668518, 0.2787119448184967);
      pos2 = AL::ALValue::array(0.39801979064941406, -0.20118434727191925, 0.17352993786334991, 1.471331238746643, -0.24805442988872528, 0.6248168349266052);
    }

    else
    {
      pos0 = AL::ALValue::array(0.12116464972496033, 0.2008720338344574, -0.3022586703300476, -2.055604934692383, 1.1719822883605957, -0.6304683089256287);
      pos1 = AL::ALValue::array(0.2852230966091156, 0.3805413246154785, -0.17208018898963928, -1.4664039611816406, 0.28257742524147034, 0.17258954048156738);
      pos2 = AL::ALValue::array(0.3599576950073242, 0.3060062527656555, 0.01953596994280815, -1.1513646841049194, -0.18644022941589355, -0.1889418214559555);
    }


    AL::ALValue time0 = 2.0f;
    AL::ALValue time1 = 3.0f;
    AL::ALValue time2 = 4.5f;


    AL::ALValue path;
    path.arrayPush(pos2);
    path.arrayPush(pos1);
    path.arrayPush(pos0);


    AL::ALValue times;
    times.arrayPush(time0);
    times.arrayPush(time1);
    times.arrayPush(time2);

    AL::ALValue chainName  = AL::ALValue::array (chain_name);
    AL::ALValue space      = AL::ALValue::array (0); // Torso
    AL::ALValue axisMask   = AL::ALValue::array (63);

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
                                       const std::string &transform_name, vpHomogeneousMatrix &oMh_sens, const bool r_eye, const std::string chain_name)
{
  // Trasformation from Torso to Hand sensor
  vpHomogeneousMatrix tMh_sens (robot.getProxy()->getTransform(chain_name,0,true));

  std::string name_last_joint;
  if (r_eye)
    name_last_joint = "REyePitch";
  else
    name_last_joint = "LEyePitch";


  // Trasformation from Torso to LEyePitch
  vpHomogeneousMatrix torsoLEyePitch(robot.getProxy()->getTransform(name_last_joint, 0, true));

  vpHomogeneousMatrix torsoMlcam = torsoLEyePitch * eMc;

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


vpHomogeneousMatrix getOpenLoopDesiredPose(const vpNaoqiRobot &robot, const vpHomogeneousMatrix &cMo,
                                           const vpHomogeneousMatrix &eMc, const std::string &in_filename, const std::string &transform_name, const bool r_eye, const std::string chain_name)
{

  // Load transformation between teabox and desired position of the hand (from sensors) to initializate the tracker
  vpHomogeneousMatrix oMe_hand;

  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  //  char filename_[FILENAME_MAX];
  //  sprintf(filename_, "%s", VISP_NAOQI_GENERAL_M_FILE);

  if (pm.parse(oMe_hand, in_filename, transform_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the Homogeneous matrix named " << transform_name<< "." << std::endl;
    exit(0);
  }
  else
    std::cout << "Homogeneous matrix " << transform_name <<": " << std::endl << oMe_hand << std::endl;

  std::string name_last_joint;
  if (r_eye)
    name_last_joint = "REyePitch";
  else
    name_last_joint = "LEyePitch";


  vpHomogeneousMatrix torsoLEyePitch_(robot.getProxy()->getTransform(name_last_joint, 0, true));


  vpHomogeneousMatrix torsoMlcam_visp_init = torsoLEyePitch_ * eMc;

  vpHomogeneousMatrix tMh_desired;
  tMh_desired = (oMe_hand.inverse() * cMo.inverse() * torsoMlcam_visp_init.inverse()).inverse();

  vpHomogeneousMatrix Mhack;// Hack: TODO remove when naoqi fixed
  if (chain_name == "RArm")
  {
    Mhack[1][3] = +0.03; // add Y - 0.025 offset
    Mhack[2][3] = 0.045;
    tMh_desired = tMh_desired * Mhack;

  }
  else
  {

    Mhack[1][3] = -0.02; // add Y - 0.025 offset
    Mhack[2][3] = 0.04;
    tMh_desired = tMh_desired * Mhack;
  }

  //  std::vector<float> tMh_desired_;
  //  tMh_desired.convert(tMh_desired_);

  return tMh_desired;

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
  std::string opt_face_cascade_name = std::string(ROMEOTK_DATA_FOLDER) + "/face/haarcascade_frontalface_alt.xml";
  std::string opt_box_name = "tabascobox";
  std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);


  bool opt_learn_open_loop_position = false;
  bool opt_learn_grasp_position = false;
  bool opt_plotter_time = false;
  bool opt_plotter_arm = false;
  bool opt_plotter_qrcode_pose = false;
  bool opt_interaction = true;
  bool opt_language_english = true;
  bool opt_record_video = false;
  bool opt_learning_detection = false;
  bool opt_no_color_tracking = true;
  std::vector<bool> opt_Reye(2);
  opt_Reye [0] = false;
  opt_Reye [1] = true;
  bool opt_plotter_q = false;
  bool opt_plotter_q_sec_arm = false;
  bool opt_right_arm = false;

  // Learning folder in /tmp/$USERNAME
  std::string username;
  // Get the user login name
  vpIoTools::getUserName(username);

  // Create a log filename to save new files...
  std::string learning_folder;
#if defined(_WIN32)
  learning_folder ="C:/temp/" + username;
#else
  learning_folder ="/tmp/" + username;
#endif
  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(learning_folder) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(learning_folder);
    }
    catch (vpException &e) {
      std::cout << "Cannot create " << learning_folder << std::endl;
      std::cout << "Error: " << e.getMessage() <<std::endl;
      return 0;
    }
  }



  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--box-name")
      opt_box_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--data-folder")
      opt_data_folder = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--learn-open-loop-position")
      opt_learn_open_loop_position = true;
    else if (std::string(argv[i]) == "--learn-grasp-position")
      opt_learn_grasp_position = true;
    else if (std::string(argv[i]) == "--learn-detection-box")
      opt_learning_detection = true;
    else if (std::string(argv[i]) == "--opt_no_color_tracking")
      opt_no_color_tracking = true;
    else if (std::string(argv[i]) == "--plot-time")
      opt_plotter_time = true;
    else if (std::string(argv[i]) == "--plot-arm")
      opt_plotter_arm = true;
    else if (std::string(argv[i]) == "--plot-qrcode-pose")
      opt_plotter_qrcode_pose = true;
    else if (std::string(argv[i]) == "--plot-q")
      opt_plotter_q = true;
    else if (std::string(argv[i]) == "--plot-q2-joint-avoidance")
      opt_plotter_q_sec_arm = true;
    else if (std::string(argv[i]) == "--no-interaction")
      opt_interaction = false;
    else if (std::string(argv[i]) == "--english")
      opt_language_english = true;
    else if (std::string(argv[i]) == "--opt-record-video")
      opt_record_video = true;
    else if (std::string(argv[i]) == "--rarm")
      opt_right_arm = true;
    else if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--box-name] [--opt_no_color_tracking]" << std::endl;
      std::cout << "       [--haar <haarcascade xml filename>] [--no-interaction] [--learn-open-loop-position] " << std::endl;
      std::cout << "       [--learn-grasp-position] [--plot-time] [--plot-arm] [--plot-qrcode-pose] [--plot-q] "<< std::endl;
      std::cout << "  add  [--rarm] tu use the right arm, nothing to use the left "<< std::endl;
      std::cout << "       [--data-folder] [--learn-detection-box] "<< std::endl;
      std::cout << "       [--english] [--opt-record-video] [--help]" << std::endl;
      return 0;
    }
  }

  if (opt_interaction && ! vpIoTools::checkFilename(opt_face_cascade_name)) {
    std::cout << "Error: the file " << opt_face_cascade_name <<  " doesn't exist." << std::endl;
    std::cout << "Use --haar <haarcascade xml filename> or --no-interaction to disable face detection " << std::endl;
    return 0;
  }

  std::vector<std::string> chain_name(2); // LArm or RArm
  chain_name[0] = "LArm";
  chain_name[1] = "RArm";


  if (opt_learning_detection)
    opt_no_color_tracking = true;

  std::vector<StateTeaboxTracker_t> state_teabox_tracker(2);
  // StateTeaboxTracker_t state_teabox_tracker;
  state_teabox_tracker[0] = Acquisition;
  state_teabox_tracker[1] = Acquisition;

  GraspStatus_t grasp_status = CloseHand;

  //  std::vector<GraspStatus_t> grasp_status(2);
  //  grasp_status[0] = CloseHand;
  //  grasp_status[1] = CloseHand;

  std::string objects_folder = "objects/" + opt_box_name  + "/";
  std::string box_folder = opt_data_folder +"/" + objects_folder;
  std::string config_detection_file_folder = box_folder + "detection/";
  std::string objects_folder_det_learning = config_detection_file_folder + "learning/";
  std::string opt_model = box_folder + "model/" + opt_box_name;
  std::string learning_detection_file = "learning_data.bin";
  std::string learning_data_file_name = objects_folder_det_learning + learning_detection_file;

  std::cout << learning_data_file_name << std::endl;


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

  /************************************************************************************************/
  /** Open the grabber for the acquisition of the images from the robot*/
  vpNaoqiGrabber g;
  g.setCamerasMulti(1); // eyes cameras
  g.setFramerate(15);

  if (! opt_ip.empty()) {
    std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    g.setRobotIp(opt_ip);
  }
  g.openMulti();

  std::vector<std::string> cameraNames(2);
  cameraNames[0] = "CameraLeftEye";
  cameraNames[1] = "CameraRightEye";

  // Initialize constant transformations
  std::vector<vpHomogeneousMatrix> eMc(2);
  eMc[0] = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");
  eMc[1] = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraRightEye");
  std::cout << "CameraLeftEye extrinsic parameters: "  << std::endl << eMc[0]  << std::endl;
  std::cout << "CameraRightEye extrinsic parameters: " << std::endl  << eMc[1] << std::endl;

  //Get cameras extrinsic parameters:
  std::vector<vpCameraParameters> cam(2);
  //Get cameras instrinsic parameters:
  cam[0] = g.getCameraParameters(AL::kQVGA,"CameraLeftEye",vpCameraParameters::perspectiveProjWithoutDistortion);
  cam[1] = g.getCameraParameters(AL::kQVGA,"CameraRightEye",vpCameraParameters::perspectiveProjWithoutDistortion);
  std::cout << "CameraLeftEye instrinsic parameters: " << std::endl << cam[0] << std::endl;
  std::cout << "CameraRightEye instrinsic parameters: " << std::endl << cam[1] << std::endl;

  vpImage<unsigned char> Ir(g.getHeight(), g.getWidth());
  vpImage<unsigned char> Il(g.getHeight(), g.getWidth());

  //  std::vector < vpImage<unsigned char>* > I;
  //  I.push_back(new vpImage<unsigned char>(g.getHeight(), g.getWidth()));
  //  I.push_back(new vpImage<unsigned char>(g.getHeight(), g.getWidth()));

  std::vector < vpImage<unsigned char> > I;
  I.push_back(Ir);
  I.push_back(Il);

  std::cout << I.size();

  vpDisplayX dl(I[0]);
  vpDisplay::setTitle(I[0], "Left camera");

  vpDisplayX dr(I[1]);
  vpDisplay::setTitle(I[1], "Rigth camera");
  vpDisplay::setWindowPosition(I[1], 2 * I[0].getHeight(), 0);


  /************************************************************************************************/

  /** Create a new istance NaoqiRobot*/
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();


  /************************************************************************************************/

  // Initialization detection and localiztion teabox
  std::vector <bool> status_teabox_tracker(2);// false if the tea box tracker fails
  status_teabox_tracker[0] = false;
  status_teabox_tracker[1] = false;
  std::vector<vpHomogeneousMatrix> cMo_teabox(2);


  vpMbLocalization teabox_tracker_l(opt_model, config_detection_file_folder, cam[0]);  // Left box
  teabox_tracker_l.initDetection(learning_data_file_name);
  teabox_tracker_l.setValiditycMoFunction(checkValiditycMo);
  teabox_tracker_l.setOnlyDetection(false);
  vpMbLocalization teabox_tracker_r(opt_model, config_detection_file_folder, cam[1]); // Right box
  teabox_tracker_r.initDetection(learning_data_file_name);
  teabox_tracker_r.setValiditycMoFunction(checkValiditycMo);
  teabox_tracker_r.setOnlyDetection(false);


  std::vector<vpMbLocalization*> teabox_tracker;
  teabox_tracker.push_back(&teabox_tracker_l);
  teabox_tracker.push_back(&teabox_tracker_r);
  std::cout << "Size Localization " << teabox_tracker.size() << std::endl;

  /************************************************************************************************/

  // Initialize the qrcode tracker
  std::vector<bool> status_qrcode_tracker(2);
  status_qrcode_tracker[0] = false;
  status_qrcode_tracker[1] = false;
  std::vector<vpHomogeneousMatrix> cMo_qrcode(2);

  std::vector<vpQRCodeTracker*> qrcode_tracker;

  vpQRCodeTracker qrcode_tracker_l;
  qrcode_tracker_l.setCameraParameters(cam[0]);
  qrcode_tracker_l.setQRCodeSize(0.045);
  qrcode_tracker_l.setMessage("romeo_left_arm");
  qrcode_tracker_l.setForceDetection(true);

  vpQRCodeTracker qrcode_tracker_r;
  qrcode_tracker_r.setCameraParameters(cam[1]);
  qrcode_tracker_r.setQRCodeSize(0.045);
  qrcode_tracker_r.setMessage("romeo_right_arm");
  qrcode_tracker_r.setForceDetection(true);

  qrcode_tracker.push_back(&qrcode_tracker_l);
  qrcode_tracker.push_back(&qrcode_tracker_r);


  /************************************************************************************************/
  // Initialize head servoing
  //  vpServoHead servo_head;
  //  servo_head.setCameraParameters(cam[0]);

  std::vector <bool> teabox_servo_converged(2);
  teabox_servo_converged[0] = false;
  teabox_servo_converged[1] = false;

  //  double servo_time_init = 0;
  //  double servo_head_time_init = 0;
  std::vector< double> servo_arm_time_init(2);
  servo_arm_time_init[0] = 0.0;
  servo_arm_time_init[1] = 0.0;


  std::vector<bool> first_time_arm_servo(2);
  first_time_arm_servo[0] = true;
  first_time_arm_servo[1] = true;

  std::vector<std::string> jointNames_head =  robot.getBodyNames("Head");

  std::vector<std::string> jointNames = robot.getBodyNames("Head");
  //jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  std::vector<std::string> jointNames_tot = jointNames;
  jointNames_tot.push_back(jointNamesREye.at(0));
  jointNames_tot.push_back(jointNamesREye.at(1));
  vpColVector head_pose(jointNames_tot.size(), 0);

  //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","REyeYaw", "REyePitch" );
  AL::ALValue angles_head;
  angles_head      = AL::ALValue::array(vpMath::rad(0.0), vpMath::rad(27.7), vpMath::rad(-2.6), vpMath::rad(0), 0.0, 0.0, 0.0, 0.0  );
  float fractionMaxSpeed  = 0.1f;
  robot.getProxy()->setAngles(jointNames_tot, angles_head, fractionMaxSpeed);


  // Map to don't consider the HeadRoll
  vpMatrix MAP_head(6,5);
  for (unsigned int i = 0; i < 3 ; i++)
    MAP_head[i][i]= 1;
  MAP_head[4][3]= 1;
  MAP_head[5][4]= 1;


  // Constant transformation Target Frame to Arm end-effector (WristPitch)
  std::vector<vpHomogeneousMatrix> oMe_Arm(2);
  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  // Create twist matrix from target Frame to Arm end-effector (WristPitch)
  std::vector <vpVelocityTwistMatrix> oVe_Arm(2);


  for (unsigned int i = 0; i < 2; i++)
  {

    std::string name_transform = "qrcode_M_e_" + chain_name[i];
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMe_Arm[i], filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
      return 0;
    }
    else
      std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm[i] << std::endl;

    oVe_Arm[i].buildFrom(oMe_Arm[i]);

  }

  // Initialize constant parameters
  std::string learned_oMh_filename = "grasping_pose2arms.xml";
  std::vector < std::string> name_oMh_open_loop(2);
  std::vector < std::string> learned_oMh_path(2);
  std::vector <vpHomogeneousMatrix> oMh_Tea_Box_grasp(2);

  for (unsigned int i = 0; i < 2; i++)
  {
    name_oMh_open_loop[i] =  "oMh_open_loop_" + cameraNames[i]; // Offset position Hand w.r.t the object (Open loop)
    learned_oMh_path[i] = box_folder + "grasping/" + chain_name[i];
    std::string name_oMh_grasp =  "oMh_close_loop_"+ cameraNames[i]; // Offset position Hand w.r.t the object to grasp it (Close loop)

    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMh_Tea_Box_grasp[i], learned_oMh_path[i] + "/" + learned_oMh_filename, name_oMh_grasp) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << name_oMh_grasp<< "." << std::endl;
      return 0;
    }
    else
      std::cout << "Homogeneous matrix " << name_oMh_grasp <<": " << std::endl << oMh_Tea_Box_grasp[i] << std::endl;

  }

  std::vector<bool> grasp_servo_converged(2);
  grasp_servo_converged[0]= false;
  grasp_servo_converged[1]= false;

  // Initialize arms servoing
  vpServoArm servo_larm_l;
  vpServoArm servo_larm_r;

  std::vector<vpServoArm*> servo_larm;
  servo_larm.push_back(&servo_larm_l);
  servo_larm.push_back(&servo_larm_r);

  std::vector < std::vector<std::string > > jointNames_larm(2);

  jointNames_larm[0] = robot.getBodyNames(chain_name[0]);
  jointNames_larm[1] = robot.getBodyNames(chain_name[1]);
  // Delete last joint Hand, that we don't consider in the servo
  jointNames_larm[0].pop_back();
  jointNames_larm[1].pop_back();

  const unsigned int numArmJoints =  jointNames_larm.size();
  std::vector<vpHomogeneousMatrix> cdMc(2);

  //Condition number Jacobian Arm
  double cond = 0.0;

  // Initalization data for the joint avoidance limit
  //Vector data for plotting
  vpColVector data(13);

  // Initialize the joint avoidance scheme from the joint limits
  vpColVector jointMin = robot.getJointMin(chain_name[0]);
  vpColVector jointMax = robot.getJointMax(chain_name[0]);

  // Vector joint position of the arm
  vpColVector q(numArmJoints);

  // Bool closedchain to 1 when the arms are grapsing a ridig object
  bool closedchain = 0;

  // Vector secondary task
  vpColVector q2_dot (numArmJoints);

  vpColVector Qmiddle(numArmJoints);

  std::cout << "Joint limits arm: " << std::endl;

  for (unsigned int i=0; i< numArmJoints; i++)
  {
    Qmiddle[i] = ( jointMin[i] + jointMax[i]) /2.;
    std::cout << " Joint " << i << " " << jointNames_larm[i]
                 << ": min=" << vpMath::deg(jointMin[i])
                 << " max=" << vpMath::deg(jointMax[i]) << std::endl;
  }

  double ro = 0.1;
  double ro1 = 0.3;

  vpColVector q_l0_min(numArmJoints);
  vpColVector q_l0_max(numArmJoints);
  vpColVector q_l1_min(numArmJoints);
  vpColVector q_l1_max(numArmJoints);


  //Create vector containing Arm and head joints
  //   std::vector<std::string> joint_names_arm_head = jointNames_larm;
  //  joint_names_arm_head.insert(joint_names_arm_head.end(), jointNames_tot.begin(), jointNames_tot.end());

  // Initialize arm open loop servoing
  std::vector < bool> arm_moved(2);
  arm_moved[0] = false;
  arm_moved[1] = false;

  std::vector<int> cpt_iter_servo_grasp(2);
  cpt_iter_servo_grasp[0] = 0;
  cpt_iter_servo_grasp[1] = 0;

  //Set the stiffness
  robot.setStiffness(jointNames_head, 1.f);
  robot.setStiffness(jointNames_larm[0], 1.f);
  robot.setStiffness(jointNames_larm[0], 1.f);

  //  vpColVector q_dot_head(jointNames_head.size(), 0);
  //  vpColVector q_dot_tot;
  vpColVector q_dot_larm(jointNames_larm[0].size(), 0);
  //vpColVector q_dot_arm_head(joint_names_arm_head.size(), 0);

  //Open the Hand
  robot.getProxy()->setAngles("LHand", 1., 1.);
  robot.getProxy()->setAngles("RHand", 1., 1.);

  // Common
  vpMouseButton::vpMouseButtonType button;
  unsigned long loop_iter = 0;
  std::vector<bool> click_done(2);

  //  // Plotter
  //  vpPlot *plotter_arm;
  //  if (opt_plotter_arm) {
  //    plotter_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+80, I.display->getWindowYPosition(), "Loop time");
  //    plotter_arm->initGraph(0, 6); // visual features
  //    plotter_arm->initGraph(1, q_dot_larm.size()); // d_dot
  //    plotter_arm->setTitle(0, "Visual features error");
  //    plotter_arm->setTitle(1, "joint velocities");

  //    plotter_arm->setLegend(0, 0, "tx");
  //    plotter_arm->setLegend(0, 1, "ty");
  //    plotter_arm->setLegend(0, 2, "tz");
  //    plotter_arm->setLegend(1, 0, "tux");
  //    plotter_arm->setLegend(1, 1, "tuy");
  //    plotter_arm->setLegend(1, 2, "tuz");

  //    plotter_arm->setLegend(1, 0, "LshouderPitch");
  //    plotter_arm->setLegend(1, 1, "LShoulderYaw");
  //    plotter_arm->setLegend(1, 2, "LElbowRoll");
  //    plotter_arm->setLegend(1, 3, "LElbowYaw");
  //    plotter_arm->setLegend(1, 4, "LWristRoll");
  //    plotter_arm->setLegend(1, 5, "LWristYaw");
  //    plotter_arm->setLegend(1, 6, "LWristPitch");


  //  }
  //  vpPlot *plotter_qrcode_pose;
  //  if (opt_plotter_qrcode_pose) {
  //    plotter_qrcode_pose = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Qrcode pose");
  //    plotter_qrcode_pose->initGraph(0, 3); // translations
  //    plotter_qrcode_pose->initGraph(1, 3); // rotations
  //    plotter_qrcode_pose->setTitle(0, "Pose translation");
  //    plotter_qrcode_pose->setTitle(1, "Pose theta u");
  //    plotter_qrcode_pose->setLegend(0, 0, "tx");
  //    plotter_qrcode_pose->setLegend(0, 1, "ty");
  //    plotter_qrcode_pose->setLegend(0, 2, "tz");
  //    plotter_qrcode_pose->setLegend(1, 0, "tux");
  //    plotter_qrcode_pose->setLegend(1, 1, "tuy");
  //    plotter_qrcode_pose->setLegend(1, 2, "tuz");
  //  }


  //  vpPlot *plotter_q;
  //  if (opt_plotter_q) {

  //    plotter_q = new vpPlot(1, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+60, "Values of q and limits");
  //    plotter_q->initGraph(0, 13);

  //    plotter_q->setTitle(0, "Q1 values");

  //    plotter_q->setLegend(0, 0, "LshouderPitch");
  //    plotter_q->setLegend(0, 1, "LShoulderYaw");
  //    plotter_q->setLegend(0, 2, "LElbowRoll");
  //    plotter_q->setLegend(0, 3, "LElbowYaw");
  //    plotter_q->setLegend(0, 4, "LWristRoll");
  //    plotter_q->setLegend(0, 5, "LWristYaw");
  //    plotter_q->setLegend(0, 6, "LWristPitch");

  //    plotter_q->setLegend(0, 7, "Low Limits");
  //    plotter_q->setLegend(0, 8, "Upper Limits");
  //    plotter_q->setLegend(0, 9, "l0 min");
  //    plotter_q->setLegend(0, 10, "l0 max");
  //    plotter_q->setLegend(0, 11, "l1 min");
  //    plotter_q->setLegend(0, 12, "l1 max");

  //    plotter_q->setColor(0, 7,vpColor::darkRed);
  //    plotter_q->setThickness(0, 7,2);
  //    plotter_q->setColor(0, 9,vpColor::darkRed);
  //    plotter_q->setThickness(0, 9,2);
  //    plotter_q->setColor(0, 11,vpColor::darkRed);
  //    plotter_q->setThickness(0, 11,2);

  //    plotter_q->setColor(0, 8,vpColor::darkRed);
  //    plotter_q->setThickness(0, 8,2);
  //    plotter_q->setColor(0, 10,vpColor::darkRed);
  //    plotter_q->setThickness(0, 10,2);
  //    plotter_q->setColor(0, 12,vpColor::darkRed);
  //    plotter_q->setThickness(0, 12,2);

  //  }

  //  vpPlot *plotter_q_sec_arm;
  //  if (opt_plotter_q_sec_arm) {
  //    plotter_q_sec_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+30, "Secondary Task");
  //    plotter_q_sec_arm->initGraph(0, 7); // translations
  //    plotter_q_sec_arm->initGraph(1, 7); // rotations

  //    plotter_q_sec_arm->setTitle(0, "Q2 values");
  //    plotter_q_sec_arm->setTitle(1, "Q tot values");

  //    plotter_q_sec_arm->setLegend(0, 0, "LshouderPitch");
  //    plotter_q_sec_arm->setLegend(0, 1, "LShoulderYaw");
  //    plotter_q_sec_arm->setLegend(0, 2, "LElbowRoll");
  //    plotter_q_sec_arm->setLegend(0, 3, "LElbowYaw");
  //    plotter_q_sec_arm->setLegend(0, 4, "LWristRoll");
  //    plotter_q_sec_arm->setLegend(0, 5, "LWristYaw");
  //    plotter_q_sec_arm->setLegend(0, 6, "LWristPitch");

  //    plotter_q_sec_arm->setLegend(1, 0, "LshouderPitch");
  //    plotter_q_sec_arm->setLegend(1, 1, "LShoulderYaw");
  //    plotter_q_sec_arm->setLegend(1, 2, "LElbowRoll");
  //    plotter_q_sec_arm->setLegend(1, 3, "LElbowYaw");
  //    plotter_q_sec_arm->setLegend(1, 4, "LWristRoll");
  //    plotter_q_sec_arm->setLegend(1, 5, "LWristYaw");
  //    plotter_q_sec_arm->setLegend(1, 6, "LWristPitch");

  //  }


  //  vpPlot *plotter_cond;
  //  if (0) {
  //    plotter_cond = new vpPlot(1, I.getHeight(), I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Loop time");
  //    plotter_cond->initGraph(0, 1);
  //  }


  //  vpPlot *plotter_time;
  //  if (opt_plotter_time) {
  //    plotter_time = new vpPlot(1, I.getHeight(), I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Loop time");
  //    plotter_time->initGraph(0, 1);
  //  }

  while(1) {
    double loop_time_start = vpTime::measureTimeMs();


    g.acquireMulti(I[0],I[1]);


    vpDisplay::display(I[0]);
    vpDisplay::display(I[1]);

    //vpImageIo::write(I, "milkbox.ppm");

    //Get Actual position of the arm joints
    //q = robot.getPosition(jointNames_larm);


    if (! opt_record_video)
      vpDisplay::displayText(I[0], vpImagePoint(I[0].getHeight() - 10, 10), "Right click to quit", vpColor::red);


    click_done[0] = false;
    click_done[1] = false;

    click_done[0] = vpDisplay::getClick(I[0], button, false);
    click_done[1] = vpDisplay::getClick(I[1], button, false);




    for(unsigned int i = 0; i < 2 ; i++)
    {

      // track teabox
      if(state_teabox_tracker[i] == Acquisition  ) {

        if (! opt_record_video)
          vpDisplay::displayText(I[i], vpImagePoint(10,10), "Left click: automatic detection, Central: manual", vpColor::red);

        if ( teabox_tracker[i]->track(I[i]))
        {
          cMo_teabox[i] =  teabox_tracker[i]->get_cMo();
          teabox_tracker[i]->getTracker()->display(I[i], cMo_teabox[i], cam[i], vpColor::red, 2);
          vpDisplay::displayFrame(I[i], cMo_teabox[i], cam[i], 0.025, vpColor::none, 3);
        }

        if (click_done[i] && button == vpMouseButton::button1 ) {
          state_teabox_tracker[i] = WaitTeaBoxTracking;
          teabox_tracker[i]->setOnlyDetection(false); // Stop only detection
          click_done[i] = false;
        }

        if (click_done[i] && button == vpMouseButton::button2 ) {
          teabox_tracker[i]->setManualDetection();
          state_teabox_tracker[i] = TeaBoxTracking;
          click_done[i] = false;
        }

      }


      status_qrcode_tracker[i] = qrcode_tracker[i]->track(I[i]);

      if (status_qrcode_tracker[i]) { // display the tracking results
        cMo_qrcode[i] = qrcode_tracker[i]->get_cMo();
        //printPose("cMo qrcode: ", cMo_qrcode);
        // The qrcode frame is only displayed when PBVS is active or learning
        if (state_teabox_tracker[i] == LearnDesiredLHandGraspPosition
            || state_teabox_tracker[i] == MoveToDesiredLHandPosition
            || state_teabox_tracker[i] == WaitGrasping
            || state_teabox_tracker[i] == Grasping) {
          vpDisplay::displayFrame(I[i], cMo_qrcode[i], cam[i], 0.04, vpColor::none, 3);
        }
        vpDisplay::displayPolygon(I[i], qrcode_tracker[i]->getCorners(), vpColor::green, 2);
      }


      if (state_teabox_tracker[i] == WaitTeaBoxTracking) {
        vpDisplay::displayText(I[i], vpImagePoint(10,10), "Left click to continue, Central to re-detect", vpColor::red);

        if ( teabox_tracker[i]->track(I[i]))
        {
          cMo_teabox[i] =  teabox_tracker[i]->get_cMo();
          teabox_tracker[i]->getTracker()->display(I[i], cMo_teabox[i], cam[i], vpColor::red, 2);
          vpDisplay::displayFrame(I[i], cMo_teabox[i], cam[i], 0.025, vpColor::none, 3);
        }

        if (click_done[i] && button == vpMouseButton::button1) {
          state_teabox_tracker[i] = TeaBoxTracking;
          click_done[i] = false;
        }

        else if (click_done[i] && button == vpMouseButton::button2)
        {
          state_teabox_tracker[i] = Acquisition;
          teabox_tracker[i]->setForceDetection();
          teabox_tracker[i]->setOnlyDetection(true); // restart only detection
          click_done[i] = false;
        }

      }


      if  (state_teabox_tracker[i] == TeaBoxTracking || state_teabox_tracker[i] == MoveToDesiredLHandPosition
           || state_teabox_tracker[i] == WaitGrasping || state_teabox_tracker[i] == Grasping)
      {
        try {
          if ( teabox_tracker[i]->track(I[i]))
          {
            cMo_teabox[i]=  teabox_tracker[i]->get_cMo();

            teabox_tracker[i]->getTracker()->display(I[i], cMo_teabox[i], cam[i], vpColor::red, 2);
            vpDisplay::displayFrame(I[i], cMo_teabox[i], cam[i], 0.025, vpColor::none, 3);

            if (! arm_moved[i])
              state_teabox_tracker[i] = MoveToDesiredLHandPosition;

            status_teabox_tracker[i] = true;

          }
        }
        catch(...) {
          status_teabox_tracker[i] = false;
          state_teabox_tracker[i] = Acquisition;
        }
      }

      // Servo eyes for teabox
      if ( status_teabox_tracker[i] && ! teabox_servo_converged[i]) {
        static int cpt_iter_servo_head = 0;

        vpImagePoint teabox_cog_cur;
        vpPoint P;
        P.setWorldCoordinates(0.05/2, 0.05/2, -0.15/2);
        P.project(cMo_teabox[i]);
        double u=0, v=0;
        vpMeterPixelConversion::convertPoint(cam[i], P.get_x(), P.get_y(), u, v);
        teabox_cog_cur.set_uv(u, v);

        vpColVector eye_vel(2);
        vpColVector e(2);
        double lambda = 0.4;
        vpImagePoint teabox_cog_des;
        if (i == 0)
          teabox_cog_des.set_ij( I[i].getHeight()*5/8, I[i].getWidth()*6.6/8 );
        else
          teabox_cog_des.set_ij( I[i].getHeight()*5/8, I[i].getWidth()*1.6/8 );
        double x_s = 0.0, y_s = 0.0;
        vpPixelMeterConversion::convertPoint(cam[i], teabox_cog_des, x_s, y_s);

        double den = 1 + P.get_x() * P.get_x() + P.get_y() * P.get_y();

        e[0] = P.get_x() -  x_s;
        e[1] = P.get_y() -  y_s ;

        eye_vel[1] =  lambda * e[1] / (den);
        eye_vel[0] =  lambda * (-e[0]) / (den);

        vpDisplay::displayCross(I[i], teabox_cog_des, 15, vpColor::blue, 3); // only desired feature
        vpDisplay::displayCross(I[i], teabox_cog_cur, 15, vpColor::yellow, 3); // only desired feature

        std::cout << "vel[1] " << eye_vel[1] << std::endl;
        std::cout << "vel[0] " << eye_vel[0] << std::endl;

        if (i == 0)
          robot.setVelocity(jointNamesLEye, eye_vel);
        else
          robot.setVelocity(jointNamesREye, eye_vel);

        if (cpt_iter_servo_head > 30) {
          if (opt_record_video)
            vpDisplay::displayText(I[i], vpImagePoint(10,10), "Click to continue", vpColor::red);
          else
            vpDisplay::displayText(I[i], vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);

        }
        if ( sqrt(e.sumSquare())*cam[i].get_px() < 10. || (click_done[i] && button == vpMouseButton::button1 && cpt_iter_servo_head > 30) )
        {
          if (i == 0)
            robot.stop(jointNamesLEye);
          else
            robot.stop(jointNamesREye);

          teabox_servo_converged[i] = true;
        }
        cpt_iter_servo_head ++;
        if (click_done[i] && button == vpMouseButton::button1)
          click_done[i] = false;
      }


      if (state_teabox_tracker[i] == MoveToDesiredLHandPosition && status_teabox_tracker[i] && teabox_servo_converged[i]) {
        if (! opt_record_video)
        {
          vpDisplay::displayText(I[i], vpImagePoint(10,10), "Left click to move the arm from rest position open loop", vpColor::red);
          vpDisplay::displayText(I[i], vpImagePoint(25,10), "Middle click to move the arm from current position in open loop", vpColor::red);
        }
        if (click_done[i]) {

          vpHomogeneousMatrix handMbox_desired = getOpenLoopDesiredPose(robot, cMo_teabox[i], eMc[i], learned_oMh_path[i] + "/" + learned_oMh_filename, name_oMh_open_loop[i], opt_Reye[i], chain_name[i]);
          std::vector<float> handMbox_desired_;
          handMbox_desired.convert(handMbox_desired_);

          switch(button) {

          case vpMouseButton::button1:{
            moveLArmFromRestPosition(robot, handMbox_desired_ ,chain_name[i]); // move up
            click_done[i] = false;
          }break;

          case vpMouseButton::button2:{
            float velocity = 0.2;
            int axis_mask = 63; // Control position and orientation
            robot.getProxy()->setTransform(chain_name[i], 0, handMbox_desired_, velocity, axis_mask);
            click_done[i] = false;
          }break;

          default:
            break;
          }
          arm_moved[i] = true;
          state_teabox_tracker[i] = WaitGrasping;
        }
      }

      if (state_teabox_tracker[i] == WaitGrasping && status_qrcode_tracker[i] && status_teabox_tracker[i]) {
        vpDisplay::displayText(I[i], vpImagePoint(10,10), "Left click to start grasping", vpColor::red);
        vpDisplay::displayFrame(I[i], cMo_teabox[i] * oMh_Tea_Box_grasp[i] , cam[i], 0.04, vpColor::none, 2);

        if (click_done[i] && button == vpMouseButton::button1) {
          state_teabox_tracker[i] = Grasping;
          click_done[i] = false;
        }
      }


      // Visual servo of the head centering teabox and qrcode
      if (state_teabox_tracker[i] == Grasping) {
        if (status_qrcode_tracker[i] && status_teabox_tracker[i]) {

          // Servo arm -pregraps
          if (! grasp_servo_converged[i]) {

            vpAdaptiveGain lambda(0.8, 0.05, 8);
            servo_larm[i]->setLambda(lambda);

            servo_larm[i]->set_eJe(robot.get_eJe(chain_name[i]));
            servo_larm[i]->m_task.set_cVe(oVe_Arm[i]);

            cdMc[i] = (cMo_qrcode[i].inverse() * cMo_teabox[i] * oMh_Tea_Box_grasp[i]).inverse();
            printPose("cdMc: ", cdMc[i]);
            servo_larm[i]->setCurrentFeature(cdMc[i]) ;



            vpDisplay::displayFrame(I[i], cMo_teabox[i] * oMh_Tea_Box_grasp[i] , cam[i], 0.025, vpColor::red, 2);

            if (first_time_arm_servo[i]) {
              std::cout << "-- Start visual servoing of the arm" << chain_name[i] << "." << std::endl;
              servo_arm_time_init[i] = vpTime::measureTimeSecond();
              first_time_arm_servo[i] = false;
            }

            q_dot_larm =  - servo_larm[i]->computeControlLaw(servo_arm_time_init[i]);


            vpTranslationVector t_error_grasp = cdMc[i].getTranslationVector();
            vpRotationMatrix R_error_grasp;
            cdMc[i].extract(R_error_grasp);
            vpThetaUVector tu_error_grasp;
            tu_error_grasp.buildFrom(R_error_grasp);
            double theta_error_grasp;
            vpColVector u_error_grasp;
            tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
            std::cout << "error: " << sqrt(t_error_grasp.sumSquare()) << " " << vpMath::deg(theta_error_grasp) << std::endl;

            //          vpVelocityTwistMatrix cVo(cMo_qrcode);
            //          vpMatrix cJe = cVo * oJo;
            // Compute the feed-forward terms
            //          vpColVector sec_ter = 0.5 * ((servo_head.m_task_head.getTaskJacobianPseudoInverse() *  (servo_head.m_task_head.getInteractionMatrix() * cJe)) * q_dot_larm);
            //          std::cout <<"Second Term:" <<sec_ter << std::endl;
            //q_dot_head = q_dot_head + sec_ter;

            vpColVector task_error = servo_larm[i]->m_task.getError();
            vpMatrix taskJac = servo_larm[i]->m_task.getTaskJacobian();
            vpMatrix taskJacPseudoInv = servo_larm[i]->m_task.getTaskJacobianPseudoInverse();

            //cond = taskJacPseudoInv.cond();
            //plotter_cond->plot(0, 0, loop_iter, cond);

            // Compute joint limit avoidance
            //q2_dot = computeQdotLimitAvoidance(task_error, taskJac, taskJacPseudoInv, jointMin, jointMax, q, q_dot_larm, ro, ro1, q_l0_min, q_l0_max, q_l1_min, q_l1_max );

            //q_dot_head = q_dot_head;

            // Add mirroring eyes
            //          q_dot_tot = q_dot_head;
            //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
            //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);


            // vpColVector q_dot_arm_head = q_dot_larm + q2_dot;

            //q_dot_arm_head.stack(q_dot_tot);
            //          robot.setVelocity(joint_names_arm_head,q_dot_arm_head);
            robot.setVelocity(jointNames_larm[i], q_dot_larm);

            //          if (opt_plotter_arm) {
            //            plotter_arm->plot(0, cpt_iter_servo_grasp, servo_larm.m_task.getError());
            //            plotter_arm->plot(1, cpt_iter_servo_grasp, q_dot_larm);
            //          }

            //          if (opt_plotter_q_sec_arm)
            //          {
            //            plotter_q_sec_arm->plot(0,loop_iter,q2_dot);
            //            plotter_q_sec_arm->plot(1,loop_iter,q_dot_larm + q2_dot);

            //          }



            if (cpt_iter_servo_grasp[i] > 100) {
              if (opt_record_video)
                vpDisplay::displayText(I[i], vpImagePoint(10,10), "Click to continue", vpColor::red);
              else
                vpDisplay::displayText(I[i], vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);
            }
            double error_t_treshold = 0.007;

            if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(3)) || (click_done[i] && button == vpMouseButton::button1 /*&& cpt_iter_servo_grasp > 150*/) )
            {
              robot.stop(jointNames_larm[i]);
              state_teabox_tracker[i] = WaitTakeTea;
              grasp_servo_converged[i] = true;

              if (click_done[i] && button == vpMouseButton::button1)
                click_done[i] = false;
            }


          }
          cpt_iter_servo_grasp[i] ++;



        }
        else {
          // state grasping but one of the tracker fails
          robot.stop(jointNames_larm[i]);
        }
      }


    }


    if (state_teabox_tracker[0] == WaitTakeTea && state_teabox_tracker[1] == WaitTakeTea  )

    {
      state_teabox_tracker[0] = TakeTea;
      state_teabox_tracker[1] = TakeTea;

    }


    if (state_teabox_tracker[0] == TakeTea)
    {


      //      static bool first_time = true;
      //      if (first_time) {
      //        // stop head servo
      //        robot.stop(jointNames_tot);
      //        first_time = false;
      //      }

      //      // servo head to center qrcode
      //      if (grasp_status >= CloseHand  && grasp_status != Finished) {
      //        if (status_qrcode_tracker) {

      //          vpImagePoint qrcode_cog_cur;
      //          qrcode_cog_cur = qrcode_tracker.getCog();

      //          vpMatrix eJe_head;
      //          if (opt_Reye)
      //            eJe_head = robot.get_eJe("REye");
      //          else
      //            eJe_head = robot.get_eJe("LEye");

      //          servo_head.set_eJe(eJe_head);
      //          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );
      //          //servo_head.setLambda(0.4);
      //          //static vpAdaptiveGain lambda(2, 0.7, 20); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
      //          vpAdaptiveGain lambda(2.5, 1., 15);
      //          //static vpAdaptiveGain lambda(1.5, 0.2, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
      //          servo_head.setLambda(lambda);
      //          servo_head.setCurrentFeature( qrcode_cog_cur );
      //          if(opt_right_arm)
      //            servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*6/8, I.getWidth()*5/8) );
      //          else
      //            servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*6/8, I.getWidth()*2/8) );
      //          vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::yellow, 3);


      //          static bool first_time_servo = true;
      //          static double servo_init;
      //          if (first_time_servo) {
      //            servo_init = vpTime::measureTimeSecond();
      //            first_time_servo = false;
      //          }
      //          vpColVector q_dot_head = servo_head.computeControlLaw(servo_init);

      //          // Add mirroring eyes
      //          q_dot_tot = q_dot_head;
      //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
      //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

      //          robot.setVelocity(jointNames_tot, q_dot_tot);
      //        }
      //        else {
      //          robot.stop(jointNames_tot);
      //        }
      //      }

      switch(grasp_status) {
      case CloseHand: {
        vpDisplay::displayText(I[0], vpImagePoint(10,10), "Left click to close the hands", vpColor::red);
        if (click_done[0] && button == vpMouseButton::button1) {
          // Close hands

          robot.getProxy()->setStiffnesses("RHand", 1.0f);
          robot.getProxy()->setStiffnesses("LHand", 1.0f);
          AL::ALValue angle = 0.15;
          robot.getProxy()->setAngles("RHand", angle, 0.15);
          robot.getProxy()->setAngles("LHand", angle, 0.15);

          if (opt_language_english == false)
            phraseToSay = "Je vais attraper la boite.";
          else
            phraseToSay = "I will grasp the box";
          tts.post.say(phraseToSay);
          click_done[0] = false;
          grasp_status = WaitLiftTeabox;
          closedchain = true;
        }
        break;
      }
      case WaitLiftTeabox: {
        vpDisplay::displayText(I[0], vpImagePoint(10,10), "Left click to lift the teabox", vpColor::red);
        if (click_done[0] && button == vpMouseButton::button1) {
          click_done[0] = false;
          grasp_status = LiftTeabox;
        }
        break;
      }
      case LiftTeabox: {
        // Open loop upward motion of the hands
        vpColVector cart_delta_pos_l(6, 0);
        cart_delta_pos_l[2] = 0.12;

        vpColVector cart_delta_pos_r(6, 0);
        cart_delta_pos_r[2] = 0.12;

        double delta_t = 2.0;

        static vpCartesianDisplacement moveCartesian_l;
        static vpCartesianDisplacement moveCartesian_r;

        vpVelocityTwistMatrix V;
        if (moveCartesian_l.computeVelocity(robot, cart_delta_pos_l, delta_t, chain_name[0], V) &&
            moveCartesian_r.computeVelocity(robot, cart_delta_pos_r, delta_t, chain_name[1], V)) {
          robot.setVelocity(moveCartesian_l.getJointNames(), moveCartesian_l.getJointVelocity());
          robot.setVelocity(moveCartesian_r.getJointNames(), moveCartesian_r.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian_l.getJointNames());
          robot.stop(moveCartesian_r.getJointNames());
          //          if (! opt_interaction)
          grasp_status = WaitDeposeTeabox;
          //          else {
          //            robot.stop(jointNames_tot);
          //            state_teabox_tracker = Interaction;
          //          }
        }
        break;
      }

      case WaitDeposeTeabox: {
        vpDisplay::displayText(I[0], vpImagePoint(10,10), "Left click to depose the boxes", vpColor::red);
        if (click_done[0] && button == vpMouseButton::button1) {
          click_done[0] = false;
          grasp_status = DeposeTeabox;
        }
        break;
      }

      case DeposeTeabox: {
        // Open loop upward motion of the hand
        vpColVector cart_delta_pos_l(6, 0);
        cart_delta_pos_l[2] = -0.12;

        vpColVector cart_delta_pos_r(6, 0);
        cart_delta_pos_r[2] = -0.12;

        double delta_t = 2.1;
        vpVelocityTwistMatrix V;

        static vpCartesianDisplacement moveCartesian_l;
        static vpCartesianDisplacement moveCartesian_r;


        if (moveCartesian_l.computeVelocity(robot, cart_delta_pos_l, delta_t, chain_name[0], V) &&
            moveCartesian_r.computeVelocity(robot, cart_delta_pos_r, delta_t, chain_name[1], V)) {
          robot.setVelocity(moveCartesian_l.getJointNames(), moveCartesian_l.getJointVelocity());
          robot.setVelocity(moveCartesian_r.getJointNames(), moveCartesian_r.getJointVelocity());
        }
        else {
          robot.stop(moveCartesian_l.getJointNames());
          robot.stop(moveCartesian_r.getJointNames());
          grasp_status = OpenHand;
        }
        break;
      }

      case OpenHand: {
        vpDisplay::displayText(I[0], vpImagePoint(10,10), "Left click to open the hand", vpColor::red);
        if (click_done[0] && button == vpMouseButton::button1) {
          double angle = 1.0f;
          robot.getProxy()->setAngles("LHand", angle, 1.);
          robot.getProxy()->setAngles("RHand", angle, 1.);
          click_done[0] = false;
          grasp_status = WaitPullOutHand;
        }
        break;
      }

        //      case WaitPullOutHand: {
        //        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to pull out the hand", vpColor::red);
        //        if (click_done && button == vpMouseButton::button1) {
        //          click_done = false;
        //          grasp_status = PullOutHandUp;
        //        }
        //        break;
        //      }

        //      case PullOutHandUp: {
        //        // Open loop upward motion of the hand
        //        vpColVector cart_delta_pos(6, 0);
        //        cart_delta_pos[2] = 0.05;
        //        double delta_t = 2;

        //        static vpCartesianDisplacement moveCartesian;
        //        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_Arm)) {
        //          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        //        }
        //        else {
        //          robot.stop(moveCartesian.getJointNames());
        //          grasp_status = PullOutHandLeft;
        //        }
        //        break;
        //      }
        //      case PullOutHandLeft: {
        //        // Open loop upward motion of the hand
        //        vpColVector cart_delta_pos(6, 0);
        //        cart_delta_pos[0] = 0.08;
        //        double delta_t = 3;

        //        static vpCartesianDisplacement moveCartesian;
        //        if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_Arm)) {
        //          robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        //        }
        //        else {
        //          robot.stop(moveCartesian.getJointNames());
        //          grasp_status = StopServo;
        //        }
        //        break;
        //      }

        //      case StopServo: {
        //        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to stop servo", vpColor::red);
        //        if (click_done && button == vpMouseButton::button1) { // Quit the loop
        //          robot.stop(jointNames_tot);
        //          robot.stop(jointNames_larm);
        //          grasp_status = Finished;
        //          click_done = false;
        //        }
        //        break;
        //      }
        //      }
      }



    }



    if (click_done[0] && button == vpMouseButton::button3 || closedchain) { // Quit the loop
      //robot.stop(jointNames_tot);
      robot.stop(jointNames_larm[0]);
      robot.stop(jointNames_larm[1]);
      break;
    }

    //    double loop_time = vpTime::measureTimeMs() - loop_time_start;
    //    if (opt_plotter_time)
    //      plotter_time->plot(0, 0, loop_iter, loop_time);

    //    if (opt_plotter_q )
    //    {

    //      // q normalized between (entre -1 et 1)
    //      for (unsigned int i=0 ; i < numArmJoints ; i++) {
    //        data[i] = (q[i] - Qmiddle[i]) ;
    //        data[i] /= (jointMax[i] - jointMin[i]) ;
    //        data[i]*=2 ;
    //      }

    //      data[numArmJoints] = -1.0;
    //      data[numArmJoints+1] = 1.0;

    //      unsigned int joint = 1;
    //      double tQmin_l0 = jointMin[joint] + ro *(jointMax[joint] - jointMin[joint]);
    //      double tQmax_l0 = jointMax[joint] - ro *(jointMax[joint] - jointMin[joint]);

    //      double tQmin_l1 =  tQmin_l0 - ro * ro1 * (jointMax[joint] - jointMin[joint]);
    //      double tQmax_l1 =  tQmax_l0 + ro * ro1 * (jointMax[joint] - jointMin[joint]);

    //      data[numArmJoints+2] = 2*(tQmin_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
    //      data[numArmJoints+3] = 2*(tQmax_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

    //      data[numArmJoints+4] =  2*(tQmin_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
    //      data[numArmJoints+5] =  2*(tQmax_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

    //      plotter_q->plot(0,0,loop_iter,data[0]);
    //      plotter_q->plot(0,1,loop_iter,data[1]);
    //      plotter_q->plot(0,2,loop_iter,data[2]);
    //      plotter_q->plot(0,3,loop_iter,data[3]);
    //      plotter_q->plot(0,4,loop_iter,data[4]);
    //      plotter_q->plot(0,5,loop_iter,data[5]);
    //      plotter_q->plot(0,6,loop_iter,data[6]);

    //      plotter_q->plot(0,7,loop_iter,data[7]);
    //      plotter_q->plot(0,8,loop_iter,data[8]);

    //      plotter_q->plot(0,9,loop_iter,data[9]);
    //      plotter_q->plot(0,10,loop_iter,data[10]);
    //      plotter_q->plot(0,11,loop_iter,data[11]);
    //      plotter_q->plot(0,12,loop_iter,data[12]);

    //    }

    vpDisplay::flush(I[0]) ;

    vpDisplay::flush(I[1]) ;

    //std::cout << "Loop time: " << vpTime::measureTimeMs() - loop_time_start << std::endl;
    loop_iter ++;
  }


  // Second part: Use two arms toghether

  vpDisplay::close(I[1]);
  g.cleanup();

  // Create new video proxy


  /** Open the grabber for the acquisition of the images from the robot (from one camera)*/
  vpNaoqiGrabber g_s;
  g_s.setFramerate(15);

  std::cout << "Using camera Eye Left" << std::endl;
  g_s.setCamera(2); // CameraLeft

  if (! opt_ip.empty())
    g_s.setRobotIp(opt_ip);
  g_s.open();

  std::cout << "Start closed chain arms phase" << std::endl;

  while(1)
  {

    g_s.acquire(I[0]);

    vpDisplay::display(I[0]);

    click_done[0] = vpDisplay::getClick(I[0], button, false);

    if (click_done[0] && button == vpMouseButton::button3) { // Quit the loop
      //robot.stop(jointNames_tot);
     // robot.stop(jointNames_larm[0]);
      break;
    }


    vpDisplay::flush(I[0]) ;

  }

  //  if (qrcode_tracker[0])
  //    delete qrcode_tracker[0];

  //  if (qrcode_tracker[1])
  //    delete qrcode_tracker[1];


  //  if (opt_interaction)
  //    delete face_tracker;

  //  if (opt_plotter_time)
  //    delete plotter_time;

  //  if (opt_plotter_arm)
  //    delete plotter_arm;

  //  if (opt_plotter_qrcode_pose)
  //    delete plotter_qrcode_pose;

  //  if (opt_plotter_q)
  //    delete plotter_q;

  //  if (opt_plotter_q_sec_arm)
  //    delete plotter_q_sec_arm;

  return 0;
}

