#include <iostream>
#include <string>
#include <time.h>

// Aldebaran includes.
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/altransform.h>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpPlot.h>
#include <visp/vpPoint.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>

#include <vpQRCodeTracker.h>
#include <vpServoHead.h>
#include <vpServoArm.h>
#include <vpCartesianDisplacement.h>
#include <vpRomeoTkConfig.h>
#include <vpBlobsTargetTracker.h>
#include <vpJointLimitAvoidance.h>

typedef enum {
  CalibratePen,
  WaitPreDraw,
  PreDraw,
  VSpen,
  End
} State_t;



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
  std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);
  bool opt_Reye = false;
  bool opt_calib = true;

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
    else if ( std::string(argv[i]) == "--reye")
      opt_Reye = true;
    else if ( std::string(argv[i]) == "--calib")
      opt_calib = true;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>]" << std::endl;
      return 0;
    }
  }

  std::vector<std::string> chain_name(2); // LArm or RArm
  chain_name[0] = "LArm";
  chain_name[1] = "RArm";


  /************************************************************************************************/
  /** Open the grabber for the acquisition of the images from the robot*/
  vpNaoqiGrabber g;
  g.setFramerate(15);

  // Initialize constant transformations
  vpHomogeneousMatrix eMc;

  if (opt_Reye)
  {
    std::cout << "Using camera Eye Right" << std::endl;
    g.setCamera(3); // CameraRightEye
    eMc = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraRightEye");
  }
  else
  {
    std::cout << "Using camera Eye Right" << std::endl;
    g.setCamera(2); // CameraLeftEye
    eMc = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");
  }
  std::cout << "eMc: " << eMc << std::endl;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.open();
  std::string camera_name = g.getCameraName();
  std::cout << "Camera name: " << camera_name << std::endl;

  vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithDistortion);
  std::cout << "Camera parameters: " << cam << std::endl;


  /** Initialization Visp Image, display and camera paramenters*/
  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "Camera view");

  //Initialize opencv color image
  cv::Mat cvI = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);



  /************************************************************************************************/


  /** Initialization target pen*/

  std::string opt_name_file_color_pen_target = opt_data_folder + "/" +"target/box_blob/color.txt";


  vpBlobsTargetTracker pen_tracker;
  bool status_pen_tracker;
  vpHomogeneousMatrix cMpen;

  const double L = 0.025/2;
  std::vector <vpPoint> points(4);
  points[2].setWorldCoordinates(-L,-L, 0) ;
  points[1].setWorldCoordinates(-L,L, 0) ;
  points[0].setWorldCoordinates(L,L, 0) ;
  points[3].setWorldCoordinates(L,-L,0) ;


  const double L1 = 0.021/2;
  std::vector <vpPoint> points_pen(4);
  points_pen[2].setWorldCoordinates(-L1,-L1, 0) ;
  points_pen[1].setWorldCoordinates(-L1,L1, 0) ;
  points_pen[0].setWorldCoordinates(L1,L1, 0) ;
  points_pen[3].setWorldCoordinates(L1,-L1,0) ;


  pen_tracker.setName("pen");
  pen_tracker.setCameraParameters(cam);
  pen_tracker.setPoints(points_pen);
  //pen_tracker.setManualBlobInit(true);
  pen_tracker.setFullManual(true);

  pen_tracker.setLeftHandTarget(true);

  if(!pen_tracker.loadHSV(opt_name_file_color_pen_target))
  {
    std::cout << "Error opening the file "<< opt_name_file_color_pen_target << std::endl;
  }



  vpHomogeneousMatrix pen_M_point(0.0, 0.0, 0.163, 0.0, 0.0, 0.0);


  /************************************************************************************************/


  /** Initialize the qrcode tracker*/
  bool status_qrcode_tracker;
  vpHomogeneousMatrix cM_tab;
  vpQRCodeTracker qrcode_tracker;
  qrcode_tracker.setCameraParameters(cam);
  qrcode_tracker.setQRCodeSize(0.045);
  qrcode_tracker.setMessage("romeo_left_arm");



  /************************************************************************************************/
  /** Initialization target hands*/

  std::string opt_name_file_color_target_path = opt_data_folder + "/" +"target/";
  std::string opt_name_file_color_target_l = opt_name_file_color_target_path + chain_name[0] +"/color.txt";
  std::string opt_name_file_color_target_r = opt_name_file_color_target_path + chain_name[1] +"/color.txt";

  std::string opt_name_file_color_target1_l = opt_name_file_color_target_path + chain_name[0] +"/color1.txt";
  std::string opt_name_file_color_target1_r = opt_name_file_color_target_path + chain_name[1] +"/color1.txt";

  std::vector<vpBlobsTargetTracker*> hand_tracker;
  std::vector<bool>  status_hand_tracker(2);
  std::vector<vpHomogeneousMatrix>  cMo_hand(2);


  vpBlobsTargetTracker hand_tracker_l;
  hand_tracker_l.setName(chain_name[0]);
  hand_tracker_l.setCameraParameters(cam);
  hand_tracker_l.setPoints(points);
  hand_tracker_l.setLeftHandTarget(true);

  if(!hand_tracker_l.loadHSV(opt_name_file_color_target_l))
    std::cout << "Error opening the file "<< opt_name_file_color_target_l << std::endl;


  vpBlobsTargetTracker hand_tracker_r;
  hand_tracker_r.setName(chain_name[1]);
  hand_tracker_r.setCameraParameters(cam);
  hand_tracker_r.setPoints(points);
  hand_tracker_r.setLeftHandTarget(false);

  if(!hand_tracker_r.loadHSV(opt_name_file_color_target1_r))
    std::cout << "Error opening the file "<< opt_name_file_color_target_r << std::endl;

  hand_tracker.push_back(&hand_tracker_l);
  hand_tracker.push_back(&hand_tracker_r);


  /************************************************************************************************/

  // Constant transformation Target Frame to Arm end-effector (WristPitch)
  std::vector<vpHomogeneousMatrix> hand_Me_Arm(2);
  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  // Create twist matrix from pen to Arm end-effector (WristPitch)
  std::vector <vpVelocityTwistMatrix> pen_Ve_Arm(2);
  // Create twist matrix from target Frame to Arm end-effector (WristPitch)
  std::vector <vpVelocityTwistMatrix> hand_Ve_Arm(2);

  // Constant transformation Target Frame to pen to target Hand
  std::vector<vpHomogeneousMatrix> pen_Mhand(2);


  for (unsigned int i = 0; i < 2; i++)
  {

    std::string name_transform = "qrcode_M_e_" + chain_name[i];
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(hand_Me_Arm[i], filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
      return 0;
    }
    else
      std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << hand_Me_Arm[i] << std::endl;

    hand_Ve_Arm[i].buildFrom(hand_Me_Arm[i]);

  }

  /************************************************************************************************/

  /** Create a new istance NaoqiRobot*/
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();

  std::vector<std::string> jointNames = robot.getBodyNames("Head");
  //jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  std::vector<std::string> jointHeadNames_tot = jointNames;
  jointHeadNames_tot.push_back(jointNamesREye.at(0));
  jointHeadNames_tot.push_back(jointNamesREye.at(1));
  /************************************************************************************************/

  // Initialize state

  State_t state;
  state = CalibratePen;

  /************************************************************************************************/

  std::vector<bool> grasp_servo_converged(2);
  grasp_servo_converged[0]= false;
  grasp_servo_converged[1]= false;


  std::vector<vpMatrix> eJe(2);


  // Initialize arms servoing
  vpServoArm servo_larm_l;
  vpServoArm servo_larm_r;

  std::vector<vpServoArm*> servo_larm;
  servo_larm.push_back(&servo_larm_l);
  servo_larm.push_back(&servo_larm_r);

  std::vector < std::vector<std::string > > jointNames_arm(2);

  jointNames_arm[0] = robot.getBodyNames(chain_name[0]);
  jointNames_arm[1] = robot.getBodyNames(chain_name[1]);
  // Delete last joint Hand, that we don't consider in the servo
  jointNames_arm[0].pop_back();
  jointNames_arm[1].pop_back();

  std::vector<std::string> jointArmsNames_tot = jointNames_arm[0];

  //jointArmsNames_tot.insert(jointArmsNames_tot.end(), jointNames_arm[1].begin(), jointNames_arm[1].end());


  const unsigned int numArmJoints =  jointNames_arm[0].size();
  std::vector<vpHomogeneousMatrix> pen_dMpen(2);

  // Vector containing the joint velocity vectors of the arms
  std::vector<vpColVector> q_dot_arm;
  // Vector containing the joint velocity vectors of the arms for the secondary task
  std::vector<vpColVector> q_dot_arm_sec;
  // Vector joint position of the arms
  std::vector<vpColVector> q;
  // Vector joint real velocities of the arms
  std::vector<vpColVector> q_dot_real;

  vpColVector   q_temp(numArmJoints);
  q_dot_arm.push_back(q_temp);
  q_dot_arm.push_back(q_temp);

  q_dot_arm_sec.push_back(q_temp);
  q_dot_arm_sec.push_back(q_temp);

  q.push_back(q_temp);
  q.push_back(q_temp);

  q_dot_real.push_back(q_temp);
  q_dot_real.push_back(q_temp);



  // Initialize the joint avoidance scheme from the joint limits
  std::vector<vpColVector> jointMin;
  std::vector<vpColVector> jointMax;

  jointMin.push_back(q_temp);
  jointMin.push_back(q_temp);

  jointMax.push_back(q_temp);
  jointMax.push_back(q_temp);

  for (unsigned int i = 0; i< 2; i++)
  {
    jointMin[i] = robot.getJointMin(chain_name[i]);
    jointMax[i] = robot.getJointMax(chain_name[i]);

    jointMin[i].resize(numArmJoints,false);
    jointMax[i].resize(numArmJoints,false);

    //        std::cout <<  jointMin[i].size() << std::endl;
    //        std::cout <<  jointMax[i].size() << std::endl;
    //         std::cout << "max " <<  jointMax[i] << std::endl;
  }



  std::vector<bool> first_time_arm_servo(2);
  first_time_arm_servo[0] = true;
  first_time_arm_servo[1] = true;

  std::vector< double> servo_arm_time_init(2);
  servo_arm_time_init[0] = 0.0;
  servo_arm_time_init[1] = 0.0;

  std::vector<int> cpt_iter_servo_grasp(2);
  cpt_iter_servo_grasp[0] = 0;
  cpt_iter_servo_grasp[1] = 0;

  unsigned int index_hand = 0; //Left Arm

  //  std::string handMpen_name = "handMpen_" + chain_name[index_hand];
  //  std::string filename_transform_pen = std::string(ROMEOTK_DATA_FOLDER) +"/pen/transformation_pen.xml";


  //  vpHomogeneousMatrix hand_M_pen ;
  //  if (state != CalibratePen)
  //  {

  //    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  //    if (pm.parse(hand_M_pen, filename_transform_pen, handMpen_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
  //      std::cout << "Cannot found the homogeneous matrix named " << handMpen_name << "." << std::endl;
  //      return 0;
  //    }
  //    else
  //      std::cout << "Homogeneous matrix " << handMpen_name <<": " << std::endl << hand_M_pen << std::endl;

  //  }

  //    vpHomogeneousMatrix M_offset;
  //    M_offset[1][3] = 0.00;
  //    M_offset[0][3] = 0.00;
  //    M_offset[2][3] = 0.00;

  vpHomogeneousMatrix M_offset;
  M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

  double d_t = 0.01;
  double d_r = 0.0;

  bool first_time_pen_pose = true;

  /************************************************************************************************/
  /** Initialization drawing*/


  vpHomogeneousMatrix p1(0.03, 0.0, 0.0, 0.0, 0.0, 0.0);
  vpHomogeneousMatrix p2(0.03, 0.03, 0.0, 0.0, 0.0, 0.0);
  vpHomogeneousMatrix p3(0.0, 0.03, 0.0, 0.0, 0.0, 0.0);
  vpHomogeneousMatrix p4(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<vpHomogeneousMatrix> P;

  P.push_back(p1);
  P.push_back(p2);
  P.push_back(p3);
  P.push_back(p4);


  /************************************************************************************************/

  vpMouseButton::vpMouseButtonType button;

  vpHomogeneousMatrix elMb; // Homogeneous matrix from right wrist roll to pen
  vpHomogeneousMatrix cMpen_d; // Desired pen final pose



  //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","LEyeYaw", "LEyePitch" );

  //  AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(1.6), vpMath::rad(28.6), vpMath::rad(6.6), vpMath::rad(4.8), vpMath::rad(0.6) , vpMath::rad(13.6), 0.0, 0.0  );
  //  float fractionMaxSpeed  = 0.1f;
  //  robot.getProxy()->setAngles(jointHeadNames_tot, angles_head, fractionMaxSpeed);



  while(1) {
    double loop_time_start = vpTime::measureTimeMs();


    g.acquire(cvI);
    vpImageConvert::convert(cvI, I);
    vpDisplay::display(I);
    bool click_done = vpDisplay::getClick(I, button, false);

    //        if (state < VSpen)
    //        {

    char key[10];
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    std::string s = key;

    if (ret)
    {
      if (s == "r")
      {
        M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

        d_t = 0.0;
        d_r = 0.2;
        std::cout << "Rotation mode. " << std::endl;
      }

      if (s == "t")
      {
        M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

        d_t = 0.01;
        d_r = 0.0;
        std::cout << "Translation mode. " << std::endl;
      }

      if (s == "h")
      {
        hand_tracker[index_hand]->setManualBlobInit(true);
        hand_tracker[index_hand]->setForceDetection(true);
      }
      if (s == "q")
      {
        pen_tracker.setFullManual(true);
        pen_tracker.setForceDetection(true);
      }

      else if ( s == "+")
      {
        unsigned int value = hand_tracker[index_hand]->getGrayLevelMaxBlob() +10;
        hand_tracker[index_hand]->setGrayLevelMaxBlob(value);
        std::cout << "Set to "<< value << "the value of  " << std::endl;

      }
      else if (s == "-")
      {
        unsigned int value = hand_tracker[1]->getGrayLevelMaxBlob()-10;
        hand_tracker[index_hand]->setGrayLevelMaxBlob(value-10);
        std::cout << "Set to "<< value << " GrayLevelMaxBlob. " << std::endl;
      }

      //          |x
      //      z\  |
      //        \ |
      //         \|_____ y
      //

      else if (s == "4") //-y
      {
        M_offset.buildFrom(0.0, -d_t, 0.0, 0.0, -d_r, 0.0) ;
      }

      else if (s == "6")  //+y
      {
        M_offset.buildFrom(0.0, d_t, 0.0, 0.0, d_r, 0.0) ;
      }

      else if (s == "8")  //+x
      {
        M_offset.buildFrom(d_t, 0.0, 0.0, d_r, 0.0, 0.0) ;
      }

      else if (s == "2") //-x
      {
        M_offset.buildFrom(-d_t, 0.0, 0.0, -d_r, 0.0, 0.0) ;
      }

      //            else if (s == "7")//-z
      //            {
      //                M_offset.buildFrom(0.0, 0.0, -d_t, 0.0, 0.0, -d_r) ;
      //            }
      //            else if (s == "9") //+z
      //            {
      //                M_offset.buildFrom(0.0, 0.0, d_t, 0.0, 0.0, d_r) ;
      //            }

      cMpen_d = cMpen_d * M_offset;


    }

    if (state < WaitPreDraw)
    {
      status_hand_tracker[index_hand] = hand_tracker[index_hand]->track(cvI,I);

      if (status_hand_tracker[index_hand] ) { // display the tracking results
        cMo_hand[index_hand] = hand_tracker[index_hand]->get_cMo();
        //printPose("cMo qrcode: ", cMo_hand);
        // The qrcode frame is only displayed when PBVS is active or learning

        vpDisplay::displayFrame(I, cMo_hand[index_hand], cam, 0.04, vpColor::none, 3);
      }

    }

    status_pen_tracker = pen_tracker.track(cvI,I);

    if (status_pen_tracker ) { // display the tracking results
      cMpen = pen_tracker.get_cMo();
      //printPose("cMo qrcode: ", cMo_hand);
      // The qrcode frame is only displayed when PBVS is active or learning

      vpDisplay::displayFrame(I, cMpen, cam, 0.04, vpColor::none, 3);
    }


    if (state == CalibratePen &&  status_hand_tracker[index_hand] )
    {

      if (status_pen_tracker)
      {
        vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to calibrate pen", vpColor::red);

        cMpen = pen_tracker.get_cMo();
        //printPose("cMo qrcode: ", cMo_hand);
        vpDisplay::displayFrame(I, cMpen, cam, 0.04, vpColor::none, 1);

        vpHomogeneousMatrix pen_Me_Arm; // Homogeneous matrix from the pen to the left wrist
        pen_Mhand[index_hand] = cMpen.inverse() *  cMo_hand[index_hand];
        pen_Me_Arm = pen_Mhand[index_hand] * hand_Me_Arm[index_hand] ; // from pen to WristPitch


        if (click_done && button == vpMouseButton::button1) {


          pen_Ve_Arm[index_hand].buildFrom(pen_Me_Arm);
          //          vpXmlParserHomogeneousMatrix p_; // Create a XML parser

          //          if (p_.save(hand_M_pen, "transformation_pen.xml", handMpen_name) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
          //          {
          //            std::cout << "Cannot save the Homogeneous matrix" << std::endl;
          //            return 0;
          //          }
          //          std::cout << "Homogeneous matrix from the hand to the pen saved " << std::endl;
          //          hand_M_pen.print();

          state = WaitPreDraw;
          click_done = false;

        }

      }

    }



    if (state == WaitPreDraw  )
    {
      vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to continue", vpColor::red);

      status_qrcode_tracker = qrcode_tracker.track(I);

      if (status_qrcode_tracker) { // display the tracking results
        cM_tab = qrcode_tracker.get_cMo();
        //printPose("cMo qrcode: ", cMo_qrcode);
        // The qrcode frame is only displayed when PBVS is active or learning

        vpDisplay::displayFrame(I, cM_tab, cam, 0.07, vpColor::none, 2);
      }

      vpHomogeneousMatrix cM_point = cMpen * pen_M_point;

      // vpDisplay::displayFrame(I, cM_point, cam, 0.03, vpColor::none, 2);

      //      vpHomogeneousMatrix cP1 = cM_tab * P1;

      vpHomogeneousMatrix cMpen_t = cM_tab;
      cMpen_t[0][3] = cM_point[0][3];
      cMpen_t[1][3] = cM_point[1][3];
      cMpen_t[2][3] = cM_point[2][3];

      vpHomogeneousMatrix cMpen_p1 = cMpen_t * p1;
      vpDisplay::displayFrame(I, cMpen_t, cam, 0.03, vpColor::none, 2);

      vpDisplay::displayFrame(I, cMpen_p1, cam, 0.03, vpColor::none, 2);

      vpHomogeneousMatrix cMpen_p1_rot_hand = cMpen;
      cMpen_p1_rot_hand[0][3] = cMpen_p1[0][3];
      cMpen_p1_rot_hand[1][3] = cMpen_p1[1][3];
      cMpen_p1_rot_hand[2][3] = cMpen_p1[2][3];
      vpDisplay::displayFrame(I, cMpen_p1_rot_hand, cam, 0.03, vpColor::none, 2);



      cMpen_d = cMpen_p1_rot_hand * pen_M_point.inverse();
      vpDisplay::displayFrame(I, cMpen_d, cam, 0.03, vpColor::none, 2);

      if (click_done && button == vpMouseButton::button1 ) {

        state = PreDraw;
        click_done = false;
      }

    }

    if (state == PreDraw && status_pen_tracker )
    {

      vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to start the servo", vpColor::red);

      //      if (first_time_pen_pose)
      //      {
      //        // Compute desired pen position cMpen_d
      //        cMpen_d = cMpen * M_offset;
      //        first_time_pen_pose = false;
      //      }

      vpDisplay::displayFrame(I, cMpen_d , cam, 0.05, vpColor::none, 3);
      //      vpDisplay::displayFrame(I, cMpen *pen_Mhand[1] , cam, 0.05, vpColor::none, 3);
      //      vpDisplay::displayFrame(I, cMpen *pen_Mhand[0] , cam, 0.05, vpColor::none, 3);

      if (click_done && button == vpMouseButton::button1 ) {

        state = VSpen;
        click_done = false;
      }
    }


    if (state == VSpen)
    {
      //Get Actual position of the arm joints
      q[0] = robot.getPosition(jointNames_arm[0]);
      q[1] = robot.getPosition(jointNames_arm[1]);

      //  if(status_pen_tracker && status_hand_tracker[index_hand] )
      if(status_pen_tracker )
      {

        if (! grasp_servo_converged[0]) {
          //                    for (unsigned int i = 0; i<2; i++)
          //                    {

          unsigned int i = 0;
          //vpAdaptiveGain lambda(0.8, 0.05, 8);
          vpAdaptiveGain lambda(0.4, 0.02, 4);

          //servo_larm[i]->setLambda(lambda);
          servo_larm[i]->setLambda(0.2);


          eJe[i] = robot.get_eJe(chain_name[i]);

          servo_larm[i]->set_eJe(eJe[i]);
          servo_larm[i]->m_task.set_cVe(pen_Ve_Arm[i]);

          pen_dMpen[i] = cMpen_d.inverse() * cMpen;
          //          printPose("pen_dMpen: ", pen_dMpen[i]);


          servo_larm[i]->setCurrentFeature(pen_dMpen[i]) ;

          vpDisplay::displayFrame(I, cMpen_d , cam, 0.05, vpColor::none, 3);


          //                    vpDisplay::displayFrame(I, cMpen *pen_Mhand[1] , cam, 0.05, vpColor::none, 3);
          //                    vpDisplay::displayFrame(I, cMpen *pen_Mhand[0] , cam, 0.05, vpColor::none, 3);

          if (first_time_arm_servo[i]) {
            std::cout << "-- Start visual servoing of the arm" << chain_name[i] << "." << std::endl;
            servo_arm_time_init[i] = vpTime::measureTimeSecond();
            first_time_arm_servo[i] = false;
          }

          q_dot_arm[i] =  - servo_larm[i]->computeControlLaw(servo_arm_time_init[i]);

          // Compute joint limit avoidance
          q_dot_arm_sec[0]  = servo_larm[0]->m_task.secondaryTaskJointLimitAvoidance(q[0], q_dot_real[0], jointMin[0], jointMax[0]);

          cpt_iter_servo_grasp[i] ++;


          vpColVector q_dot_tot = q_dot_arm[0] + q_dot_arm_sec[0];

          robot.setVelocity(jointArmsNames_tot, q_dot_tot);

          vpTranslationVector t_error_grasp = pen_dMpen[0].getTranslationVector();
          vpRotationMatrix R_error_grasp;
          pen_dMpen[0].extract(R_error_grasp);
          vpThetaUVector tu_error_grasp;
          tu_error_grasp.buildFrom(R_error_grasp);
          double theta_error_grasp;
          vpColVector u_error_grasp;
          tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
          std::cout << "error: " << sqrt(t_error_grasp.sumSquare()) << " " << vpMath::deg(theta_error_grasp) << std::endl;


          //                    if (cpt_iter_servo_grasp[0] > 100) {

          //                        vpDisplay::displayText(I, vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);
          //                    }
          //                    double error_t_treshold = 0.007;

          //                    if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(3)) || (click_done && button == vpMouseButton::button1 /*&& cpt_iter_servo_grasp > 150*/) )
          //                    {
          //                        robot.stop(jointArmsNames_tot);

          //                        state = End;
          //                        grasp_servo_converged[0] = true;

          //                        if (click_done && button == vpMouseButton::button1)
          //                            click_done = false;
          //                    }


        }

      }
      else {
        // state grasping but one of the tracker fails
        robot.stop(jointArmsNames_tot);
      }

    }

    if (state == End)
    {
      std::cout<< "End" << std::endl;
    }

    vpDisplay::flush(I) ;

    if (click_done && button == vpMouseButton::button3) { // Quit the loop
      break;
    }
    //std::cout << "Loop time: " << vpTime::measureTimeMs() - loop_time_start << std::endl;

  }

  robot.stop(jointArmsNames_tot);


}
