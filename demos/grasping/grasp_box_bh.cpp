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
    LearnBoxDetection,
    WaitGrasping, // wait for a click to start grasping
    Grasping,
    TakeTea,
    Interaction
} StateTeaboxTracker_t;



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
            // pos1 = AL::ALValue::array(0.38404589891433716, -0.23612679541110992, -0.09724850952625275, 1.4714961051940918, 0.5567980408668518, 0.2787119448184967);
            //       pos2 = AL::ALValue::array(0.3702833652496338, -0.34589311480522156, 0.23645465075969696, 1.3869593143463135, -0.2769468426704407, -0.16718725860118866);
            pos1 = AL::ALValue::array(0.3692784905433655, -0.35209423303604126, -0.07788902521133423, 1.2388615608215332, 0.6193198561668396, -0.17408452928066254);
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
        Mhack[1][3] = +0.020; // add Y - 0.025 offset
        tMh_desired = tMh_desired * Mhack;

    }
    else
    {

        Mhack[1][3] = -0.030; // add Y - 0.025 offset
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

    //  std::string opt_model = std::string(ROMEOTK_DATA_FOLDER) + "/milkbox/milkbox";
    //  std::string config_detection_file_folder = std::string(ROMEOTK_DATA_FOLDER) + "/milkbox/";
    //  //std::string learning_data_file_name = "teabox_learning_data_test.bin";
    //  std::string learning_data_file_name = "milkbox/milk_learning_data.bin";

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
    bool opt_no_color_tracking = false;
    bool opt_Reye = false;
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
        else if (std::string(argv[i]) == "--Reye")
            opt_Reye = true;
        else if (std::string(argv[i]) == "--rarm")
            opt_right_arm = true;
        else if (std::string(argv[i]) == "--haar")
            opt_face_cascade_name = std::string(argv[i+1]);
        else if (std::string(argv[i]) == "--help") {
            std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--box-name] [--opt_no_color_tracking]" << std::endl;
            std::cout << "       [--haar <haarcascade xml filename>] [--no-interaction] [--learn-open-loop-position] " << std::endl;
            std::cout << "       [--learn-grasp-position] [--plot-time] [--plot-arm] [--plot-qrcode-pose] [--plot-q] "<< std::endl;
            std::cout << "  add  [--rarm] tu use the right arm, nothing to use the left "<< std::endl;
            std::cout << "       [--data-folder] [--learn-detection-box] [--Reye] "<< std::endl;
            std::cout << "       [--english] [--opt-record-video] [--help]" << std::endl;
            return 0;
        }
    }

    if (opt_interaction && ! vpIoTools::checkFilename(opt_face_cascade_name)) {
        std::cout << "Error: the file " << opt_face_cascade_name <<  " doesn't exist." << std::endl;
        std::cout << "Use --haar <haarcascade xml filename> or --no-interaction to disable face detection " << std::endl;
        return 0;
    }

    std::string chain_name; // LArm or RArm
    if (opt_right_arm)
    {
        chain_name = "RArm";
        // opt_Reye = true;
    }
    else
    {
        chain_name = "LArm";
    }


    if (opt_learning_detection)
        opt_no_color_tracking = true;

    StateTeaboxTracker_t state_teabox_tracker;
    if (opt_no_color_tracking)
        state_teabox_tracker = Acquisition;
    else
        state_teabox_tracker = HeadToZero; // Acquisition; //Interaction; //TakeTea;

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

    vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithoutDistortion);
    std::cout << "Camera parameters: " << cam << std::endl;

    /** Create a new istance NaoqiRobot*/
    vpNaoqiRobot robot;
    if (! opt_ip.empty())
        robot.setRobotIp(opt_ip);
    robot.open();

    /** Initialization Visp Image, display and camera paramenters*/
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "Right camera view");

    //Initialize opencv color image
    cv::Mat cvI = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);


    // Initialization detection and localiztion teabox
    vpMbLocalization teabox_tracker(opt_model, config_detection_file_folder, cam);
    teabox_tracker.initDetection(learning_data_file_name);
    teabox_tracker.setValiditycMoFunction(checkValiditycMo);
    bool onlyDetection = true;
    teabox_tracker.setOnlyDetection(onlyDetection);

    bool status_teabox_tracker = false; // false if the tea box tracker fails
    vpHomogeneousMatrix cMo_teabox;

    // Initialize the qrcode tracker
    bool status_qrcode_tracker;
    vpHomogeneousMatrix cMo_qrcode;
    vpQRCodeTracker qrcode_tracker;
    qrcode_tracker.setCameraParameters(cam);
    if (opt_right_arm)
    {
        qrcode_tracker.setQRCodeSize(0.045);
        qrcode_tracker.setMessage("romeo_right_arm");
    }
    else
    {
        qrcode_tracker.setQRCodeSize(0.045);
        qrcode_tracker.setMessage("romeo_left_arm");
    }

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    bool teabox_servo_converged = false;
    double servo_time_init = 0;
    double servo_head_time_init = 0;
    double servo_arm_time_init = 0;
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


    // Map to don't consider the HeadRoll
    vpMatrix MAP_head(6,5);
    for (unsigned int i = 0; i < 3 ; i++)
        MAP_head[i][i]= 1;
    MAP_head[4][3]= 1;
    MAP_head[5][4]= 1;


    // Initialize Detection color class

    vpColorDetection obj_color;
    obj_color.setName(opt_box_name);
    std::string filename_color = box_folder + "color/" + opt_box_name + "HSV.txt";
    if (!obj_color.loadHSV(filename_color))
    {
        std::cout << "Cannot load file " << filename_color << std::endl;
        return 0;
    }


    // Constant transformation Target Frame to Arm end-effector (WristPitch)
    vpHomogeneousMatrix oMe_Arm;

    std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
    std::string name_transform = "qrcode_M_e_" + chain_name;
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMe_Arm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
        return 0;
    }
    else
        std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;

    // Create twist matrix from target Frame to Arm end-effector (WristPitch)
    vpVelocityTwistMatrix oVe_Arm(oMe_Arm);


    // Initialize constant parameters
    std::string learned_oMh_filename = "grasping_pose.xml"; // This file contains the following two transf. matrix:
    std::string learned_oMh_path = box_folder + "grasping/" + chain_name; // This file contains the following two transf. matrix:

    std::string name_oMh_open_loop =  "oMh_open_loop_" + camera_name; // Offset position Hand w.r.t the object (Open loop)
    std::string name_oMh_grasp =  "oMh_close_loop_"+ camera_name; // Offset position Hand w.r.t the object to grasp it (Close loop)

    vpHomogeneousMatrix oMh_Tea_Box_grasp;
    if (! opt_learn_grasp_position && ! opt_learn_open_loop_position) {
        vpXmlParserHomogeneousMatrix pm; // Create a XML parser

        if (pm.parse(oMh_Tea_Box_grasp, learned_oMh_path + "/" + learned_oMh_filename, name_oMh_grasp) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
            std::cout << "Cannot found the homogeneous matrix named " << name_oMh_grasp<< "." << std::endl;
            return 0;
        }
        else
            std::cout << "Homogeneous matrix " << name_oMh_grasp <<": " << std::endl << oMh_Tea_Box_grasp << std::endl;
    }

    bool grasp_servo_converged = false;
    vpServoArm servo_larm; // Initialize arm servoing
    std::vector<std::string> jointNames_larm =  robot.getBodyNames(chain_name);
    jointNames_larm.pop_back(); // Delete last joints LHand, that we don't consider in the servo
    const unsigned int numArmJoints =  jointNames_larm.size();
    vpHomogeneousMatrix cdMc;

    //Condition number Jacobian Arm
    double cond = 0.0;

    // Initalization data for the joint avoidance limit
    //Vector data for plotting
    vpColVector data(13);

    // Initialize the joint avoidance scheme from the joint limits
    vpColVector jointMin = robot.getJointMin(chain_name);
    vpColVector jointMax = robot.getJointMax(chain_name);

    jointMin.resize(numArmJoints,false);
    jointMax.resize(numArmJoints,false);


    // Vector joint position of the arm
    vpColVector q(numArmJoints);

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

    std::vector<std::string> joint_names_arm_head = jointNames_larm;
    joint_names_arm_head.insert(joint_names_arm_head.end(), jointNames_tot.begin(), jointNames_tot.end());


    // Initialize arm open loop servoing
    bool arm_moved = false;

    //Set the stiffness
    robot.setStiffness(jointNames_head, 1.f);
    robot.setStiffness(jointNames_larm, 1.f);
    vpColVector q_dot_head(jointNames_head.size(), 0);
    vpColVector q_dot_tot;
    vpColVector q_dot_larm(jointNames_larm.size(), 0);
    //vpColVector q_dot_arm_head(joint_names_arm_head.size(), 0);

    //Open the Hand
    robot.getProxy()->setAngles("LHand", 1., 1.);
    robot.getProxy()->setAngles("RHand", 1., 1.);

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

        plotter_arm->setLegend(0, 0, "tx");
        plotter_arm->setLegend(0, 1, "ty");
        plotter_arm->setLegend(0, 2, "tz");
        plotter_arm->setLegend(1, 0, "tux");
        plotter_arm->setLegend(1, 1, "tuy");
        plotter_arm->setLegend(1, 2, "tuz");

        plotter_arm->setLegend(1, 0, "LshouderPitch");
        plotter_arm->setLegend(1, 1, "LShoulderYaw");
        plotter_arm->setLegend(1, 2, "LElbowRoll");
        plotter_arm->setLegend(1, 3, "LElbowYaw");
        plotter_arm->setLegend(1, 4, "LWristRoll");
        plotter_arm->setLegend(1, 5, "LWristYaw");
        plotter_arm->setLegend(1, 6, "LWristPitch");


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

        plotter_q->setColor(0, 7,vpColor::darkRed);
        plotter_q->setThickness(0, 7,2);
        plotter_q->setColor(0, 9,vpColor::darkRed);
        plotter_q->setThickness(0, 9,2);
        plotter_q->setColor(0, 11,vpColor::darkRed);
        plotter_q->setThickness(0, 11,2);

        plotter_q->setColor(0, 8,vpColor::darkRed);
        plotter_q->setThickness(0, 8,2);
        plotter_q->setColor(0, 10,vpColor::darkRed);
        plotter_q->setThickness(0, 10,2);
        plotter_q->setColor(0, 12,vpColor::darkRed);
        plotter_q->setThickness(0, 12,2);

    }

    vpPlot *plotter_q_sec_arm;
    if (opt_plotter_q_sec_arm) {
        plotter_q_sec_arm = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+30, "Secondary Task");
        plotter_q_sec_arm->initGraph(0, 7); // translations
        plotter_q_sec_arm->initGraph(1, 7); // rotations

        plotter_q_sec_arm->setTitle(0, "Q2 values");
        plotter_q_sec_arm->setTitle(1, "Q tot values");

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


    vpPlot *plotter_cond;
    if (0) {
        plotter_cond = new vpPlot(1, I.getHeight(), I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Loop time");
        plotter_cond->initGraph(0, 1);
    }


    vpPlot *plotter_time;
    if (opt_plotter_time) {
        plotter_time = new vpPlot(1, I.getHeight(), I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Loop time");
        plotter_time->initGraph(0, 1);
    }

    while(1) {
        double loop_time_start = vpTime::measureTimeMs();
        //std::cout << "Loop iteration: " << loop_iter << std::endl;

        g.acquire(cvI);
        vpImageConvert::convert(cvI, I);


        //g.acquire(I);

        vpDisplay::display(I);

        //vpImageIo::write(I, "milkbox.ppm");

        //Get Actual position of the arm joints
        q = robot.getPosition(jointNames_larm);


        if (! opt_record_video)
            vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Right click to quit", vpColor::red);

        bool click_done = vpDisplay::getClick(I, button, false);

       if(state_teabox_tracker == HeadToZero) {

            if(opt_learn_grasp_position || opt_learn_open_loop_position)
            {
                state_teabox_tracker = Acquisition;
            }
            else
            {
                head_pose = 0;
                head_pose[1] = vpMath::rad(-8.); // NeckPitch
                head_pose[2] = vpMath::rad(-13.); // HeadPitch
                robot.setPosition(jointNames_tot, head_pose, 0.06);
                state_teabox_tracker = WaitHeadToZero;
            }

        }

        if(state_teabox_tracker == WaitHeadToZero) {
            vpColVector head_pose_mes = robot.getPosition(jointNames_tot);
            if (sqrt((head_pose_mes-head_pose).sumSquare()) < vpMath::rad(4))
                state_teabox_tracker = TrackColorObject;
        }


        if(state_teabox_tracker == TrackColorObject) {

            vpDisplay::displayText(I, vpImagePoint(10,10), "Color detection. Click left to continue", vpColor::red);


            static bool reinit_servo = true;

            if (reinit_servo) {
                servo_time_init = vpTime::measureTimeSecond();
                reinit_servo = false;

            }


            bool obj_found =  obj_color.detect(cvI);


            if (obj_found) {

                bool static firstTime = true;

                if (firstTime) {

                    if (opt_box_name == "tabascobox")
                        tts.post.say(" \\rspd=80\\ \\pau=1000\\ \\emph=2\\ What a beatiful orange box  \\eos=1\\ \\wait=5\\ \\emph=2\\ Can you put it on the table ? \\wait=2\\  \\emph=2\\ please!\\eos=1\\  " );
                    else
                        tts.post.say(" \\rspd=80\\ \\pau=1000\\ \\emph=2\\ What a beatiful " + opt_box_name + " \\eos=1\\ \\wait=5\\ \\emph=2\\ Can you put it on the table ? \\wait=2\\  \\emph=2\\ please!\\eos=1\\  " );
                    firstTime = false;
                }

                unsigned int j = 0; //Biggest object

                vpRect bbox =  obj_color.getBBox(j);
                vpDisplay::displayRectangle(I, bbox, vpColor::yellow, false, 1);
                vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), obj_color.getMessage(j), vpColor::yellow);

                vpImagePoint head_cog_cur = obj_color.getCog(j);
                vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);

                vpMatrix eJe;
                if (opt_Reye)
                    eJe = robot.get_eJe("REye");
                else
                    eJe = robot.get_eJe("LEye");
                servo_head.set_eJe( eJe );
                servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );


                vpAdaptiveGain lambda(2, 2.0, 30); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
                servo_head.setLambda(lambda);

                servo_head.setCurrentFeature(head_cog_cur);
                servo_head.setDesiredFeature(head_cog_des);
                vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);


                q_dot_head = servo_head.computeControlLaw(servo_time_init);
                // Add mirroring eyes
                q_dot_tot = q_dot_head;
                std::cout << "q = " << q_dot_tot << std::endl;
                q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
                q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

                robot.setVelocity(jointNames_tot, q_dot_tot);

            }

            else {
                robot.stop(jointNames_tot);
                reinit_servo = true;
            }


            if (click_done && button == vpMouseButton::button1 ) {
                robot.stop(jointNames_tot);
                state_teabox_tracker = Acquisition;
                click_done = false;
            }


        }

        // track teabox
        if(state_teabox_tracker == Acquisition) {

            if (! opt_record_video && !opt_learning_detection)
                vpDisplay::displayText(I, vpImagePoint(10,10), "Left click: automatic detection, Central: manual", vpColor::red);

            // Move the head in the default position
            if(!opt_learn_grasp_position && !opt_learn_open_loop_position && opt_no_color_tracking )
            {
                //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","LEyeYaw", "LEyePitch" );
                AL::ALValue angles_head;
                if (opt_right_arm)
                    //angles_head      = AL::ALValue::array(vpMath::rad(-8.3), vpMath::rad(19), vpMath::rad(11.4), vpMath::rad(0), 0.0 , 0.0, 0.0, 0.0  );
                    angles_head      = AL::ALValue::array(vpMath::rad(-7.5), vpMath::rad(18.9), vpMath::rad(11.0), vpMath::rad(0.0), vpMath::rad(2.4) , vpMath::rad(1.4), 0.0, 0.0  );
                else
                    angles_head      = AL::ALValue::array(vpMath::rad(4.3), vpMath::rad(24.3), vpMath::rad(3.7), vpMath::rad(0.0), 0.0 , 0.0, 0.0, 0.0  );
                float fractionMaxSpeed  = 0.1f;
                robot.getProxy()->setAngles(jointNames_tot, angles_head, fractionMaxSpeed);

            }


            if (opt_learning_detection)
            {
                state_teabox_tracker = LearnBoxDetection;
            }

            else{
                if ( teabox_tracker.track(I) )
                {
                    cMo_teabox =  teabox_tracker.get_cMo();
                    teabox_tracker.getTracker()->display(I, cMo_teabox, cam, vpColor::red, 2);
                    vpDisplay::displayFrame(I, cMo_teabox, cam, 0.025, vpColor::none, 3);
                }

                if (click_done && button == vpMouseButton::button1 ) {
                    onlyDetection = false;
                    state_teabox_tracker = WaitTeaBoxTracking;
                    teabox_tracker.setOnlyDetection(onlyDetection); // Stop only detection
                    click_done = false;
                }

                if (click_done && button == vpMouseButton::button2 ) {
                    teabox_tracker.setManualDetection();
                    state_teabox_tracker = TeaBoxTracking;
                    click_done = false;
                }
            }
        }


        if (state_teabox_tracker == WaitTeaBoxTracking) {
            vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to continue, Central to re-detect", vpColor::red);

            if ( teabox_tracker.track(I))
            {
                cMo_teabox =  teabox_tracker.get_cMo();
                teabox_tracker.getTracker()->display(I, cMo_teabox, cam, vpColor::red, 2);
                vpDisplay::displayFrame(I, cMo_teabox, cam, 0.025, vpColor::none, 3);
            }

            if (click_done && button == vpMouseButton::button1) {
                state_teabox_tracker = TeaBoxTracking;
                click_done = false;
            }

            else if (click_done && button == vpMouseButton::button2)
            {
                state_teabox_tracker = Acquisition;
                teabox_tracker.setForceDetection();
                onlyDetection = true;
                teabox_tracker.setOnlyDetection(onlyDetection); // restart only detection
                click_done = false;
            }

        }

        if ( (state_teabox_tracker == TeaBoxTracking || state_teabox_tracker == MoveToDesiredLHandPosition
              || state_teabox_tracker == LearnDesiredLHandOpenLoopPosition || state_teabox_tracker == LearnDesiredLHandGraspPosition
              || state_teabox_tracker == WaitGrasping || state_teabox_tracker == Grasping) && !opt_learning_detection) {
            try {
                if ( teabox_tracker.track(I))
                {
                    cMo_teabox =  teabox_tracker.get_cMo();

                    teabox_tracker.getTracker()->display(I, cMo_teabox, cam, vpColor::red, 2);
                    vpDisplay::displayFrame(I, cMo_teabox, cam, 0.025, vpColor::none, 3);


                    if (opt_learn_open_loop_position)
                        state_teabox_tracker = LearnDesiredLHandOpenLoopPosition;
                    else if (opt_learn_grasp_position)
                        state_teabox_tracker = LearnDesiredLHandGraspPosition;
                    else if (! arm_moved)
                        state_teabox_tracker = MoveToDesiredLHandPosition;

                    status_teabox_tracker = true;

                }
            }
            catch(...) {
                status_teabox_tracker = false;
                //state_teabox_tracker = Acquisition;
            }
        }

        // track qrcode
        //    if (state_teabox_tracker == TakeTea)
        //      qrcode_tracker.setForceDetection(true);
        //    else
        //      qrcode_tracker.setForceDetection(false);
        status_qrcode_tracker = qrcode_tracker.track(I);

        if (status_qrcode_tracker && !opt_learning_detection) { // display the tracking results
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
            P.setWorldCoordinates(0.05/2, 0.05/2, -0.15/2);
            P.project(cMo_teabox);
            double u=0, v=0;
            vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), u, v);
            teabox_cog_cur.set_uv(u, v);

            // vpAdaptiveGain lambda(1.5, .6, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
            vpAdaptiveGain lambda(1.5, .3, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
            servo_head.setLambda(lambda);


            vpMatrix eJe_head;
            if (opt_Reye)
                eJe_head = robot.get_eJe("REye");
            else
                eJe_head = robot.get_eJe("LEye");

            servo_head.set_eJe(eJe_head);
            servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

            servo_head.setCurrentFeature(teabox_cog_cur);
            vpImagePoint teabox_cog_des;

            if(opt_right_arm)
            {
                if (opt_box_name == "milkbox")
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*1/8 );
                else if (opt_box_name =="spraybox")
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*2/8 );
                else if (opt_box_name =="tabascobox")
                    teabox_cog_des.set_ij( I.getHeight()*5.3/8, I.getWidth()*1.3/8 );
                else
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*2/8 );

            }

            else
            {

                if (opt_box_name == "milkbox")
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*7/8 );
                else if (opt_box_name =="spraybox")
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*6/8 );
                else if (opt_box_name =="tabascobox")
                    teabox_cog_des.set_ij( I.getHeight()*5/8, I.getWidth()*6.6/8 );
                else
                    teabox_cog_des.set_ij( I.getHeight()*6/8, I.getWidth()*5.5/8 );
            }

            servo_head.setDesiredFeature( teabox_cog_des );
            //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::blue, 3);
            if (! opt_record_video)
                vpDisplay::displayCross(I, teabox_cog_cur, 15, vpColor::blue, 3); // current feature
            vpDisplay::displayCross(I, teabox_cog_des, 15, vpColor::green, 3); // desired feature

            q_dot_head = servo_head.computeControlLaw(servo_time_init);

            // Add mirroring eyes
            q_dot_tot = q_dot_head;
            q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
            q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

            robot.setVelocity(jointNames_tot, q_dot_tot);

            if (cpt_iter_servo_head > 100) {
                if (opt_record_video)
                    vpDisplay::displayText(I, vpImagePoint(10,10), "Click to continue", vpColor::red);
                else
                    vpDisplay::displayText(I, vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);

            }
            if ( sqrt(servo_head.m_task_head.getError().sumSquare())*cam.get_px() < 10. || (click_done && button == vpMouseButton::button1 && cpt_iter_servo_head > 100) )
            {
                robot.stop(jointNames_tot);
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
                if (learnDesiredLHandOpenLoopPosition(robot, cMo_teabox, eMc, learning_folder + "/" + learned_oMh_filename, name_oMh_open_loop, teaboxMh_offset, opt_Reye, chain_name)) {
                    printPose("The learned open loop pose: ", teaboxMh_offset);
                    std::cout << "is saved in " << learning_folder + "/" + learned_oMh_filename << std::endl;
                    return 0;
                }
            }
        }

        if (state_teabox_tracker == LearnDesiredLHandGraspPosition && status_qrcode_tracker && status_teabox_tracker) {
            vpDisplay::displayText(I, vpImagePoint(10,10), "Put arm in grasping position", vpColor::red);
            vpDisplay::displayText(I, vpImagePoint(25,10), "and left click to learn the pose", vpColor::red);
            if (click_done && button == vpMouseButton::button1) {
                vpHomogeneousMatrix teaboxMh_grasp;
                if (learnDesiredLHandGraspingPosition(robot, cMo_teabox, cMo_qrcode, learning_folder + "/" + learned_oMh_filename, name_oMh_grasp, teaboxMh_grasp)) {
                    printPose("The learned grasping pose: ", teaboxMh_grasp);
                    std::cout << "is saved in " << learning_folder + "/" + learned_oMh_filename << std::endl;
                    return 0;
                }
            }
        }

        if (state_teabox_tracker == LearnBoxDetection)
        {

            vpDisplay::displayText(I, 10, 10, "Click left to continue with detection...", vpColor::red);
            vpDisplay::displayText(I, 30, 10, "Click central to save...", vpColor::red);

            if (click_done && button == vpMouseButton::button1) {
                click_done = false;
                teabox_tracker.learnObject(I);

            }

            else if (click_done && button == vpMouseButton::button2)
            {

                //std::string path_folder = learning_folder + "/" + objects_folder + "learning/";
                std::string path_folder = learning_folder + "/learning_" + opt_box_name + "_" + currentDateTime() + "/";
                // Test if the output path exist. If no try to create it
                if (vpIoTools::checkDirectory(path_folder) == false) {
                    try {
                        // Create the dirname
                        vpIoTools::makeDirectory(path_folder);
                    }
                    catch (vpException &e) {
                        std::cout << "Cannot create " << path_folder << std::endl;
                        std::cout << "Error: " << e.getMessage() <<std::endl;
                        return 0;
                    }
                }

                std::string path_file = path_folder + learning_detection_file;
                teabox_tracker.saveLearningData(path_file);
                std::cout << "Learning data are saved in " << path_file << std::endl;
                click_done = false;
                return 0;
            }
        }


        if (state_teabox_tracker == MoveToDesiredLHandPosition && status_teabox_tracker && teabox_servo_converged) {
            if (! opt_record_video)
            {
                vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to move the arm from rest position open loop", vpColor::red);
                vpDisplay::displayText(I, vpImagePoint(25,10), "Middle click to move the arm from current position in open loop", vpColor::red);
            }
            if (click_done) {

                vpHomogeneousMatrix handMbox_desired = getOpenLoopDesiredPose(robot, cMo_teabox, eMc, learned_oMh_path + "/" + learned_oMh_filename, name_oMh_open_loop, opt_Reye, chain_name);
                std::vector<float> handMbox_desired_;
                handMbox_desired.convert(handMbox_desired_);

                switch(button) {

                case vpMouseButton::button1:{
                    moveLArmFromRestPosition(robot, handMbox_desired_ ,chain_name); // move up
                    click_done = false;
                }break;

                case vpMouseButton::button2:{
                    float velocity = 0.2;
                    int axis_mask = 63; // Control position and orientation
                    robot.getProxy()->setTransform(chain_name, 0, handMbox_desired_, velocity, axis_mask);
                    click_done = false;
                }break;

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

                vpImagePoint teabox_cog_cur;
                vpImagePoint qrcode_cog_cur;
                vpPoint P;
                P.setWorldCoordinates(0.065/2, 0.045/2, -0.1565/2);
                P.project(cMo_teabox);
                double u=0, v=0;
                vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), u, v);
                teabox_cog_cur.set_uv(u, v);
                qrcode_cog_cur = qrcode_tracker.getCog();

                vpMatrix eJe_head;
                if (opt_Reye)
                    eJe_head = robot.get_eJe("REye");
                else
                    eJe_head = robot.get_eJe("LEye");

                servo_head.set_eJe(eJe_head);
                servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

                //vpAdaptiveGain lambda(1, .4, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
                vpAdaptiveGain lambda(0.6, .3, 10); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30

                servo_head.setLambda(lambda);

                servo_head.setCurrentFeature( (teabox_cog_cur + qrcode_cog_cur)/2 );
                servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*5/8, I.getWidth()/2) );
                vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::yellow, 3);

                if (1) {
                    static bool first_time_head_servo = true;
                    if (first_time_head_servo) {
                        std::cout << "-- Start visual servoing of the head" << std::endl;
                        servo_head_time_init = vpTime::measureTimeSecond();
                        first_time_head_servo = false;
                        //g.setCameraParameter(AL::kCameraAutoExpositionID, 0);
                        //g.setCameraParameter(AL::kCameraAutoWhiteBalanceID, 0);
                    }

                q_dot_head = servo_head.computeControlLaw(servo_head_time_init);

                }

                // Servo arm -pregraps
                if (! grasp_servo_converged) {


                    //          vpMatrix oJo = oVe_Arm * robot.get_eJe("LArm");
                    //          servo_larm.setLambda(0.15);
                    //          //vpAdaptiveGain lambda(0.3, .2, 15); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
                    //          //servo_larm.setLambda(lambda);
                    //          servo_larm.set_eJe(oJo);

                    //          std::string name_last_eye_joint;
                    //          if (opt_Reye)
                    //            name_last_eye_joint = "REyePitch";
                    //          else
                    //            name_last_eye_joint = "LEyePitch";

                    //          vpHomogeneousMatrix torsoMLEyePitch(robot.getProxy()->getTransform(name_last_eye_joint, 0, true));
                    //          vpVelocityTwistMatrix cVtorso( (torsoMLEyePitch * eMc).inverse());
                    //          servo_larm.set_cVf( cVtorso );

                    //          vpHomogeneousMatrix torsoMLWristPitch(robot.getProxy()->getTransform("LWristPitch", 0, true));
                    //          vpVelocityTwistMatrix torsoVo(torsoMLWristPitch * oMe_Arm.inverse());
                    //          servo_larm.set_fVe( torsoVo );

                    //          // Compute the desired position of the hand taking into account the off-set
                    //          //cdMc = cMo_teabox * oMh_Tea_Box_grasp * cMo_qrcode.inverse() ;

                    //          cdMc = cMo_qrcode.inverse() * cMo_teabox * oMh_Tea_Box_grasp;
                    //          printPose("cdMc: ", cdMc);
                    //          servo_larm.setCurrentFeature(cdMc) ;


                    vpAdaptiveGain lambda(0.8, 0.05, 8);
                    servo_larm.setLambda(lambda);

                    servo_larm.set_eJe(robot.get_eJe(chain_name));
                    servo_larm.m_task.set_cVe(oVe_Arm);

                    cdMc = (cMo_qrcode.inverse() * cMo_teabox * oMh_Tea_Box_grasp).inverse();
                    printPose("cdMc: ", cdMc);
                    servo_larm.setCurrentFeature(cdMc) ;



                    vpDisplay::displayFrame(I, cMo_teabox * oMh_Tea_Box_grasp , cam, 0.025, vpColor::red, 2);

                    static bool first_time_arm_servo = true;
                    if (first_time_arm_servo) {
                        std::cout << "-- Start visual servoing of the head" << std::endl;
                        servo_arm_time_init = vpTime::measureTimeSecond();
                        first_time_arm_servo = false;
                    }

                    q_dot_larm =  - servo_larm.computeControlLaw(servo_arm_time_init);


                    vpTranslationVector t_error_grasp = cdMc.getTranslationVector();
                    vpRotationMatrix R_error_grasp;
                    cdMc.extract(R_error_grasp);
                    vpThetaUVector tu_error_grasp;
                    tu_error_grasp.buildFrom(R_error_grasp);
                    double theta_error_grasp;
                    vpColVector u_error_grasp;
                    tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
                    std::cout << "error: " << t_error_grasp << " " << vpMath::deg(theta_error_grasp) << std::endl;

                    //          vpVelocityTwistMatrix cVo(cMo_qrcode);
                    //          vpMatrix cJe = cVo * oJo;
                    // Compute the feed-forward terms
                    //          vpColVector sec_ter = 0.5 * ((servo_head.m_task_head.getTaskJacobianPseudoInverse() *  (servo_head.m_task_head.getInteractionMatrix() * cJe)) * q_dot_larm);
                    //          std::cout <<"Second Term:" <<sec_ter << std::endl;
                    //q_dot_head = q_dot_head + sec_ter;

                    vpColVector task_error = servo_larm.m_task.getError();
                    vpMatrix taskJac = servo_larm.m_task.getTaskJacobian();
                    vpMatrix taskJacPseudoInv = servo_larm.m_task.getTaskJacobianPseudoInverse();

                    //cond = taskJacPseudoInv.cond();
                    //plotter_cond->plot(0, 0, loop_iter, cond);

                    // Compute joint limit avoidance
                    //q2_dot = computeQdotLimitAvoidance(task_error, taskJac, taskJacPseudoInv, jointMin, jointMax, q, q_dot_larm, ro, ro1, q_l0_min, q_l0_max, q_l1_min, q_l1_max );

                    q2_dot  = servo_larm.m_task.secondaryTaskJointLimitAvoidance(q,q_dot_larm,jointMin,jointMax);
                    //q_dot_head = q_dot_head;

                    // Add mirroring eyes
                    q_dot_tot = q_dot_head;
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);


                    vpColVector q_dot_arm_head = q_dot_larm + q2_dot;

                    q_dot_arm_head.stack(q_dot_tot);


                    robot.setVelocity(joint_names_arm_head,q_dot_arm_head);

                    if (opt_plotter_arm) {
                        plotter_arm->plot(0, cpt_iter_servo_grasp, servo_larm.m_task.getError());
                        plotter_arm->plot(1, cpt_iter_servo_grasp, q_dot_larm);
                    }

                    if (opt_plotter_q_sec_arm)
                    {
                        plotter_q_sec_arm->plot(0,loop_iter,q2_dot);
                        plotter_q_sec_arm->plot(1,loop_iter,q_dot_larm + q2_dot);

                    }



                    if (cpt_iter_servo_grasp > 100) {
                        if (opt_record_video)
                            vpDisplay::displayText(I, vpImagePoint(10,10), "Click to continue", vpColor::red);
                        else
                            vpDisplay::displayText(I, vpImagePoint(10,10), "Cannot converge. Click to continue", vpColor::red);
                    }
                    double error_t_treshold;
                    if (opt_right_arm)
                        error_t_treshold = 0.001;
                    else
                        error_t_treshold = 0.006 ;


                    if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(3)) || (click_done && button == vpMouseButton::button1 /*&& cpt_iter_servo_grasp > 150*/) )
                    {
                        robot.stop(joint_names_arm_head);
                        state_teabox_tracker = TakeTea;
                        grasp_servo_converged = true;
                        //g.setCameraParameter(AL::kCameraAutoExpositionID, 1);
                        // g.setCameraParameter(AL::kCameraAutoWhiteBalanceID, 1);
                        //            if (click_done && button == vpMouseButton::button1 && cpt_iter_servo_grasp > 150) {
                        //              click_done = false;
                        //            }


                        if (click_done && button == vpMouseButton::button1)
                            click_done = false;
                    }


                }
                cpt_iter_servo_grasp ++;



            }
            else {
                // state grasping but one of the tracker fails
                robot.stop(joint_names_arm_head);
                //g.setCameraParameter(AL::kCameraAutoExpositionID, 1);
                //g.setCameraParameter(AL::kCameraAutoWhiteBalanceID, 1);
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


            static bool first_time = true;
            if (first_time) {
                // stop head servo
                robot.stop(jointNames_tot);
                first_time = false;
            }

            // servo head to center qrcode
            if (grasp_status >= CloseHand  && grasp_status != Finished) {
                if (status_qrcode_tracker) {

                    vpImagePoint qrcode_cog_cur;
                    qrcode_cog_cur = qrcode_tracker.getCog();

                    vpMatrix eJe_head;
                    if (opt_Reye)
                        eJe_head = robot.get_eJe("REye");
                    else
                        eJe_head = robot.get_eJe("LEye");

                   // servo_head.m_task_head.init();

                    servo_head.set_eJe(eJe_head);
                    servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );
                    //servo_head.setLambda(0.4);
                    //static vpAdaptiveGain lambda(2, 0.7, 20); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
                    // vpAdaptiveGain lambda(2.5, 1., 15);
                    static vpAdaptiveGain lambda(1.0, 0.2, 10); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
                    servo_head.setLambda(lambda);
                    servo_head.setCurrentFeature( qrcode_cog_cur );
                    if(opt_right_arm)
                        servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*6/8, I.getWidth()*5/8) );
                    else
                        servo_head.setDesiredFeature( vpImagePoint( I.getHeight()*6/8, I.getWidth()*2/8) );
                    vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::yellow, 3);


                    static bool first_time_servo = true;
                    static double servo_init;
                    if (first_time_servo) {
                        servo_init = vpTime::measureTimeSecond();
                        first_time_servo = false;
                    }
                    vpColVector q_dot_head = servo_head.computeControlLaw(servo_init);

                    // Add mirroring eyes
                    q_dot_tot = q_dot_head;
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

                    robot.setVelocity(jointNames_tot, q_dot_tot);
                }
                else {
                    robot.stop(jointNames_tot);
                }
            }

            switch(grasp_status) {
            case CloseHand: {
                vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to close the hand", vpColor::red);
                if (click_done && button == vpMouseButton::button1) {
                    // Close hand
                    std::string hand;
                    if (opt_right_arm)
                        hand = "RHand";
                    else
                        hand = "LHand";

                    robot.getProxy()->setStiffnesses(hand, 1.0f);
                    AL::ALValue angle = 0.15;
                    robot.getProxy()->setAngles(hand, angle, 0.15);

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
                if (opt_right_arm)
                    cart_delta_pos[1] = 0.12;
                else
                    cart_delta_pos[1] = -0.12;
                double delta_t = 5;

                static vpCartesianDisplacement moveCartesian;
                vpVelocityTwistMatrix V;
                if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, chain_name, V)) {
                    robot.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
                }
                else {
                    robot.stop(moveCartesian.getJointNames());
                    if (! opt_interaction)
                        grasp_status = WaitDeposeTeabox;
                    else {
                        robot.stop(jointNames_tot);
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
                if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_Arm)) {
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
                if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_Arm)) {
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
                if (moveCartesian.computeVelocity(robot, cart_delta_pos, delta_t, "LArm", oVe_Arm)) {
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
                    robot.stop(jointNames_tot);
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
            static vpColVector head_pos(jointNames_tot.size(), 0);
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
                        tts.post.say(" \\rspd=80\\ \\pau=500\\ \\emph=2\\ There you are!  \\eos=1\\ " );

                        first_time = false;
                    }

                    head_cog_cur = face_tracker->getFace().getCenter();

                    vpAdaptiveGain lambda(2.5, 1., 30); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
                    servo_head.setLambda(lambda);

                    vpMatrix eJe_head;
                    if (opt_Reye)
                        eJe_head = robot.get_eJe("REye");
                    else
                        eJe_head = robot.get_eJe("LEye");

                    servo_head.set_eJe( eJe_head );

                    servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

                    servo_head.setCurrentFeature(head_cog_cur);
                    servo_head.setDesiredFeature(head_cog_des);
                    if (opt_record_video)
                        vpDisplay::displayCross(I, head_cog_cur, 15, vpColor::blue, 3); // only desired feature
                    // vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 2);
                    else
                        vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 1);

                    q_dot_head = servo_head.computeControlLaw(servo_time_init);
                    // Add mirroring eyes
                    q_dot_tot = q_dot_head;
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
                    q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

                    robot.setVelocity(jointNames_tot, q_dot_tot);


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
                    robot.stop(jointNames_tot);
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
                //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","LEyeYaw", "LEyePitch" );
                //AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(0.0), vpMath::rad(-10.0), vpMath::rad(-14.0), vpMath::rad(0), 0.0 , 0.0, 0.0, 0.0  );
                head_pos = 0;
                head_pos[1] = vpMath::rad(-8.); // NeckPitch
                head_pos[2] = vpMath::rad(-13.); // HeadPitch
                robot.setPosition(jointNames_tot, head_pos, 0.06);
                //robot.getProxy()->setAngles(jointNames_tot, angles_head, 0.06);
                interaction_status = WaitHeadInZero;
                break;
            }

            case WaitHeadInZero: {
                vpColVector head_pos_mes = robot.getPosition(jointNames_tot);
                if (sqrt((head_pos_mes-head_pos).sumSquare()) < vpMath::rad(4)) {
                    interaction_status = HeadFollowFace;
                }
                break;
            }
            case HeadFollowFace: {
                vpDisplay::displayText(I, 10, 10, "Left click to move the arm" ,vpColor::red);
                if (click_done && button == vpMouseButton::button1) {
                    click_done = false;
                    robot.stop(jointNames_tot);
                    tts.post.say(" \\rspd=90\\ \\emph=2\\ I will give you the box!  \\eos=1\\ " );
                    interaction_status = MoveArm;
                }
                break;
            }
            case MoveArm: {
                std::vector<float> head_pose;
                AL::ALValue ShoulderYaw_limits;
                float shoulderYaw_pos ;

                head_pose = robot.getProxy()->getPosition("Head", 0, true); // Position Head w.r.t the torso
                shoulderYaw_pos = head_pose[5]; //rotation arround z
                ShoulderYaw_limits = robot.getProxy()->getLimits("LShoulderYaw");

                float offset = vpMath::rad(5);
                if (shoulderYaw_pos < (float)ShoulderYaw_limits[0][0])
                    shoulderYaw_pos = (float)ShoulderYaw_limits[0][0] + offset;
                else if(shoulderYaw_pos > (float)ShoulderYaw_limits[0][1])
                    shoulderYaw_pos = (float)ShoulderYaw_limits[0][1] - offset;

                std::vector<float> LArmShoulderElbow_pos;
                std::vector<std::string> LArmShoulderElbowYaw_name;

                if (opt_right_arm)
                {
                    robot.getProxy()->setAngles("RWristRoll", vpMath::rad(10.), 0.5);

                    LArmShoulderElbow_pos.push_back(vpMath::rad(-10.));
                    LArmShoulderElbow_pos.push_back(shoulderYaw_pos);
                    LArmShoulderElbow_pos.push_back(vpMath::rad(+40.));
                    LArmShoulderElbow_pos.push_back(vpMath::rad(+90.));
                    //LArmShoulderElbow_pos.push_back(vpMath::rad(10.));

                    LArmShoulderElbowYaw_name.push_back("RShoulderPitch");
                    LArmShoulderElbowYaw_name.push_back("RShoulderYaw");
                    LArmShoulderElbowYaw_name.push_back("RElbowYaw");
                    LArmShoulderElbowYaw_name.push_back("RElbowRoll");
                    //LArmShoulderElbowYaw_name.push_back("RWristRoll");
                }
                else
                {
                    robot.getProxy()->setAngles("LWristRoll", vpMath::rad(-10.), 0.5);
                    LArmShoulderElbow_pos.push_back(vpMath::rad(+10.));
                    LArmShoulderElbow_pos.push_back(shoulderYaw_pos);
                    LArmShoulderElbow_pos.push_back(vpMath::rad(-40.));
                    LArmShoulderElbow_pos.push_back(vpMath::rad(-90.));

                    LArmShoulderElbowYaw_name.push_back("LShoulderPitch");
                    LArmShoulderElbowYaw_name.push_back("LShoulderYaw");
                    LArmShoulderElbowYaw_name.push_back("LElbowYaw");
                    LArmShoulderElbowYaw_name.push_back("LElbowRoll");
                }

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
                        phraseToSay = " \\rspd=90\\ \\emph=2\\ Take it!  \\eos=1\\ " ;
                    tts.post.say(phraseToSay);
                    double angle = 1.0f;
                    if (opt_right_arm)
                        robot.getProxy()->setAngles("RHand", angle, 1.);
                    else
                        robot.getProxy()->setAngles("LHand", angle, 1.);
                    click_done = false;
                    robot.stop(jointNames_tot);
                    interaction_status = MoveArmToRestPosition;
                }
                break;
            }
            case MoveArmToRestPosition: {
                vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to move arm to rest", vpColor::red);
                if (click_done && button == vpMouseButton::button1)  {
                    moveLArmToRestPosition(robot, chain_name); // move down to rest
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
            robot.stop(jointNames_tot);
            robot.stop(jointNames_larm);
            break;
        }

        double loop_time = vpTime::measureTimeMs() - loop_time_start;
        if (opt_plotter_time)
            plotter_time->plot(0, 0, loop_iter, loop_time);

        if (opt_plotter_q )
        {

            // q normalized between (entre -1 et 1)
            for (unsigned int i=0 ; i < numArmJoints ; i++) {
                data[i] = (q[i] - Qmiddle[i]) ;
                data[i] /= (jointMax[i] - jointMin[i]) ;
                data[i]*=2 ;
            }

            data[numArmJoints] = -1.0;
            data[numArmJoints+1] = 1.0;

            unsigned int joint = 1;
            double tQmin_l0 = jointMin[joint] + ro *(jointMax[joint] - jointMin[joint]);
            double tQmax_l0 = jointMax[joint] - ro *(jointMax[joint] - jointMin[joint]);

            double tQmin_l1 =  tQmin_l0 - ro * ro1 * (jointMax[joint] - jointMin[joint]);
            double tQmax_l1 =  tQmax_l0 + ro * ro1 * (jointMax[joint] - jointMin[joint]);

            data[numArmJoints+2] = 2*(tQmin_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
            data[numArmJoints+3] = 2*(tQmax_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

            data[numArmJoints+4] =  2*(tQmin_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
            data[numArmJoints+5] =  2*(tQmax_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

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

    if (opt_interaction)
        delete face_tracker;

    if (opt_plotter_time)
        delete plotter_time;

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

