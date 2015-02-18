/*! \example servo_face_detection_visp_head.cpp */

#include <iostream>
#include <string>

#include <alproxies/altexttospeechproxy.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerWarpSRT.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpFaceTracker.h>
#include <vpServoHead.h>


/*!

  Connect to Romeo robot, grab, display images using ViSP and start
  face detection with OpenCV and tracking with ViSP when the detection fails.
  More over all the four joints of Romeo's head plus the two joint of the Left eye are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 198.18.0.1.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head --ip <robot ip address> --haar <haar cascade .xml file>

  Example:

  ./servo_face_detection_visp_head --ip 169.254.168.230 --haar ./haarcascade_frontalface_alt.xml
 */
int main(int argc, const char* argv[])
{
  std::string opt_ip = "198.18.0.1";
  std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
  bool opt_language_english = false;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--english")
      opt_language_english = true;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--ip <robot address>] [--haar <haarcascade xml filename>] [--english] [--help]" << std::endl;
      return 0;
    }
  }

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(15);
  g.setCamera(2); // CameraLeftEye
  g.open();
  vpCameraParameters cam = g.getCameraParameters();
  vpHomogeneousMatrix eMc = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");


  // Connect to the robot
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();


  std::vector<std::string> jointNames = robot.getBodyNames("Head");
  jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  std::vector<std::string> jointNames_tot = jointNames;
  jointNames_tot.push_back(jointNamesREye.at(0));
  jointNames_tot.push_back(jointNamesREye.at(1));


  vpMatrix MAP_head(6,5);
  for (unsigned int i = 0; i < 3 ; i++)
    MAP_head[i][i]= 1;
  MAP_head[4][3]= 1;
  MAP_head[5][4]= 1;


  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");

  vpColVector head_pos(jointNames_head.size());
  head_pos = 0;
  head_pos[1] = vpMath::rad(-10.); // NeckPitch
  head_pos[2] = vpMath::rad(0.); // HeadPitch
  robot.setPosition(jointNames_head, head_pos, 0.3);

  vpTime::sleepMs(1000);

  // Open Proxy for the speech
  AL::ALTextToSpeechProxy tts(opt_ip, 9559);
  std::string phraseToSay;
  if (opt_language_english)
  {
    tts.setLanguage("English");
    phraseToSay = " \\emph=2\\ Hi,\\pau=200\\ How are you ?";
  }
  else
  {
    tts.setLanguage("French");
    phraseToSay = " \\emph=2\\ Bonjour,\\pau=200\\ comment vas  tu ?";
  }
  try {

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTracker face_tracker;
    face_tracker.setFaceCascade(opt_face_cascade_name);

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(2, 1.5, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    //vpAdaptiveGain lambda(3, 1., 30);
    servo_head.setLambda(lambda);

    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot_head;
    vpColVector q_dot_tot;

    bool reinit_servo = true;
    bool speech = true;

    while(1) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }

      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      bool face_found = face_tracker.track(I);

      if (face_found) {
        vpDisplay::displayRectangle(I, face_tracker.getFace(), vpColor::red, false, 4);
        head_cog_cur = face_tracker.getFace().getCenter();

        servo_head.set_eJe( robot.get_eJe("LEye") * MAP_head );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        q_dot_head = servo_head.computeControlLaw(servo_time_init);

       // Add mirroring eyes
        q_dot_tot = q_dot_head;
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

        robot.setVelocity(jointNames_tot, q_dot_tot);

        //std::cout << "q dot: " << q_dot_head.t() << std::endl;

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);

        if (distance < 0.03*I.getWidth() && speech) { // 3 % of the image witdh
          // Call the say method
          static bool firstTime = true;
          if (firstTime) {
            tts.post.say(phraseToSay);
            firstTime = false;
          }
          speech = false;

        }
        else if (distance > 0.20*I.getWidth()) // 20 % of the image witdh
          speech = true;

      }
      else {
        robot.stop(jointNames_tot);
        reinit_servo = true;
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames_tot);


  return 0;
}

