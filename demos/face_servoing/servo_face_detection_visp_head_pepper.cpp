/**
 *
 * This example demonstrates how to get images from the robot remotely, how
 * to track a face using all the four joints of the Romeo Head;
 *
 */

/*! \example servo_face_detection_visp_head.cpp */

#include <iostream>
#include <string>

#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

#include <alproxies/altexttospeechproxy.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
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

#include <visp/vpPlot.h>

/*!

  Connect to Pepper robot, grab, display images using ViSP and start
  face detection with OpenCV and tracking with ViSP when the detection fails.
  More over all the four joints of Romeo's head are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 198.18.0.1.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head --ip <robot ip address> --haar <haar cascade .xml file>

  Example:

  ./servo_face_detection_visp_head --ip 169.254.168.230 --haar ./haarcascade_frontalface_alt.xml
 */






int main(int argc, const char* argv[])
{
  std::string opt_ip = "131.254.10.126";
  std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--ip <robot address>] [--haar <haarcascade xml filename>] [--help]" << std::endl;
      return 0;
    }
  }



  // USE NEW MODULE

  qi::SessionPtr session = qi::makeSession();
  session->connect("tcp://131.254.10.126:9559");
  qi::AnyObject proxy = session->service("pepper_control");

  proxy.call<void >("start");

  //  qi::ApplicationSession app(argc, argv);
  //  app.start();

  //  qi::SessionPtr session = app.session();
  //  qi::AnyObject tts = session->service("Control");


  // -------------------------------------


  std::string camera_name = "CameraTopPepper";

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(30);
  g.setCamera(0);
  g.open();
  vpCameraParameters cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA,camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  vpHomogeneousMatrix eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name,vpCameraParameters::perspectiveProjWithDistortion);


  std::cout << "eMc:" << std::endl << eMc << std::endl;


  // Connect to the robot
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();

  if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  {
    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
    return 0;
  }

  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");



  // Open Proxy for the speech
  AL::ALTextToSpeechProxy tts(opt_ip, 9559);
  tts.setLanguage("English");
  const std::string phraseToSay = "Hello there!";


  // Plotting

  vpPlot plotter_diff_vel (2);
  plotter_diff_vel.initGraph(0, 2);
  plotter_diff_vel.initGraph(1, 2);
  plotter_diff_vel.setTitle(0,  "HeadYaw");
  plotter_diff_vel.setTitle(1,  "HeadPitch");


  vpPlot plotter_error (2);
  plotter_error.initGraph(0, 1);
  plotter_error.initGraph(1, 1);

  plotter_error.setTitle(0,  "HeadYaw");
  plotter_error.setTitle(1,  "HeadPitch");



  try {

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTracker face_tracker;
    face_tracker.setFaceCascade(opt_face_cascade_name);

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(4, 0.5, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    servo_head.setLambda(lambda);

    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot_head;

    bool reinit_servo = true;
    bool speech = true;
    unsigned long loop_iter = 0;



    std::vector<float> q(jointNames_head.size());
    std::vector<float> q_new(jointNames_head.size());
    double delta_t = 0.0;
    double t_prev = vpTime::measureTimeSecond();

    while(1) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        t_prev = vpTime::measureTimeSecond();
        reinit_servo = false;
        //proxy.call<void >("start");

      }

      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      bool face_found = face_tracker.track(I);

      if (face_found) {
        vpDisplay::displayRectangle(I, face_tracker.getFace(), vpColor::red, false, 4);
        head_cog_cur = face_tracker.getFace().getCenter();

        servo_head.set_eJe( robot.get_eJe("Head") );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        vpDisplay::setFont(I, "-*-*-bold-*-*-*-*-*-*-*-*-*-*-*");
        //vpDisplay::displayText(I, face_tracker.getFace().getTopLeft()+vpImagePoint(-20,0), "Coraline", vpColor::red);
        //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        q_dot_head = servo_head.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        // robot.setVelocity(jointNames_head, q_dot_head,true);

        // --------------------------

        //        robot.getPosition(jointNames_head, q, true);

        //        double now_t = vpTime::measureTimeSecond();
        //        delta_t = now_t - t_prev;

        //        for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
        //         q_new[i] = q[i] + q_dot_head[i]*delta_t;

        //        }

        //        std::cout << "Pose:" << q_new << std::endl;
        //        std::cout << "Delta:" << delta_t << std::endl;

        //        robot.setPosition(jointNames_head, q_new, 1.0);

        //        t_prev = now_t;


        // -------------------------------------------

        // USE NEW MODULE
        std::vector<float> vel(jointNames_head.size());
        for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
          vel[i] = q_dot_head[i];

        }

        proxy.async<void >("setDesJointVelocity", jointNames_head,vel );


        //  ______________________________________________

        std::cout << "q dot: " << q_dot_head.t() << std::endl;

        vpColVector vel_head = robot.getJointVelocity(jointNames_head);
        for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
          plotter_diff_vel.plot(i,1,loop_iter,q_dot_head[i]);
          plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);
          plotter_error.plot(i,0,loop_iter,servo_head.m_task_head.getError()[i]);
        }

        std::cout <<"getVel: " << vel_head << std::endl;
        //      std::cout <<"__________________________: " << result << std::endl;

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);

        if (distance < 0.03*I.getWidth() && speech) { // 3 % of the image witdh
          // Call the say method
          static bool firstTime = true;
          if (firstTime) {
            //tts.post.say(phraseToSay);
            firstTime = false;
          }
          speech = false;

        }
        else if (distance > 0.20*I.getWidth()) // 20 % of the image witdh
          speech = true;

      }
      else {
        //        robot.stop(jointNames_head);
       proxy.call<void >("stopJoint");
       std::cout << "Stop!" << std::endl;
       reinit_servo = true;
      }


      //      for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
      //        plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);
      //        // plotter_diff_vel.plot(i,1,loop_iter,q_dot_head[i]);
      //      }


      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      loop_iter ++;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }

    vpDisplay::getClick(I, true);

    proxy.call<void >("stop");


  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames_head);




  return 0;
}





