/**
 *
 * This example demonstrates how to get images from the robot remotely, how
 * to track a face using all the four joints of the Romeo Head;
 *
 */

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

#include <visp/vpPlot.h>

/*!

  Connect to Romeo robot, grab, display images using ViSP and start
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
  std::string opt_ip = "198.18.0.1";
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

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(15);
  g.setCamera(0);
  g.open();
  vpCameraParameters cam = g.getCameraParameters();
  vpHomogeneousMatrix eMc = g.get_eMc();


  // Connect to the robot
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();
  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");

  // Open Proxy for the speech
  AL::ALTextToSpeechProxy tts(opt_ip, 9559);
  tts.setLanguage("French");
  const std::string phraseToSay = "Bonjour, bienvenue dans le laboratoire de recherche.";


  // Plotting

  // Head
  std::vector <std::string > names_head = robot.getBodyNames("Head");

  vpPlot plotter_diff_vel (4);
  plotter_diff_vel.initGraph(0, 2);
  plotter_diff_vel.initGraph(1, 2);
  plotter_diff_vel.initGraph(2, 2);
  plotter_diff_vel.initGraph(3, 2);
  plotter_diff_vel.setTitle(0,  "NeckYaw");
  plotter_diff_vel.setTitle(1,  "NeckPitch");
  plotter_diff_vel.setTitle(2,  "HeadPitch");
  plotter_diff_vel.setTitle(3,  "HeadRoll");

  vpPlot plotter_error (4);
  plotter_error.initGraph(0, 1);
  plotter_error.initGraph(1, 1);
  plotter_error.initGraph(2, 1);
  plotter_error.initGraph(3, 1);
  plotter_error.setTitle(0,  "NeckYaw");
  plotter_error.setTitle(1,  "NeckPitch");
  plotter_error.setTitle(2,  "HeadPitch");
  plotter_error.setTitle(3,  "HeadRoll");


  try {

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTracker face_tracker;
    face_tracker.setFaceCascade(opt_face_cascade_name);

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(2, 0.8, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    servo_head.setLambda(lambda);

    bool teabox_servo_converged = false;
    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot_head;

    bool reinit_servo = true;
    bool speech = true;
    unsigned long loop_iter = 0;

    while(1) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }

      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      bool face_found = face_tracker.track(I);

        vpColVector vel_head = robot.getJointVelocity(names_head);

      if (face_found) {
        vpDisplay::displayRectangle(I, face_tracker.getFace(), vpColor::red, false, 4);
        head_cog_cur = face_tracker.getFace().getCenter();

        servo_head.set_eJe( robot.get_eJe("Head") );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        vpDisplay::setFont(I, "-*-*-bold-*-*-*-*-*-*-*-*-*-*-*");
        vpDisplay::displayText(I, face_tracker.getFace().getTopLeft()+vpImagePoint(-20,0), "Coraline", vpColor::red);
        //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        q_dot_head = servo_head.computeControlLaw(servo_time_init);
        robot.setVelocity(jointNames_head, q_dot_head);
        std::cout << "q dot: " << q_dot_head.t() << std::endl;

        //vpColVector vel_head = robot.getJointVelocity(names_head);
        for (unsigned int i=0 ; i < names_head.size() ; i++) {
          plotter_diff_vel.plot(i,1,loop_iter,q_dot_head[i]);
          plotter_error.plot(i,0,loop_iter,vel_head[i] - q_dot_head[i]);
        }

        //      std::cout <<"getVel: " << vel << std::endl;
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
        robot.stop(jointNames_head);
        reinit_servo = true;
      }


      for (unsigned int i=0 ; i < names_head.size() ; i++) {
        plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);
        // plotter_diff_vel.plot(i,1,loop_iter,q_dot_head[i]);
      }


      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      loop_iter ++;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }

    vpDisplay::getClick(I, true);

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





#if 0
/** Initialization settings face detection*/
vpTemplateTrackerWarpSRT warp;
vpTemplateTrackerSSDInverseCompositional tracker(&warp);
tracker.setSampling(2,2);
tracker.setLambda(0.001);
tracker.setIterationMax(5);
tracker.setPyramidal(2, 1);

std::vector<cv::Rect> faces;
size_t larger_face_index = 0;

state_t state = detection;
vpTemplateTrackerZone m_zone_ref, zone_cur;
double m_area_m_zone_ref, m_area_zone_cur, area_zone_prev;
vpColVector p; // Estimated parameters

/** Initialization Visp Image, display and camera paramenters*/
vpImage<unsigned char> I(g.getHeight(), g.getWidth());
vpDisplayX d(I);
vpDisplay::setTitle(I, "ViSP viewer");
vpCameraParameters cam = g.getCameraParameters();

cv::Mat frame_gray;

vpRect target;
int iter = 0;

/** Initialization Visual servoing task*/
vpServo task; // Visual servoi    vpServo task; // Visual servoing task
vpFeaturePoint sd; //The desired point feature.
//Set the desired features x and y
double xd = 0;
double yd = 0;
//Set the depth of the point in the camera frame.
double Zd = 1.8;
//Set the point feature thanks to the desired parameters.
sd.buildFrom(xd, yd, Zd);
vpFeaturePoint s; //The current point feature.
//Set the current features x and y
double x = xd; //You have to compute the value of x.
double y = yd; //You have to compute the value of y.
double Z = Zd; //You have to compute the value of Z.
//Set the point feature thanks to the current parameters.
s.buildFrom(x, y, Z);
//In this case the parameter Z is not necessary because the interaction matrix is computed
//with the desired visual feature.
// Set eye-in-hand control law.
// The computed velocities will be expressed in the camera frame
task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
// Interaction matrix is computed with the desired visual features sd
task.setInteractionMatrixType(vpServo::DESIRED);
// Add the 2D point feature to the task
task.addFeature(s, sd);

vpAdaptiveGain lambda(2, 0.8, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
task.setLambda(lambda);
//task.setLambda(0.8);

vpColVector q_dot;

// Transformation HeadRoll to Camera Left
vpHomogeneousMatrix eMc = g.get_eMc();

std::vector<std::string> jointNames =  robot.getBodyNames("Head");
const unsigned int numJoints = jointNames.size();

// Declate Jacobian
vpMatrix eJe(6,numJoints);

robot.setStiffness(jointNames, 1.f);


double tinit = 0; // initial time in second

try
{
while(1) {
double t = vpTime::measureTimeMs();
g.acquire(I);
vpDisplay::display(I);

vpImageConvert::convert(I, frame_gray);

std::cout << "state: " << state << std::endl;
//-- Detect faces
bool m_target_found = false;
if (1) {//state == detection) {
faces.clear();
face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
std::cout << "Detect " << faces.size() << " faces" << std::endl;
if (faces.size()) {

  state = init_tracking;

  // Display all the detected faces
  int face_max_area = 0;
  for( size_t i = 0; i < faces.size(); i++ ) {
    if (faces[i].area() > face_max_area) {
      face_max_area = faces[i].area();
      larger_face_index = i;
    }
  }
  m_target_found = true;
  // Display the larger face
  size_t i=larger_face_index;
  target.set(faces[i].tl().x, faces[i].tl().y, faces[i].size().width, faces[i].size().height);
  //                vpDisplay::displayRectangle(I, target, vpColor::green, false, 4);
}
}

//-- Track the face
if (state == init_tracking) {
  vpDisplay::displayCharString(I, 10,10, "state: detection", vpColor::red);
  size_t i=larger_face_index;
  double scale = 0.05; // reduction factor
  int x = faces[i].tl().x;
  int y = faces[i].tl().y;
  int width  = faces[i].size().width;
  int height = faces[i].size().height;
  std::vector<vpImagePoint> corners;
  corners.push_back( vpImagePoint(y+scale*height    , x+scale*width) );
  corners.push_back( vpImagePoint(y+scale*height    , x+(1-scale)*width) );
  corners.push_back( vpImagePoint(y+(1-scale)*height, x+(1-scale)*width) );
  corners.push_back( vpImagePoint(y+(1-scale)*height, x+scale*width) );
  try {
    tracker.resetTracker();
    tracker.initFromPoints(I, corners, true);
    tracker.track(I);
    //tracker.display(I, vpColor::green);
    m_zone_ref = tracker.getZoneRef();
    m_area_m_zone_ref = m_zone_ref.getArea();
    p = tracker.getp();
    warp.warpZone(m_zone_ref, p, zone_cur);
    area_zone_prev = m_area_zone_cur = zone_cur.getArea();
    state = tracking;
  }
  catch(...) {
    std::cout << "Exception init tracking" << std::endl;
    state = detection;
  }
}
else if (state == tracking) {
  try {
    vpDisplay::displayCharString(I, 10,10, "state: tracking", vpColor::red);
    tracker.track(I);

    //tracker.display(I, vpColor::blue);
    {
      // Instantiate and get the reference zone
      p = tracker.getp();
      warp.warpZone(m_zone_ref, p, zone_cur);
      m_area_zone_cur = zone_cur.getArea();

      // std::cout << "Area ref: " << m_area_m_zone_ref << std::endl;
      std::cout << "Area tracked: " << m_area_zone_cur << std::endl;

      double size_percent = 0.6;
      //if (m_area_zone_cur/m_area_m_zone_ref < size_percent || m_area_zone_cur/m_area_m_zone_ref > (1+size_percent)) {
      if (m_area_zone_cur/area_zone_prev < size_percent || m_area_zone_cur/area_zone_prev > (1+size_percent)) {
        std::cout << "reinit caused by size" << std::endl;
        state = detection;
      }
      else {
        target = zone_cur.getBoundingBox();
        m_target_found = true;
      }

      area_zone_prev = m_area_zone_cur;
    }
  }
  catch(...) {
    std::cout << "Exception tracking" << std::endl;
    state = detection;
  }
}

if (m_target_found) {
  vpDisplay::displayRectangle(I, target, vpColor::red, false, 4);

  vpImagePoint cog = target.getCenter();
  vpDisplay::displayCross(I, cog, 12, vpColor::red, 3);
  double x=0, y=0;
  vpPixelMeterConversion::convertPoint(cam, cog, x, y);
  s.buildFrom(x, y, Z);

  eJe = robot.get_eJe("Head");
  task.set_eJe(eJe);
  task.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

  q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - tinit);
  task.print();

  vpImagePoint cog_desired;
  vpMeterPixelConversion::convertPoint(cam, sd.get_x(), sd.get_y(), cog_desired);
  vpDisplay::displayCross(I, cog_desired, 10, vpColor::green, 2);
  std::cout << "q dot: " << q_dot.t() << " in deg/s: "
            << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;
  robot.setVelocity(jointNames, q_dot);

  // Compute the distance in pixel between the target and the center of the image
  double distance = vpImagePoint::distance(cog_desired, cog);

  if (distance < 0.03*I.getWidth() && speech) // 3 % of the image witdh
  {
    /** Call the say method */
    tts.post.say(phraseToSay);
    speech = false;

  }
  else if (distance > 0.20*I.getWidth()) // 20 % of the image witdh
    speech = true;
}
else {
std::cout << "Stop the robot..." << std::endl;
robot.stop(jointNames);

}
vpDisplay::flush(I);
if (vpDisplay::getClick(I, false))
break;
std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

}
}
catch (const AL::ALError& e)
{
  std::cerr << "Caught exception " << e.what() << std::endl;
}

std::cout << "The end: stop the robot..." << std::endl;
robot.stop(jointNames);
task.kill();

return 0;
}
#endif
