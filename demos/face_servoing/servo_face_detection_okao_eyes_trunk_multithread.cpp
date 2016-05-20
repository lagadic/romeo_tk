/*! \example servo_face_detection_visp_head.cpp */

#include <iostream>
#include <string>

#include <alproxies/altexttospeechproxy.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vpServoHead.h>
#include <vpFaceTrackerOkao.h>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpServo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/sensor/vpV4l2Grabber.h>


/*!

  Connect to Romeo robot, grab, display images using ViSP and start
  face detection with Okao and tracking with ViSP when the detection fails.
  More over all the four joints of Romeo's head plus the two joint of the Left eye plus the trunk are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 198.18.0.1.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head --ip <robot ip address>

  Example:

  ./servo_face_detection_visp_head --ip 169.254.168.230
 */

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;

bool s_face_available = false;
bool opt_Reye = false;
bool opt_language_english = true;
double s_distance;
std::string opt_ip = "198.18.0.1";
vpImagePoint s_head_cog_des;
vpImagePoint s_head_cog_cur;
vpRect s_face_bbox;
#if defined(VISP_HAVE_V4L2)
vpImage<unsigned char> s_frame;
#elif defined(VISP_HAVE_OPENCV)
cv::Mat s_frame;
#endif
double s_face_size;
double s_distance_servo;

vpMutex s_mutex_capture;
vpMutex s_mutex_frame;
vpMutex s_mutex_face;
vpMutex s_mutex_cogs;
vpMutex s_mutex_distance;
vpMutex s_mutex_face_size;

bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}

bool pred(const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs)
{
  return lhs.second < rhs.second;
}

vpThread::Return captureFunction(vpThread::Args args)
{
  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(15);
  g.setCamera(2);
  g.open();

  vpImage<unsigned char> frame_;
  bool stop_capture_ = false;

  //Get a first frame
  g.acquire(frame_);
  {
    vpMutex::vpScopedLock lock(s_mutex_cogs);
    s_head_cog_des.set_uv(frame_.getWidth()/2, frame_.getHeight()/2);
    s_distance_servo = 0.03*frame_.getWidth();
  }

  while (!stop_capture_) {
    // Capture in progress
    g.acquire(frame_);

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;
      s_frame = frame_;
    }
  }

  {
    vpMutex::vpScopedLock lock(s_mutex_capture);
    s_capture_state = capture_stopped;
  }

  std::cout << "End of capture thread" << std::endl;
  return 0;
}

vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args

  t_CaptureState capture_state_;
  bool face_available_ = false;
  vpImage<unsigned char> frame_;
  bool display_initialized_ = false;
  double distance;
  vpImagePoint head_cog_des_;

#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *d_ = NULL;
#endif

  vpFaceTrackerOkao face_tracker(opt_ip,9559);

  std::vector<std::string> recognized_names;
  std::map<std::string,unsigned int> detected_face_map;
  bool detection_phase = true;
  double face_size = 0;
  unsigned int f_count = 0;


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

  do {
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      capture_state_ = s_capture_state;
    }

    // Check if a new frame is available
    if (capture_state_ == capture_started) {
      // Get the frame
      {
        vpMutex::vpScopedLock lock(s_mutex_frame);
        frame_ = s_frame;
      }

      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        d_ = new vpDisplayX(frame_);
        display_initialized_ = true;
#elif defined(VISP_HAVE_GDI)
        d_ = new vpDisplayGDI(frame_);
        display_initialized_ = true;
#endif
      }

      // Display the image
      vpDisplay::display(frame_);
      vpDisplay::setTitle(frame_, "ViSP viewer");

      // Check if a face was detected
      bool face_found_ = face_tracker.detect();

      // Get the center of the frame detected
      {
        vpMutex::vpScopedLock lock(s_mutex_cogs);
        head_cog_des_ = s_head_cog_des;
      }
      //Check if a face was detected
      if (face_found_) {
        {
          vpMutex::vpScopedLock lock(s_mutex_face);
          s_face_available = true;
        }

        std::ostringstream text;
        text << "Found " << face_tracker.getNbObjects() << " face(s)";
        vpDisplay::displayText(frame_, 10, 10, text.str(), vpColor::red);

        for(size_t i=0; i < face_tracker.getNbObjects(); i++) {
          vpDisplay::displayCross(frame_, face_tracker.getCog(i), 10, vpColor::red, 2);
        }

        vpDisplay::displayCross(frame_, head_cog_des_, 10, vpColor::green, 2);

        vpRect bbox = face_tracker.getBBox(0);
        std::string name = face_tracker.getMessage(0);

        {
          vpMutex::vpScopedLock lock(s_mutex_face_size);
          s_face_size = bbox.getSize();
        }

        //Get the distance between the target and the center
        {
          vpMutex::vpScopedLock lock(s_mutex_distance);
          distance = s_distance;
        }

        if (distance < 0.06*frame_.getWidth() && bbox.getSize() > 3000)
        {
          vpDisplay::displayRectangle(frame_, bbox, vpColor::red, false, 1);
          vpDisplay::displayText(frame_, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::red);
        }
        else
        {
          vpDisplay::displayRectangle(frame_, bbox, vpColor::green, false, 1);
          vpDisplay::displayText(frame_, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::green);
        }

        double u = face_tracker.getCog(0).get_u();
        double v = face_tracker.getCog(0).get_v();
        if (u<= frame_.getWidth() && v <= frame_.getHeight()){
          {
            vpMutex::vpScopedLock lock(s_mutex_face);
            s_head_cog_cur.set_uv(u,v);
          }
        }

        if (detection_phase)
        {
          if (distance < 0.06*frame_.getWidth() && bbox.getSize() > 3000)
          {
            detected_face_map[name]++;
            f_count++;
          }
          if (f_count>10)
          {
            detection_phase = false;
            f_count = 0;
          }
        }
        else
        {
          std::string recognized_person_name = std::max_element(detected_face_map.begin(), detected_face_map.end(), pred)->first;
          unsigned int times = std::max_element(detected_face_map.begin(), detected_face_map.end(), pred)->second;

          if (!in_array(recognized_person_name, recognized_names) && recognized_person_name != "Unknown") {

            if (opt_language_english)
            {
              phraseToSay = "\\emph=2\\ Hi \\wait=200\\ \\emph=2\\" + recognized_person_name + "\\pau=200\\ How are you ?";
            }
            else
            {
              phraseToSay = "\\emph=2\\ Salut \\wait=200\\ \\emph=2\\" + recognized_person_name + "\\pau=200\\ comment vas  tu ?";;
            }

            std::cout << phraseToSay << std::endl;
            tts.post.say(phraseToSay);
            recognized_names.push_back(recognized_person_name);
          }
          if (!in_array(recognized_person_name, recognized_names) && recognized_person_name == "Unknown"
              && times > 15)
          {

            if (opt_language_english)
            {
              phraseToSay = "\\emph=2\\ Hi \\wait=200\\ \\emph=2\\. I don't know you! \\emph=2\\ What's your name?";
            }
            else
            {
              phraseToSay = " \\emph=2\\ Salut \\wait=200\\ \\emph=2\\. Je ne te connais pas! \\emph=2\\  Comment t'appelles-tu ?";
            }

            std::cout << phraseToSay << std::endl;
            tts.post.say(phraseToSay);
            recognized_names.push_back(recognized_person_name);
          }

          detection_phase = true;
          detected_face_map.clear();

        }

        face_available_ = false;

      }
      else{
        {
          vpMutex::vpScopedLock lock(s_mutex_face);
          s_face_available = false;
        }
      }

      // Trigger end of acquisition with a mouse click
      vpDisplay::displayText(frame_, 235, 10, "Click to exit...", vpColor::red);
      if (vpDisplay::getClick(frame_, false)) {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        s_capture_state = capture_stopped;
      }

      // Update the display
      vpDisplay::flush(frame_);

    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}

vpThread::Return servoHeadFunction(vpThread::Args args)
{
  //Initialise the robot
  vpNaoqiRobot robot;

  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();

  vpImage<unsigned char> frame_;

  //Get the intrinsic and extrinsic parameters of the camera used
  vpHomogeneousMatrix eMc;
  vpCameraParameters cam;
  if (opt_Reye)
  {
    eMc = vpNaoqiGrabber::getExtrinsicCameraParameters("CameraRightEye", vpCameraParameters::perspectiveProjWithDistortion);
    cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA,"CameraRightEye", vpCameraParameters::perspectiveProjWithDistortion);
  }
  else
  {
    eMc = vpNaoqiGrabber::getExtrinsicCameraParameters("CameraLeftEye",vpCameraParameters::perspectiveProjWithDistortion);
    cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA,"CameraLeftEye",vpCameraParameters::perspectiveProjWithDistortion);
  }

  std::cout << "Extrinsic camera param: " <<  std::endl << eMc << std::endl;
  std::cout << "Intrinsic camera param: " <<  std::endl << cam << std::endl;

  std::vector<std::string> jointNames;
  jointNames.push_back("TrunkYaw");
  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");
  jointNames.insert(jointNames.end(),jointNames_head.begin(),jointNames_head.end());

  jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesREye.begin(), jointNamesREye.end());
  std::vector<std::string> jointNames_tot = jointNames;

  jointNames_tot.push_back(jointNamesLEye.at(0));
  jointNames_tot.push_back(jointNamesLEye.at(1));

  std::cout <<" Joint Names: " << jointNames_tot << std::endl;

  vpMatrix MAP_head(7,6);
  for (unsigned int i = 0; i < 4 ; i++)
    MAP_head[i][i]= 1;
  MAP_head[5][4]= 1;
  MAP_head[6][5]= 1;


  // Initialize the joint avoidance scheme from the joint limits
  vpColVector jointMin(jointNames.size());// = robot.getJointMin(jointNames);
  vpColVector jointMax(jointNames.size());//= robot.getJointMax(jointNames);

  robot.getJointMinAndMax(jointNames, jointMin,jointMax);
  jointMin[0]=vpMath::rad(-16.8);
  jointMin[jointNames.size()-2]=vpMath::rad(-16.8);
  jointMin[jointNames.size()-1]=vpMath::rad(-14.8);

  jointMax[jointNames.size()-2]= vpMath::rad(17.2);
  jointMax[jointNames.size()-1]= vpMath::rad(15.3);

  std::cout << "limit max:" << jointMax << std::endl;
  std::cout << "limit min:" << jointMin << std::endl;

  vpColVector qmoy(jointNames.size());
  for (unsigned int i = 0; i<qmoy.size(); i++)
  {
    qmoy[i] = jointMax[i] -jointMin[i];
  }

  // Vector secondary task
  vpColVector q2 (jointNames.size());

  vpColVector head_pos(jointNames_tot.size());
  head_pos = 0;
  head_pos[2] = vpMath::rad(-10.); // NeckPitch
  head_pos[3] = vpMath::rad(-6.); // HeadPitch
  robot.setPosition(jointNames_tot, head_pos, 0.3);

  vpTime::sleepMs(1000);


  try {

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(2, 1.8,  35); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    //    vpAdaptiveGain lambda(2, 1.8,  35); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    //    vpAdaptiveGain lambda(3, 1., 30);
    servo_head.setLambda(lambda);

    double servo_time_init = 0;
    bool face_available_ = false;
    t_CaptureState capture_state_;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des;

    //We have to wait for the grabber to capture the first frame
    while(1)
    {
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        capture_state_ = s_capture_state;
      }
      // Check if a frame is available
      if (capture_state_ == capture_started)
      {
        {
          vpMutex::vpScopedLock lock(s_mutex_cogs);
          head_cog_des = s_head_cog_des;
        }
        break;
      }
      vpTime::sleepMs(5);
    }

    //Get a frame
    {
      vpMutex::vpScopedLock lock(s_mutex_frame);
      frame_ = s_frame;
    }

    vpColVector q_dot_head;
    vpColVector q_dot_tot;
    vpColVector q;

    bool reinit_servo = true;
    bool speech = true;

    vpColVector sum_dedt(2);

    // Constant integral gain used when target tracking: [default 0.01]
    double opt_mu = 0.01;

    while(capture_state_ != capture_stopped) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }

      double t = vpTime::measureTimeMs();
      //Get actual position joint of the robot
      q = robot.getPosition(jointNames);
      q[0] = -q[0];

      // Check if a face was detected
      {
        vpMutex::vpScopedLock lock(s_mutex_face);
        face_available_ = s_face_available;
      }

      if (face_available_) {

        if (opt_Reye)
          servo_head.set_eJe( robot.get_eJe("REye_t") * MAP_head );
        else
          servo_head.set_eJe( robot.get_eJe("LEye_t") * MAP_head );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );
        //Get the cog of the face detected
        {
          vpMutex::vpScopedLock lock(s_mutex_face);
          head_cog_cur = s_head_cog_cur;
        }
        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);

        q_dot_head = servo_head.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        ///Integral term///
        sum_dedt += servo_head.m_task_head.error;
        q_dot_head -= opt_mu*servo_head.m_task_head.getTaskJacobianPseudoInverse()*sum_dedt;
        ///Integral term///

        vpMatrix P = servo_head.m_task_head.getI_WpW();
        //vpMatrix P = servo_head.m_task_head.getLargeP();
        double alpha = -0.08;

        vpColVector z_q2 (q_dot_head.size());
        //z_q2[1] = 2 * alpha * (q[1] - qmoy[1])/(jointMax[1]- jointMin[1]);
        //z_q2[4] = 2 * alpha * (q[4] - qmoy[4])/(jointMax[4]- jointMin[4]);
        z_q2[1] = 2 * alpha * q[1]/ pow((jointMax[1]- jointMin[1]),2);
        z_q2[4] = 2 * alpha * q[4]/pow((jointMax[4]- jointMin[4]),2);
        z_q2[5] = 2 * alpha * q[5]/pow((jointMax[5]- jointMin[5]),2);

        vpColVector q3 = P * z_q2;
        //std::cout << "q3: " << q3 << std::endl;
        if (q3.euclideanNorm()<10.0)
          q_dot_head =  q_dot_head + q3;


        // Add mirroring eyes
        q_dot_tot = q_dot_head;
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

        robot.setVelocity(jointNames_tot, q_dot_tot);

        //std::cout << "q dot: " << q_dot_head.t() << std::endl;

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);

        {
          vpMutex::vpScopedLock lock(s_mutex_distance);
          s_distance = distance;
        }

        //        if (distance < 0.03*frame_.getWidth() && speech) { // 3 % of the image witdh
        //          // Call the say method
        //          static bool firstTime = true;
        //          if (firstTime) {
        //            tts.post.say(phraseToSay);
        //            firstTime = false;
        //          }
        //          speech = false;

        //        }
        //        else if (distance > 0.20*frame_.getWidth()) // 20 % of the image witdh
        //          speech = true;

        // Compute the distance in pixel between the target and the center of the image
        if (distance > s_distance_servo)
          robot.setVelocity(jointNames_tot, q_dot_tot);

        std::cout << "q dot: " << q_dot_head.t() << std::endl;

      }
      else {
        robot.stop(jointNames_tot);
        reinit_servo = true;
      }

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        capture_state_ = s_capture_state;
      }
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
  std::cout << "End of servoing the head thread" << std::endl;

  return 0;
}

int main(int argc, const char* argv[])
{
  std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--Reye")
      opt_Reye = true;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--ip <robot address>] [--haar <haarcascade xml filename>] [--english] [--Reye] [--help]" << std::endl;
      return 0;
    }
  }


  // Start the threads
  vpThread thread_capture(captureFunction);
  vpThread thread_display(displayFunction);
  vpThread thread_servo(servoHeadFunction);

  // Wait until thread ends up
  thread_display.join();
  thread_capture.join();
  thread_servo.join();

  return 0;
}
