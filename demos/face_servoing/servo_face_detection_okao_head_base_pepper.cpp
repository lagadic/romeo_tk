#include <iostream>
#include <string>
#include <map>


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
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpServoHead.h>
#include <vpFaceTrackerOkao.h>


/*!

  Connect to Pepper robot, grab, display images using ViSP and start
  face detection with Okao .
  More over all the two joints of Pepper's head are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 131.254.10.126.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head --ip <robot ip address>

  Example:

  ./servo_face_detection_visp_head --ip 169.254.168.230
 */



bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}

bool pred(const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs)
{
  return lhs.second < rhs.second;
}


int main(int argc, const char* argv[])
{
  std::string opt_ip = "131.254.10.126";
  bool opt_language_english = true;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--fr]" << std::endl;
      return 0;
    }
  }

  // USE NEW MODULE

  qi::SessionPtr session = qi::makeSession();
  session->connect("tcp://131.254.10.126:9559");
  qi::AnyObject proxy = session->service("pepper_control");

  proxy.call<void >("start");

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

    vpFaceTrackerOkao face_tracker(opt_ip,9559);

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(3.5, 0.8, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    servo_head.setLambda(lambda);

    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot_head;

    bool reinit_servo = true;
    unsigned long loop_iter = 0;

    std::vector<std::string> recognized_names;
    std::map<std::string,unsigned int> detected_face_map;
    bool detection_phase = true;
    unsigned int f_count = 0;

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
      bool face_found = face_tracker.detect();


      if (face_found) {
        std::ostringstream text;
        text << "Found " << face_tracker.getNbObjects() << " face(s)";
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        for(size_t i=0; i < face_tracker.getNbObjects(); i++) {
          vpRect bbox = face_tracker.getBBox(i);
          if (i == 0)
            vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 2);
          else
            vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), face_tracker.getMessage(i) , vpColor::red);
        }

        double u = face_tracker.getCog(0).get_u();
        double v = face_tracker.getCog(0).get_v();
        if (u<= g.getWidth() && v <= g.getHeight())
          head_cog_cur.set_uv(u,v);

        vpRect bbox = face_tracker.getBBox(0);
        std::string name = face_tracker.getMessage(0);

        // Get Head Jacobian (6x2)
        vpMatrix eJe = robot.get_eJe("Head");

        // Add column relative to the base rotation (Wz)
        vpColVector col_wz(6);
        col_wz[5] = 1;
        eJe.resize(6,3,false);
        for (unsigned int i =0; i <6; i++)
          eJe[i][2] = col_wz[i];

        //        std::cout << "JAC" << std::endl << eJe << std::endl;

        servo_head.set_eJe( eJe );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        //vpDisplay::setFont(I, "-*-*-bold-*-*-*-*-*-*-*-*-*-*-*");
        //vpDisplay::displayText(I, face_tracker.getFace().getTopLeft()+vpImagePoint(-20,0), "Coraline", vpColor::red);
        //vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        q_dot_head = servo_head.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        std::vector<float> vel(jointNames_head.size());
        for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
          vel[i] = q_dot_head[i];
        }

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);
        //if (distance > 0.03*I.getWidth())

        std::cout << "vel" << std::endl << q_dot_head << std::endl;
        proxy.async<void >("setDesJointVelocity", jointNames_head,vel );
        robot.getProxy()->move(0.0, 0.0, q_dot_head[2]);

        vpColVector vel_head = robot.getJointVelocity(jointNames_head);
        for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
          plotter_diff_vel.plot(i,1,loop_iter,q_dot_head[i]);
          plotter_diff_vel.plot(i,0,loop_iter,vel_head[i]);
          plotter_error.plot(i,0,loop_iter,servo_head.m_task_head.getError()[i]);
        }

        if (detection_phase)
        {

          //if (score >= 0.4 && distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          if (distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          {
            vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 1);
            vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::red);
            detected_face_map[name]++;
            f_count++;
          }
          else
          {
            vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
            vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::green);
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

      }
      else {
        proxy.call<void >("stopJoint");
        robot.getProxy()->move(0.0, 0.0, 0.0);
        std::cout << "Stop!" << std::endl;
        reinit_servo = true;
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      loop_iter ++;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }

    vpDisplay::getClick(I, true);

    proxy.call<void >("stop");
    robot.getProxy()->move(0.0, 0.0, 0.0);


  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  proxy.call<void >("stop");
  robot.getProxy()->move(0.0, 0.0, 0.0);



  return 0;
}





