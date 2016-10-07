#include <iostream>
#include <string>
#include <map>


#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alspeechrecognitionproxy.h>
#include <alproxies/alpeopleperceptionproxy.h>
#include <alproxies/alledsproxy.h>

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
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpFaceTrackerOkao.h>


/*!

Connect to Pepper robot, grab, display images using ViSP, start face detection with Okao and peoplepercetion.

The aim is to control the head joints (HeadYaw and HeadPitch) and the base (Vx, Vy, Wz) to follow a person.

By default, this example connects to a robot with IP address: 131.254.10.126.

If you want to connect to another robot, run:
  ./servo_face_detection_okao_head_translation_base_pepper --ip <robot ip address>

  Example:

  ./servo_face_detection_okao_head_translation_base_pepper --ip 169.254.168.230
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
  bool opt_debug = false;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--debug")
      opt_debug = false;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--fr]" << std::endl;
      return 0;
    }
  }



  // Start Velocity controller proxy
  qi::SessionPtr session = qi::makeSession();
  std::string connection_string = "tcp://" + opt_ip +":9559";
  session->connect(connection_string);
  qi::AnyObject proxy = session->service("pepper_control");

  proxy.call<void >("start");

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
  std::cout << "cam:" << std::endl << cam << std::endl;

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

  // Inizialize PeoplePerception
  AL::ALPeoplePerceptionProxy people_proxy(opt_ip, 9559);
  AL::ALMemoryProxy m_memProxy(opt_ip, 9559);
  people_proxy.subscribe("People", 30, 0.0);
  std::cout << "period: " << people_proxy.getCurrentPeriod() << std::endl;

  // Open Proxy for the recognition speech
  AL::ALSpeechRecognitionProxy asr(opt_ip, 9559);

  //  asr.unsubscribe("Test_ASR");

  asr.setVisualExpression(false);
  asr.setLanguage("English");
  std::vector<std::string> vocabulary;
  vocabulary.push_back("move");
  vocabulary.push_back("stop");

  // Set the vocabulary
  asr.setVocabulary(vocabulary,false);

  // Start the speech recognition engine with user Test_ASR
  asr.subscribe("Test_ASR");
  std::cout << "Speech recognition engine started" << std::endl;

  // Proxy to control the leds
  AL::ALLedsProxy led_proxy(opt_ip, 9559);



  vpPlot * plotter_diff_vel; vpPlot *plotter_vel;
  vpPlot *plotter_error;  vpPlot * plotter_distance;

  if (opt_debug)
  {
    // Plotting
    plotter_diff_vel = new vpPlot (2);
    plotter_diff_vel->initGraph(0, 2);
    plotter_diff_vel->initGraph(1, 2);
    plotter_diff_vel->setTitle(0,  "HeadYaw");
    plotter_diff_vel->setTitle(1,  "HeadPitch");

    plotter_vel= new vpPlot (1);
    plotter_vel->initGraph(0, 5);
    plotter_vel->setLegend(0, 0, "vx");
    plotter_vel->setLegend(0, 1, "vy");
    plotter_vel->setLegend(0, 2, "wz");
    plotter_vel->setLegend(0, 3, "q_yaw");
    plotter_vel->setLegend(0, 4, "q_pitch");

    plotter_error = new vpPlot(1);
    plotter_error->initGraph(0, 3);
    plotter_error->setLegend(0, 0, "x");
    plotter_error->setLegend(0, 1, "y");
    plotter_error->setLegend(0, 2, "Z");

    plotter_distance = new vpPlot (1);
    plotter_distance->initGraph(0, 1);
    plotter_distance->setLegend(0, 0, "dist");
  }

  try {
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTrackerOkao face_tracker(opt_ip,9559);

    double dist = 0.0; // Distance between person detected from peoplePerception and faceDetection

    // Set Visual Servoing:
    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    //    vpAdaptiveGain lambda_adapt;
    //    lambda_adapt.initStandard(1.6, 1.8, 15);
    vpAdaptiveGain lambda_base(2.3, 0.7, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    vpAdaptiveGain lambda_nobase(4, 0.5, 15); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    task.setLambda(lambda_base) ;

    double Z = 1.2;
    double Zd = 1.2;
    bool stop_vxy = false;
    bool move_base = true;
    bool move_base_prev = true;


    // Create the desired  visual feature
    vpFeaturePoint s;
    vpFeaturePoint sd;
    vpImagePoint ip(I.getHeight()/2, I.getWidth()/2);
    // Create the current x visual feature
    vpFeatureBuilder::create(s, cam, ip);
    vpFeatureBuilder::create(sd, cam, ip);

    //   sd.buildFrom( I.getWidth()/2, I.getHeight()/2, Zd);

    AL::ALValue limit_yaw = robot.getProxy()->getLimits("HeadYaw");

    std::cout << limit_yaw[0][0] << " " << limit_yaw[0][1] << std::endl;

    // Add the feature
    task.addFeature(s, sd) ;

    vpFeatureDepth s_Z, s_Zd;
    s_Z.buildFrom(s.get_x(), s.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Zd.buildFrom(sd.get_x(), sd.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0

    // Add the feature
    task.addFeature(s_Z, s_Zd);

    // Jacobian 6x5 (vx,vy,wz,q_yaq,q_pitch)
    vpMatrix tJe(6,5);
    tJe[0][0]= 1;
    tJe[1][1]= 1;
    tJe[5][2]= 1;
    vpMatrix eJe(6,5);

    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot;

    bool reinit_servo = true;
    unsigned long loop_iter = 0;

    std::vector<std::string> recognized_names;
    std::map<std::string,unsigned int> detected_face_map;
    bool detection_phase = true;
    unsigned int f_count = 0;
    AL::ALValue leg_names  = AL::ALValue::array("HipRoll","HipPitch", "KneePitch" );
    AL::ALValue values  = AL::ALValue::array(0.0, 0.0, 0.0 );


    double t_prev = vpTime::measureTimeSecond();

    while(1) {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        t_prev = vpTime::measureTimeSecond();
        reinit_servo = false;
        led_proxy.fadeRGB("FaceLeds","white",0.1);
      }

      double t = vpTime::measureTimeMs();
      if (opt_debug)
      {
        g.acquire(I);
        vpDisplay::display(I);
      }
      // Detect face
      bool face_found = face_tracker.detect();
      stop_vxy = false;

      std::cout << "Loop time face_tracker: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

      // Check speech recognition result

      AL::ALValue result_speech = m_memProxy.getData("WordRecognized");

      if ( ((result_speech[0]) == vocabulary[0]) && (double (result_speech[1]) > 0.4 )) //move
      {
        std::cout << "Recognized: " << result_speech[0] << "with confidence of " << result_speech[1] << std::endl;
        task.setLambda(lambda_base) ;

        move_base = true;
      }
      else if ( (result_speech[0] == vocabulary[1]) && (double(result_speech[1]) > 0.4 )) //stop
      {
        std::cout << "Recognized: " << result_speech[0] << "with confidence of " << result_speech[1] << std::endl;
        task.setLambda(lambda_nobase) ;
        move_base = false;
      }

      if (move_base != move_base_prev)
      {
        if (move_base)
        {
          phraseToSay = "Ok, I will follow you.";
          tts.post.say(phraseToSay);
        }
        else
        {
          phraseToSay = "Ok, I will stop.";
          tts.post.say(phraseToSay);
        }

      }

      std::cout << "Loop time check_speech: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


      move_base_prev = move_base;

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

        led_proxy.post.fadeRGB("FaceLeds","blue",0.1);



        double u = face_tracker.getCog(0).get_u();
        double v = face_tracker.getCog(0).get_v();
        if (u<= g.getWidth() && v <= g.getHeight())
          head_cog_cur.set_uv(u,v);

        vpRect bbox = face_tracker.getBBox(0);
        std::string name = face_tracker.getMessage(0);


        std::cout << "Loop time face print " << vpTime::measureTimeMs() - t << " ms" << std::endl;



        AL::ALValue result = m_memProxy.getData("PeoplePerception/VisiblePeopleList");

        std::cout << "Loop time get Data PeoplePerception " << vpTime::measureTimeMs() - t << " ms" << std::endl;

        if (result.getSize() > 0)
        {
          AL::ALValue info = m_memProxy.getData("PeoplePerception/PeopleDetected");

          int num_people = info[1].getSize();

          bool found_person = false;
          vpImagePoint cog_face;
          double dist_min = 1000;
          unsigned int index_person = 0;

          for (unsigned int i = 0; i < num_people; i++)
          {

            float alpha =  info[1][i][2];
            float beta =  info[1][i][3];
            //Z = Zd;
            // Centre of face into the image
            float x =  g.getWidth()/2 -  g.getWidth() * beta;
            float y =  g.getHeight()/2  + g.getHeight() * alpha;
            cog_face.set_uv(x,y);
            dist = vpImagePoint::distance(cog_face, head_cog_cur);

            if (dist < dist_min)
            {
              dist_min = dist;
              index_person  = i;
            }
          }

          vpDisplay::displayCross(I, cog_face, 10, vpColor::red);

          if (dist_min < 55.)
            Z = info[1][index_person][1]; // Current distance
          else
            stop_vxy = true;
        }
        else
        {
          std::cout << "No distance computed " << std::endl;
          stop_vxy = true;
          //Z = Zd;
        }
        //          float alpha =  info[1][0][2];
        //          float beta =  info[1][0][3];
        //          //Z = Zd;
        //          // Centre of face into the image
        //          float x =  g.getWidth()/2 -  g.getWidth() * beta;
        //          float y =  g.getHeight()/2  + g.getHeight() * alpha;

        //          vpImagePoint cog_face(y,x);
        //          dist = vpImagePoint::distance(cog_face,head_cog_cur);
        //          if (dist < 55.)
        //            Z = info[1][0][1]; // Current distance
        //          else
        //            stop_vxy = true;
        //        }
        //        else
        //        {
        //          std::cout << "No distance computed " << std::endl;
        //          stop_vxy = true;
        //          //Z = Zd;
        //        }

        std::cout << "Loop time before VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

        // Get Head Jacobian (6x2)
        vpMatrix torso_eJe_head;
        robot.get_eJe("Head",torso_eJe_head);

        // Add column relative to the base rotation (Wz)
        vpColVector col_wz(6);
        col_wz[5] = 1;
        for (unsigned int i = 0; i < 6; i++)
          for (unsigned int j = 0; j < torso_eJe_head.getCols(); j++)
            tJe[i][j+3] = torso_eJe_head[i][j];

        // std::cout << "tJe" << std::endl << tJe << std::endl;

        //        vpHomogeneousMatrix torsoMHeadPith( robot.getProxy()->getTransform(jointNames_head[jointNames_head.size()-1], 0, true));// get transformation  matrix between torso and HeadRoll
        vpHomogeneousMatrix torsoMHeadPith( robot.getProxy()->getTransform("HeadPitch", 0, true));// get transformation  matrix between torso and HeadRoll

        vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

        for(unsigned int i=0; i< 3; i++)
          for(unsigned int j=0; j< 3; j++)
            HeadPitchVLtorso[i][j+3] = 0;

        //std::cout << "HeadPitchVLtorso: " << std::endl << HeadPitchVLtorso << std::endl;
        // Transform the matrix
        eJe = HeadPitchVLtorso *tJe;

        // std::cout << "eJe" << std::endl << eJe << std::endl;

        task.set_eJe( eJe );
        task.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        vpDisplay::displayCross(I, head_cog_des, 10, vpColor::blue);
        vpDisplay::displayCross(I, head_cog_cur, 10, vpColor::green);
        //  std::cout << "head_cog_des:" << std::endl << head_cog_des << std::endl;
        //  std::cout << "head_cog_cur:" << std::endl << head_cog_cur << std::endl;

        // Update the current x feature
        double x,y;
        vpPixelMeterConversion::convertPoint(cam, head_cog_cur, x, y);
        s.buildFrom(x, y, Z);
        //s.set_xyZ(head_cog_cur.get_u(), head_cog_cur.get_v(), Z);

        // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
        s_Z.buildFrom(s.get_x(), s.get_y(), Z, log(Z/Zd)) ;

        q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        std::cout << "Loop time compute VS: " << vpTime::measureTimeMs() - t << " ms" << std::endl;


        //        vpMatrix P = task.getI_WpW();
        //        double alpha = -4.3;

        //        double min = limit_yaw[0][0];
        //        double max = limit_yaw[0][1];

        //        vpColVector z_q2 (q_dot.size());
        //        z_q2[3] = 2 * alpha * q_dot[3]/ pow((max - min),2);

        //        vpColVector q3 = P * z_q2;
        //       std::cout << "q3: " << q3 << std::endl;
        //        if (q3.euclideanNorm()<10.0)
        //          q_dot =  q_dot + q3;

        std::vector<float> vel(jointNames_head.size());
        vel[0] = q_dot[3];
        vel[1] = q_dot[4];

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);
        //if (distance > 0.03*I.getWidth())
        //  std::cout << "e:" << std::endl << task.getError() << std::endl;
        //  std::cout << "vel" << std::endl << q_dot << std::endl;


        proxy.async<void >("setDesJointVelocity", jointNames_head, vel );
        // robot.getProxy()->setAngles(leg_names,values,1.0);


        std::cout << "errorZ: " << task.getError()[2] << std::endl;
        std::cout << "stop_vxy: " << stop_vxy << std::endl;

        if (std::fabs(Z -Zd) < 0.05 || stop_vxy || !move_base)
          robot.getProxy()->move(0.0, 0.0, q_dot[2]);
        else
          robot.getProxy()->move(q_dot[0], q_dot[1], q_dot[2]);

        if (opt_debug)
        {
          vpColVector vel_head = robot.getJointVelocity(jointNames_head);
          for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
            plotter_diff_vel->plot(i, 1, loop_iter, q_dot[i]);
            plotter_diff_vel->plot(i, 0, loop_iter, vel_head[i]);
          }
          plotter_error->plot(0,loop_iter,task.getError());
          plotter_vel->plot(0,loop_iter, q_dot);
          plotter_distance->plot(0,0,loop_iter,Z);
        }

        if (detection_phase)
        {

          //if (score >= 0.4 && distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          if (distance < 0.06*I.getWidth() && bbox.getSize() > 3000)
          {
            if (opt_debug)
            {
              vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 1);
              vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::red);
            }
            detected_face_map[name]++;
            f_count++;
          }
          else
          {
            if (opt_debug)
            {
              vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
              vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::green);
            }
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

      if (opt_debug)
        vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      loop_iter ++;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
    proxy.call<void >("stopJoint");
    robot.getProxy()->move(0.0, 0.0, 0.0);

    asr.unsubscribe("Test_ASR");

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

  proxy.call<void >("stopJoint");
  robot.getProxy()->move(0.0, 0.0, 0.0);
  proxy.call<void >("stop");
  led_proxy.fadeRGB("FaceLeds","white",0.1);



  return 0;
}





