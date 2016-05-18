/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://team.inria.fr/lagadic/visp for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://team.inria.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 *   Connect to Romeo robot, grab, display images using ViSP and start face detection with Okao library running on the robot.
 *   More over all the four joints of Romeo's head plus the two joint of the Left eye are controlled by visual servoing to center
 *   the detected head in the image.
 *   By default, this example connect to a robot with ip address: 198.18.0.1.
 *
 * Authors:
 * Giovanni Claudio
 *
 * Note: http://abitworld.com/nao-robot-visual-debugging-of-the-vision-variables/
 *
 *****************************************************************************/

/*! \example face_detection_okao.cpp */
#include <iostream>
#include <string>
#include <map>


#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alfacedetectionproxy.h>


#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <vpServoHead.h>
#include <vpFaceTrackerOkao.h>

bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}

bool pred(const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs)
{
  return lhs.second < rhs.second;
}

/*!

  Connect to Romeo robot, grab, display images using ViSP and start face detection with Okao library running on the robot.
  More over all the four joints of Romeo's head plus the two joint of the Left eye are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 198.18.0.1.

 */
int main(int argc, const char* argv[])
{

  std::string opt_ip = "198.18.0.1";

  bool opt_language_english = true;
  bool opt_clear_database = false;
  bool opt_forget_person = false;
  std::string opt_name_to_forget = "";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--forget")
    {
      opt_forget_person = true;
      opt_name_to_forget = std::string(argv[i+1]);
    }
    else if (std::string(argv[i]) == "--clear-database")
      opt_clear_database = true;
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--forget name_person_to_forget] [--clear-database] [--fr]" << std::endl;

      return 0;
    }
  }

  // Connect to the robot
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(15);
  g.setCamera(2); // CameraLeftEye
  g.open();
  vpCameraParameters cam = g.getCameraParameters();
  vpHomogeneousMatrix eMc = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");

  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");


  std::vector<std::string> jointNames;
  jointNames.push_back("TrunkYaw");
  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");
  jointNames.insert(jointNames.end(),jointNames_head.begin(),jointNames_head.end());

  jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  std::vector<std::string> jointNames_tot = jointNames;
  jointNames_tot.push_back(jointNamesREye.at(0));
  jointNames_tot.push_back(jointNamesREye.at(1));

  std::cout << jointNames << std::endl;

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
  if (!opt_clear_database || !opt_forget_person)
  {
    robot.setPosition(jointNames_tot, head_pos, 0.2);
    vpTime::sleepMs(1000);
  }

  std::vector<std::string> recognized_names;
  std::map<std::string,unsigned int> detected_face_map;
  bool detection_phase = true;

  try
  {
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

    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(2, 1.5, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    //vpAdaptiveGain lambda(4, 2, 40);
    //vpAdaptiveGain lambda(3, 1., 30);
    servo_head.setLambda(lambda);
    //servo_head.setLambda(2.0);
    double servo_time_init = 0;

    vpImagePoint head_cog_cur;
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    vpColVector q_dot_head;
    vpColVector q_dot_tot;
    vpColVector q;

    bool reinit_servo = true;
    bool speech = true;
    unsigned int f_count = 0;

    vpFaceTrackerOkao face_tracker(opt_ip,9559);

    if (opt_clear_database)
    {
      if (face_tracker.clearDatabase())
        std::cout << "******* All the faces are removed from the database" << std::endl;
      else
        std::cout << "ERROR: Cannot clear the database" << std::endl;
      return 0;
    }

    if (opt_forget_person)
    {
      if (face_tracker.forgetPerson(opt_name_to_forget))
        std::cout <<"*******" << opt_name_to_forget <<" is removed from the database." << std::endl;
      else
        std::cout << "ERROR: Cannot remove " <<opt_name_to_forget << " from the database." << std::endl;
      return 0;
    }

    while (1)
    {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      //Get actual position joint of the robot
      q = robot.getPosition(jointNames);
      q[0] = -q[0];

      bool result = face_tracker.detect();

      if (result)
      {

        std::ostringstream text;
        text << "Found " << face_tracker.getNbObjects() << " face(s)";
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        for(size_t i=1; i < face_tracker.getNbObjects(); i++) {
          vpDisplay::displayCross(I, face_tracker.getCog(i), 3, vpColor::red);
          //vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
          //vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), face_tracker.getMessage(i) , vpColor::red);
        }

        vpRect bbox = face_tracker.getBBox(0);
        std::string name = face_tracker.getMessage(0);
        float score = face_tracker.getScore(0);
        // vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
        vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), name, vpColor::red);

        double u = face_tracker.getCog(0).get_u();
        double v = face_tracker.getCog(0).get_v();
        if (u<= g.getWidth() && v <= g.getHeight())
          head_cog_cur.set_uv(u,v);

        servo_head.set_eJe( robot.get_eJe("LEye_t")* MAP_head );
        servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        servo_head.setCurrentFeature(head_cog_cur);
        servo_head.setDesiredFeature(head_cog_des);
        vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

        q_dot_head = servo_head.computeControlLaw(servo_time_init);

        //        q2 = servo_head.m_task_head.secondaryTaskJointLimitAvoidance(q, q_dot_head, jointMin, jointMax);

        //        if (q2.euclideanNorm()<10.0)
        //          q_dot_head =  q_dot_head + q2;

        //std::cout << "q2: " << q2 << std::endl;
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


        //q_dot_head[0] = -q_dot_head[0];
        // Add mirroring eyes
        q_dot_tot = q_dot_head;
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
        q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

        // Compute the distance in pixel between the target and the center of the image
        double distance = vpImagePoint::distance(head_cog_cur, head_cog_des);
        if (distance > distance < 0.03*I.getWidth())
          robot.setVelocity(jointNames_tot, q_dot_tot);

        std::cout << "q dot: " << q_dot_head.t() << std::endl;

        if (detection_phase)
        {
          //std::cout << "score: " << score << std::endl;
          //std::cout << "distance: " << distance << std::endl;
          //std::cout << "size: " << bbox.getSize() << " < 4000 " << std::endl;

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
        robot.stop(jointNames_tot);
        reinit_servo = true;
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      //std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }

  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  catch (const AL::ALError &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames_tot);


  return 0;
}

