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
 * This example demonstrates how to control the robot remotely in position and velocity.
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


#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
#include<alproxies/alfacedetectionproxy.h>


#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <vpServoHead.h>


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

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
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

    AL::ALFaceDetectionProxy df(opt_ip, 9559);


    // Start the face recognition engine
    const int period = 1;

    df.subscribe("Face", period, 0.0);


    AL::ALMemoryProxy memProxy(opt_ip, 9559);

    float  mImageHeight = g.getHeight();
    float  mImageWidth = g.getWidth();

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



    while (1)
    {

      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);


      AL::ALValue result = memProxy.getData("FaceDetected");

      bool face_found = false;


      if (result.getSize() >=2)
      {

        face_found = true;

        //        for (unsigned int i = 0; i < info_face_array.getSize()-1; i++ )
        //        {

        unsigned int i = 0;

        //Extract face info

        // Face Detected [1]/ First face [0]/ Shape Info [0]/ Alpha [1]
        float alpha = result[1][0][0][1];
        float beta = result[1][0][0][2];
        float sx = result[1][0][0][3];
        float sy = result[1][0][0][4];

        // sizeX / sizeY are the face size in relation to the image

        float sizeX = mImageHeight * sx;
        float sizeY = mImageWidth * sy;

        // Centre of face into the image
        float x = mImageWidth / 2 - mImageWidth * alpha;
        float y = mImageHeight / 2 + mImageHeight * beta;

        vpDisplay::displayCross(I, y, x, 10, vpColor::red);

        vpDisplay::displayRectangle(I,y,x,0.0,sizeX,sizeY,vpColor::cyan,1);

        //          cv::Point p1(x - (sizeX / 2), y - (sizeY / 2));
        //          cv::Point p2(x + (sizeX / 2), y + (sizeY / 2));

        //          cv::rectangle(I, p1, p2, cv::Scalar(255,255,255));

        //        }


        head_cog_cur.set_uv(x,y);

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



    df.unsubscribe("Face");





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

