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


#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alfacedetectionproxy.h>


#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpServoHead.h>
#include <vpJointLimitAvoidance.h>

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

  vpMouseButton::vpMouseButtonType button;


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
  //jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
  std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  unsigned int numJoints = jointNames.size();
  std::vector<std::string> jointNames_tot = jointNames;
  jointNames_tot.push_back(jointNamesREye.at(0));
  jointNames_tot.push_back(jointNamesREye.at(1));


  //    vpMatrix MAP_head(6,5);
  //    for (unsigned int i = 0; i < 3 ; i++)
  //      MAP_head[i][i]= 1;
  //    MAP_head[4][3]= 1;
  //    MAP_head[5][4]= 1;


  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");
  vpColVector head_pos(jointNames_head.size());
  head_pos = 0;
  head_pos[1] = vpMath::rad(-3); // NeckPitch
  head_pos[2] = vpMath::rad(-4.); // HeadPitch
  head_pos[0] = vpMath::rad(-10.); // NeckYaw
  robot.setPosition(jointNames_head, head_pos, 0.3);

  std::vector<std::string> jointNames_leye = robot.getBodyNames("LEye");
  vpColVector eye_pos(jointNames_leye.size());
  eye_pos = 0;
  robot.setPosition(jointNames_leye, eye_pos, 0.3);





  // Initialize the joint avoidance scheme from the joint limits
  vpColVector jointMin = robot.getJointMin("Head");
  //jointMin.stack(robot.getJointMin("LEye")); //Limits from AL
  //  jointMin.stack(-0.29753);
  //  jointMin.stack(-0.17940);
  jointMin.stack(vpMath::rad(-16.8));
  jointMin.stack(vpMath::rad(-14.8));


  vpColVector jointMax = robot.getJointMax("Head");
  // jointMax.stack(robot.getJointMax("LEye")); //Limits from AL
  //  jointMax.stack(0.297533);
  //  jointMax.stack(0.273738);

  jointMax.stack(vpMath::rad(17.2));
  jointMax.stack(vpMath::rad(15.3));


  std::cout << "limit max:" << jointMax << std::endl;
  std::cout << "limit min:" << jointMin << std::endl;


  // Vector secondary task
  vpColVector q2 (numJoints);

  //Vector data for plotting
  vpColVector data(12);

  vpColVector Qmiddle(numJoints);

  std::cout << "Joint limits arm: " << std::endl;

  for (unsigned int i=0; i< numJoints; i++)
  {
    Qmiddle[i] = ( jointMin[i] + jointMax[i]) /2.;
    std::cout << " Joint " << i << " " << jointNames[i]
                 << ": min=" << vpMath::deg(jointMin[i])
                 << " max=" << vpMath::deg(jointMax[i]) << std::endl;
  }


  //  double ro = 0.2;
  //  double ro1 = 0.4;

  double ro = 0.1;
  double ro1 = 0.3;

  vpColVector q_l0_min(numJoints);
  vpColVector q_l0_max(numJoints);
  vpColVector q_l1_min(numJoints);
  vpColVector q_l1_max(numJoints);

  bool opt_plotter_q_sep = true;
  bool opt_plotter_q = true;


  vpPlot *plotter_q_sep;
  vpPlot *plotter_q_sep1;
  if (opt_plotter_q_sep) {

    plotter_q_sep = new vpPlot(4, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+60, "Values of each q and limits");
    plotter_q_sep->initGraph(0, 7);
    plotter_q_sep->initGraph(1, 7);
    plotter_q_sep->initGraph(2, 7);
    plotter_q_sep->initGraph(3, 7);

    plotter_q_sep->setLegend(0, 0, "Low Limits");
    plotter_q_sep->setLegend(0, 1, "Upper Limits");
    plotter_q_sep->setLegend(0, 2, "l0 min");
    plotter_q_sep->setLegend(0, 3, "l0 max");
    plotter_q_sep->setLegend(0, 4, "l1 min");
    plotter_q_sep->setLegend(0, 5, "l1 max");
    plotter_q_sep->setLegend(0, 6, "q");

    plotter_q_sep->setTitle( 0, "NeckYaw");
    plotter_q_sep->setTitle( 1, "NeckPitch");
    plotter_q_sep->setTitle( 2, "HeadPitch");
    plotter_q_sep->setTitle( 3, "HeadRoll");


    plotter_q_sep1 = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+60, "Values of each q and limits");
    plotter_q_sep1->initGraph(0, 7);
    plotter_q_sep1->initGraph(1, 7);


    plotter_q_sep1->setLegend(0, 0, "Low Limits");
    plotter_q_sep1->setLegend(0, 1, "Upper Limits");
    plotter_q_sep1->setLegend(0, 2, "l0 min");
    plotter_q_sep1->setLegend(0, 3, "l0 max");
    plotter_q_sep1->setLegend(0, 4, "l1 min");
    plotter_q_sep1->setLegend(0, 5, "l1 max");
    plotter_q_sep1->setLegend(0, 6, "q");

    plotter_q_sep1->setTitle( 0, "LEyeYaw");
    plotter_q_sep1->setTitle( 1, "LEyePitch");


  }




  vpPlot *plotter_q;
  if (opt_plotter_q) {

    plotter_q = new vpPlot(1, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+100, I.display->getWindowYPosition()+60, "Values of q and limits");
    plotter_q->initGraph(0, 12);

    plotter_q->setTitle(0, "Q1 values normalized");

    plotter_q->setLegend(0, 0, "NeckYaw");
    plotter_q->setLegend(0, 1, "NeckPitch");
    plotter_q->setLegend(0, 2, "HeadPitch");
    plotter_q->setLegend(0, 3, "HeadRoll");
    plotter_q->setLegend(0, 4, "LEyeYaw");
    plotter_q->setLegend(0, 5, "LEyePitch");

    plotter_q->setLegend(0, 6, "Low Limits");
    plotter_q->setLegend(0, 7, "Upper Limits");
    plotter_q->setLegend(0, 8, "l0 min");
    plotter_q->setLegend(0, 9, "l0 max");
    plotter_q->setLegend(0, 10, "l1 min");
    plotter_q->setLegend(0, 11, "l1 max");

    plotter_q->setColor(0, 6,vpColor::darkRed);
    plotter_q->setThickness(0, 6,2);
    plotter_q->setColor(0, 8,vpColor::darkRed);
    plotter_q->setThickness(0, 8,2);
    plotter_q->setColor(0, 10,vpColor::darkRed);
    plotter_q->setThickness(0, 10,2);

    plotter_q->setColor(0, 7,vpColor::darkRed);
    plotter_q->setThickness(0, 7,2);
    plotter_q->setColor(0, 9,vpColor::darkRed);
    plotter_q->setThickness(0, 9,2);
    plotter_q->setColor(0, 11,vpColor::darkRed);
    plotter_q->setThickness(0, 11,2);

  }





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

    df.enableTracking(true);
    df.enableRecognition(false);


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
    vpColVector q;

    bool reinit_servo = true;
    bool speech = true;
    unsigned long loop_iter = 0;



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

      q = robot.getPosition(jointNames);


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

        std::string name = result[1][i][1][2];
        float score = result[1][i][1][1];

        // sizeX / sizeY are the face size in relation to the image

        float sizeX = mImageHeight * sx;
        float sizeY = mImageWidth * sy;

        // Centre of face into the image
        float x = mImageWidth / 2 - mImageWidth * alpha;
        float y = mImageHeight / 2 + mImageHeight * beta;

        vpDisplay::displayCross(I, y, x, 10, vpColor::red);
        vpDisplay::displayRectangle(I,y,x,0.0,sizeX,sizeY,vpColor::cyan,1);

        if (x<= mImageWidth && y <= mImageHeight)
        {
          head_cog_cur.set_uv(x,y);

          //servo_head.set_eJe( robot.get_eJe("LEye") * MAP_head );
          servo_head.set_eJe( robot.get_eJe("LEye"));
          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

          servo_head.setCurrentFeature(head_cog_cur);
          servo_head.setDesiredFeature(head_cog_des);
          vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);

          // q_dot_head = servo_head.computeControlLaw(servo_time_init);
          q_dot_head = servo_head.computeControlLaw();
          vpColVector e = servo_head.m_task_head.getError();
          vpMatrix TaskJac = servo_head.m_task_head.getTaskJacobian();
          vpMatrix TaskJacPseudoInv = servo_head.m_task_head.getTaskJacobianPseudoInverse();
          vpMatrix InteractionM = servo_head.m_task_head.getInteractionMatrix();
          //q2 = computeQdotLimitAvoidance(e, TaskJac, TaskJacPseudoInv, jointMin, jointMax, q, q_dot_head, ro, ro1, q_l0_min, q_l0_max, q_l1_min, q_l1_max );

          vpColVector q_dot_head_real = robot.getJointVelocity(jointNames_tot);
          q_dot_head_real.resize(6,false);
          q2 = servo_head.m_task_head.secondaryTaskJointLimitAvoidance(q, q_dot_head, jointMin, jointMax);


          if (q2.euclideanNorm()<10.0)
            q_dot_head =  q_dot_head + q2;

          // Add mirroring eyes
          q_dot_tot = q_dot_head;
          q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
          q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);
          std::cout << "q2: " << q2 << std::endl;
          robot.setVelocity(jointNames_tot, q_dot_tot);


        std::cout << "q dot: "<< std::endl << q_dot_head.t() << std::endl;
        std::cout << "LEye: " << std::endl<< robot.get_eJe("LEye") << std::endl;

        std::cout << "rank: "<< std::endl << servo_head.m_task_head.getTaskRank()<< std::endl;
        std::cout << "InteractionM: "<< std::endl << InteractionM << std::endl;
        std::cout << "TaskJacPseudoInv: "<< std::endl << TaskJacPseudoInv << std::endl;
        std::cout << "e: " << std::endl<< e << std::endl;
        std::cout << "head_cog_cur: " << std::endl<< head_cog_cur << std::endl;
        std::cout << "head_cog_des: " << std::endl<< head_cog_des << std::endl;

 }
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



      if (opt_plotter_q_sep )
      {

        vpColVector info(7);
        for (unsigned int i=0 ; i < numJoints ; i++) {
          info[0] = jointMin[i];
          info[1] = jointMax[i];
          info[2] = q_l0_min[i];
          info[3] = q_l0_max[i];
          info[4] = q_l1_min[i];
          info[5] = q_l1_max[i];
          info[6] = q[i];

          if (i < 4)
            plotter_q_sep->plot(i,loop_iter,info);
          else
            plotter_q_sep1->plot(i-4,loop_iter,info);
        }


      }




      if (opt_plotter_q  )
      {

        // q normalized between (entre -1 et 1)
        for (unsigned int i=0 ; i < numJoints ; i++) {
          data[i] = (q[i] - Qmiddle[i]) ;
          data[i] /= (jointMax[i] - jointMin[i]) ;
          data[i]*=2 ;
        }

        data[numJoints] = -1.0;
        data[numJoints+1] = 1.0;

        unsigned int joint = 0;
        double tQmin_l0 = jointMin[joint] + ro *(jointMax[joint] - jointMin[joint]);
        double tQmax_l0 = jointMax[joint] - ro *(jointMax[joint] - jointMin[joint]);

        double tQmin_l1 =  tQmin_l0 - ro * ro1 * (jointMax[joint] - jointMin[joint]);
        double tQmax_l1 =  tQmax_l0 + ro * ro1 * (jointMax[joint] - jointMin[joint]);

        data[numJoints+2] = 2*(tQmin_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
        data[numJoints+3] = 2*(tQmax_l0 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

        data[numJoints+4] =  2*(tQmin_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);
        data[numJoints+5] =  2*(tQmax_l1 - Qmiddle[joint])/(jointMax[joint] - jointMin[joint]);

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


      }



      loop_iter ++;
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }



    df.unsubscribe("Face");


    while(1)
    {

      bool click_done = vpDisplay::getClick(I, button, false);

      if (click_done && button == vpMouseButton::button3) { // Quit the loop
        break;
      }

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

