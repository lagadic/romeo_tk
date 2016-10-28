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
 * This example demonstrates detect an objects using the color information.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example tracking_box_color.cpp */
#include <iostream>
#include <string>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Visp
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <vpColorDetection.h>
#include <vpServoHead.h>

using namespace cv;

int main(int argc, const char* argv[])
{

  std::string opt_ip;

  std::vector<std::string> opt_names;
  std::string opt_name_file;
  unsigned int num_objects = 0;


  bool opt_learning = false;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--learn-color")
      opt_learning = true;
    else if (std::string(argv[i]) == "--object_name")
    {
      int num = atoi(argv[i+1]);
      for (unsigned int k = 2;k <=num+1;k++)
      {
        opt_names.push_back(std::string(argv[i+k]));
      }
    }
    else if (std::string(argv[i]) == "--file_name")
      opt_name_file = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn-color] [--object_name <name>]" << std::endl;
      std::cout <<                         "[--file_name <path>]" << std::endl;
      return 0;
    }
  }

  // opt_name_file = opt_name + "HSV.txt";

  std::cout <<"Names " << opt_names << std::endl;
  num_objects = opt_names.size();
  std::vector<vpColor> color_rects(num_objects);

  if (opt_learning && num_objects != 1)
  {
    std::cout << "Please learn one object each time." << std::endl;
    return 0;
  }

  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(15);
  g.setCamera(2); // CameraLeftEye
  g.open();
  vpCameraParameters cam = g.getCameraParameters();
  vpHomogeneousMatrix eMc = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");


  vpImage<vpRGBa> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");
  bool click_done = false;
  vpMouseButton::vpMouseButtonType button;


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


  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");
  vpColVector head_pos(jointNames_head.size());
  head_pos = 0;
  head_pos[1] = vpMath::rad(-10.); // NeckPitch
  head_pos[2] = vpMath::rad(0.); // HeadPitch
  //robot.setPosition(jointNames_head, head_pos, 0.3);

  vpTime::sleepMs(1000);

  try
  {
    // Initialize head servoing
    vpServoHead servo_head;
    servo_head.setCameraParameters(cam);
    vpAdaptiveGain lambda(2, 2.0, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    //vpAdaptiveGain lambda(3, 1., 30);
    servo_head.setLambda(lambda);

    double servo_time_init = 0;
    bool reinit_servo = true;

    vpImagePoint head_cog_cur(0.0,0.0);
    vpImagePoint head_cog_des(I.getHeight()/2, I.getWidth()/2);
    //vpImagePoint head_cog_prev(0.0,0.0);
    vpColVector q_dot_head;
    vpColVector q_dot_tot;

    vpMatrix MAP_head(6,5);
    for (unsigned int i = 0; i < 3 ; i++)
      MAP_head[i][i]= 1;
    MAP_head[4][3]= 1;
    MAP_head[5][4]= 1;


    std::vector <vpColorDetection> objects (num_objects);



    for (unsigned int i; i < num_objects ; i++)
    {
      objects[i].setName(opt_names[i]);

      color_rects.at(i).id = vpColor::vpColorIdentifier( std::rand() % ( 18 + 1 ) );


      if (!opt_learning )
      {
        std::string filename = opt_names[i] + "HSV.txt";
        if (!objects[i].loadHSV(filename))
        {
          std::cout << "Cannot load file " << opt_name_file << std::endl;
          return 0;
        }
      }

    }

    Mat cvI = Mat(Size(g.getWidth(), g.getHeight()), CV_8UC3);


    vpColVector sum_dedt(servo_head.m_task_head.getDimension());
    double opt_mu = 0.05;

    sum_dedt = 0.;


    // bool first_detection = true;

    while(1)
    {
      if (reinit_servo) {
        servo_time_init = vpTime::measureTimeSecond();
        reinit_servo = false;
      }
      double t = vpTime::measureTimeMs();

      g.acquire(cvI);
      vpImageConvert::convert(cvI, I);
      vpDisplay::display(I);
      click_done = vpDisplay::getClick(I, button, false);

      if (opt_learning)
      {
        unsigned int index = 0;
        objects[index].learningColor(cvI);
        for(size_t i=0; i <  objects[index].getNbObjects(); i++) {
          vpRect bbox =  objects[index].getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::red, false, 1);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), objects[index].getMessage(i), vpColor::red);
        }
      }

      else
      {
        unsigned int k = 0;
        bool obj_found =  objects[k].detect(cvI);

        if (obj_found) {
          unsigned int i = 0; // We want the biggest object


          //          if (!first_detection && objects[k].getNbObjects() > 1 )
          //          {
          //            double dist = 1000.0;

          //            for (unsigned int j = 0; j < 2; j++ )
          //            {
          //              double dist_temp = vpImagePoint::distance(head_cog_prev,objects[k].getCog(j));
          //              if (dist_temp < dist)
          //                i = j;
          //            }
          //          }
          //          first_detection = false;

          vpRect bbox =  objects[k].getBBox(i);

          vpDisplay::displayRectangle(I, bbox, color_rects[k], false, 1);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), objects[k].getMessage(i), color_rects[k]);

          head_cog_cur = objects[k].getCog(i);


          vpMatrix eJe = robot.get_eJe("LEye") * MAP_head;

          servo_head.set_eJe( eJe );
          servo_head.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

          servo_head.setCurrentFeature(head_cog_cur);
          servo_head.setDesiredFeature(head_cog_des);
          vpServoDisplay::display(servo_head.m_task_head, cam, I, vpColor::green, vpColor::red, 3);


          q_dot_head = servo_head.computeControlLaw(servo_time_init);



          //          sum_dedt += servo_head.m_task_head.getError();
          //          q_dot_head -= opt_mu*servo_head.m_task_head.getTaskJacobianPseudoInverse()*sum_dedt;


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


      }

      vpDisplay::flush(I);

      if (click_done && button == vpMouseButton::button3) {
        click_done = false;
        break;
      }
      else if (click_done && button == vpMouseButton::button1 && opt_learning) {
        opt_learning = false;
        //std::vector<int> HSV_values = tests.getValueHSV();
        std::string filename = opt_names[0] + "HSV.txt";
        objects[0].saveHSV(filename);
        cv::destroyAllWindows();
      }
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

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


