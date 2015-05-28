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

/*! \example vpColoDetectionExample.cpp */
#include <iostream>
#include <string>

#include <alproxies/altexttospeechproxy.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Visp
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <vpColorDetection.h>

using namespace cv;

/*!

   Connect to Nao or Romeo robot, grab and display images using OpenCV.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./image_viewer_opencv -ip <robot ip address>

   Example:

   ./image_viewer_opencv -ip 169.254.168.230
 */





int main(int argc, const char* argv[])
{
  try
  {



    std::string opt_ip = "198.18.0.1";

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
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn-color] [--object_name <numberObjects> <name1 name2 ...>]" << std::endl;
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
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }
    g.setFramerate(15);
    g.setCamera(0);
    g.open();

    g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,1);
    g.setCameraParameter(AL::kCameraAutoExpositionID,1);

//   // g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraBrightnessID);
//    g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraContrastID);
//    g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraSaturationID);
//    g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraHueID);
//    g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraGainID);


    std::vector<int> result(4);
    result[0] = g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraContrastID);
    result[1] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraSaturationID);
    result[2] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraHueID);
    result[3] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraGainID);

    std::cout<< "Result constrast, sat, Hue, Gain " << std::endl << result << std::endl;


    // Open Proxy for the speech
    AL::ALTextToSpeechProxy tts(opt_ip, 9559);
    std::string phraseToSay;
    tts.setLanguage("English");
    phraseToSay = " \\emph=2\\ Hi,\\pau=200\\ How are you ?";


    std::vector <bool> firstTime(num_objects);


    vpImage<vpRGBa> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");
    bool click_done = false;
    vpMouseButton::vpMouseButtonType button;



    std::vector <vpColorDetection> objects (num_objects);

    for (unsigned int i; i < num_objects ; i++)
    {
      objects[i].setName(opt_names[i]);
      firstTime[i] = true;
      objects[i].setMaxAndMinObjectArea(130.0,4000.0);
      objects[i].setLevelMorphOps(false);
      objects[i].setShapeRecognition(true);

      color_rects.at(i).id = vpColor::vpColorIdentifier( std::rand() % ( 18 + 1 ) );


      if (!opt_learning )
      {
        std::cout << "Try to load the color " << objects[i].getName() << std::endl;
        std::string filename = opt_names[i] + "HSV.txt";
        if (!objects[i].loadHSV(filename))
        {
          std::cout << "Cannot load file " << opt_name_file << std::endl;
          return 0;
        }
      }

    }


    Mat cvI = Mat(Size(g.getWidth(), g.getHeight()), CV_8UC3);

    while(1)
    {
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

        for (unsigned int k = 0; k < num_objects ; k++)
        {
          bool obj_found =  objects[k].detect(cvI);

          if (obj_found) {
            for(size_t i=0; i <  objects[k].getNbObjects(); i++) {
              vpRect bbox =  objects[k].getBBox(i);
              vpDisplay::displayRectangle(I, bbox, color_rects[k], false, 2);
              vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(),
                                     objects[k].getMessage(i), color_rects[k]);

              if (firstTime[k]) {
                tts.post.say("\\emph=2\\" + opt_names[k]);
                firstTime[k] = false;
              }

            }
          }


        }


      }

      vpDisplay::flush(I);

      if (click_done && button == vpMouseButton::button3) {
        click_done = false;
        g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,1);
        g.setCameraParameter(AL::kCameraAutoExpositionID,1);
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





#if 0
    std::string opt_ip;

    std::string opt_name = "object";
    std::string opt_name_file;


    bool opt_learning = false;

    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "--learn-color")
        opt_learning = true;
      else if (std::string(argv[i]) == "--object_name")
        opt_name = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--file_name")
        opt_name_file = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn-color] [--object_name <name>]" << std::endl;
        std::cout <<                         "[--file_name <path>]" << std::endl;
        return 0;
      }
    }

    opt_name_file = opt_name + "HSV.txt";

    vpNaoqiGrabber g;
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }
    g.setFramerate(15);
    g.setCamera(0);
    g.open();

    vpImage<vpRGBa> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");
    bool click_done = false;
    vpMouseButton::vpMouseButtonType button;


    vpColorDetection tests;
    tests.setName(opt_name);
    if (!opt_learning )
    {
      if (!tests.loadHSV(opt_name_file))
      {
        std::cout << "Cannot load file " << opt_name_file << std::endl;
        return 0;
      }
    }

    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      click_done = vpDisplay::getClick(I, button, false);

      if (opt_learning)
      {
        tests.learningColor(I);
      }

      else
      {
        bool obj_found = tests.detectRGB(I);

        if (obj_found) {
          //std::ostringstream text;
          //text << "Found " << tests.getNbObjects() << " object(s)";
          //vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
          for(size_t i=0; i < tests.getNbObjects(); i++) {
            std::vector<vpImagePoint> p = tests.getPolygon(i);
            vpRect bbox = tests.getBBox(i);
            vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 2);
            vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), "Message: \"" + tests.getMessage(i) + "\"", vpColor::red);
          }
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
        tests.saveHSV(opt_name_file);
        cv::destroyAllWindows();
      }
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }







#endif



    //    std::cout << "Image size: " << g.getWidth() << " " << g.getHeight() << std::endl;
    //    // Create an OpenCV image container
    //    Mat src = Mat(Size(g.getWidth(), g.getHeight()), CV_8UC3);

    //    // Create an OpenCV window to display the images
    //    namedWindow("images");

    //    vpColorDetection tests;

    //    // Main loop. Exit when pressing ESC
    //    while (1)//(char) waitKey(30) != 27)

    //    {

    //      int key = waitKey(15) ;

    //      double t = vpTime::measureTimeMs();
    //      g.acquire(src);



    //      if (opt_learning)
    //      {
    //        tests.learningColor(src);

    //        //if ((char)key == 32)

    //      }
    //      // Display the image on screen
    //      imshow("images", src);
    //      if ((char)key == 32)
    //        break;



    //      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    //    }
  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  catch (const AL::ALError &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  return 0;
}


