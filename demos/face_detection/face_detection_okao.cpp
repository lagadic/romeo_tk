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
 * Connect to Romeo robot, grab, display images using ViSP and start face detection with Okao library running on the robot.
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
#include <vector>


#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alfacedetectionproxy.h>


#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

#include <visp_naoqi/vpNaoqiGrabber.h>





bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}


/*!

  Connect to Romeo robot, grab, display images using ViSP and start face detection with Okao library running on the robot.
  By default, this example connect to a robot with ip address: 198.18.0.1.

 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip = "198.18.0.1";;

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
    }

    vpNaoqiGrabber g;
    if (! opt_ip.empty())
      g.setRobotIp(opt_ip);
    g.setCamera(0);
    g.open();

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    // Open Proxy for the speech
    AL::ALTextToSpeechProxy tts(opt_ip, 9559);
    tts.setLanguage("English");

    std::string phraseToSay = "Hi!";
    //    int id = tts.post.say(phraseToSay);
    //    tts.wait(id,2000);
    // Open Face detection proxy

    AL::ALFaceDetectionProxy df(opt_ip, 9559);


    // Start the face recognition engine
    const int period = 1;

    df.subscribe("Face", period, 0.0);


    AL::ALMemoryProxy memProxy(opt_ip, 9559);

    float  mImageHeight = g.getHeight();
    float  mImageWidth = g.getWidth();
    std::vector<std::string> names;


    while (1)
    {

      g.acquire(I);
      vpDisplay::display(I);


      AL::ALValue result = memProxy.getData("FaceDetected");



      if (result.getSize() >=2)
      {
        //std::cout << "face" << std::endl;
        AL::ALValue info_face_array = result[1];

        for (unsigned int i = 0; i < info_face_array.getSize()-1; i++ )
        {
          // AL::ALValue info_face = info_face_array[i];

          //Extract face info

          //AL::ALValue shape_face =info_face[0];

          //std::cout << "alpha "<< shape_face[1] <<". Beta:" << shape_face[1] << std::endl;
          // Face Detected [1]/ First face [0]/ Shape Info [0]/ Alpha [1]
          float alpha = result[1][i][0][1];
          float beta = result[1][i][0][2];
          float sx = result[1][i][0][3];
          float sy = result[1][i][0][4];

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

          std::cout << i << "- size: " << sizeX*sizeY << std::endl;

          if (score >= 0.4)
          {
            std::cout << i << "- Name: " << name <<" score "  << score << std::endl;

            //            std::cout << "NAMES: " << names << std::endl;
            //            std::cout << "Esite: " << in_array(name, names) << std::endl;
            //            std::cout << "==================================== " << std::endl;

            if (!in_array(name, names) && sizeX*sizeY > 4000) {
              std::string phraseToSay = "\\emph=2\\ Hi \\wait=200\\ \\emph=2\\" + name;
              std::cout << phraseToSay << std::endl;
              tts.post.say(phraseToSay);
              names.push_back(name);

            }
          }

          else
            std::cout << i << "- Face not recognized. " << std::endl;

          //          cv::Point p1(x - (sizeX / 2), y - (sizeY / 2));
          //          cv::Point p2(x + (sizeX / 2), y + (sizeY / 2));
          //          cv::rectangle(I, p1, p2, cv::Scalar(255,255,255));

        }

      }

      //vpTime::sleepMs(60);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;

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



  return 0;
}

