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
 *****************************************************************************/

/*! \example qrcode_detector.cpp */
#include <iostream>
#include <string>

#include <visp/vpImagePoint.h>


#include <alproxies/almemoryproxy.h>
#include <alproxies/alredballdetectionproxy.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>


/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion --ip <robot ip address>

   Example:

   ./motion --ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip = "198.18.0.1";

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
    }


    vpNaoqiGrabber g;
    if (! opt_ip.empty())
      g.setRobotIp(opt_ip);
    g.setCamera(0);
    g.open();

//    vpCameraParameters cam = g.getCameraParameters();
//    vpCameraParameters cam_kvga = g.getCameraParameters(AL::kVGA,"CameraLeftEye");

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    //    vpNaoqiRobot robot;
    //    if (! opt_ip.empty()) {
    //      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    //      robot.setRobotIp(opt_ip);
    //    }

    //    robot.open();


    AL::ALRedBallDetectionProxy color_proxy(opt_ip, 9559);

   // color_proxy.unsubscribe("red_circlse");
    color_proxy.subscribe("red_circlse");


    AL::ALMemoryProxy m_memProxy(opt_ip, 9559);

    float  mImageHeight = g.getHeight();
    float  mImageWidth = g.getWidth();



    while (1)
    {

      g.acquire(I);
      vpDisplay::display(I);

      AL::ALValue result = m_memProxy.getData("redBallDetected");

      if (result.getSize() > 0)
      {
       // int name = result[0][0];


        float alpha = result[1][0];
        float beta = result[1][1];
        float sx = result[1][2];
        float sy = result[1][3];


        float sizeX = mImageHeight * sx;
        float sizeY = mImageWidth * sy;

        // Centre of face into the image
        float x = mImageWidth / 2 - mImageWidth * alpha;
        float y = mImageHeight / 2 + mImageHeight * beta;

        vpDisplay::displayCross(I, y, x, 10, vpColor::red);
        vpDisplay::displayRectangle(I,y,x,0.0,sizeX,sizeY,vpColor::cyan,1);



        std::cout << "P: " << x << " , " <<  y << std::endl;

      }
      else
        std::cout << "No Red Blob Found" << std::endl;




      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;

    }


    color_proxy.unsubscribe("red_circlse");

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

