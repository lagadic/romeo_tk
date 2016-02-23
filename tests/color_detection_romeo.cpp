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
#include <alproxies/alcolorblobdetectionproxy.h>

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


//    vpNaoqiGrabber g;
//    if (! opt_ip.empty())
//      g.setRobotIp(opt_ip);
//    g.setCamera(2);
//    g.open();

//    vpCameraParameters cam = g.getCameraParameters();
//    vpCameraParameters cam_kvga = g.getCameraParameters(AL::kVGA,"CameraLeftEye");

//    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
//    vpDisplayX d(I);
//    vpDisplay::setTitle(I, "ViSP viewer");

    //    vpNaoqiRobot robot;
    //    if (! opt_ip.empty()) {
    //      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    //      robot.setRobotIp(opt_ip);
    //    }

    //    robot.open();


    AL::ALColorBlobDetectionProxy color_proxy(opt_ip, 9559);

    color_proxy.subscribe("red_circle");
    color_proxy.setColor(255,0,0,10);

  //  AL::ALBarcodeReaderProxy m_barcode_proxy(opt_ip, 9559);

    AL::ALMemoryProxy m_memProxy(opt_ip, 9559);

//    // Start the barcode_detector
//    m_barcode_proxy.subscribe("barcode_detector");
//    m_barcode_proxy.setResolution(2);
//    m_barcode_proxy.setFrameRate(25);
//    std::cout << "Initialized ALBarcodeReaderProxy at framerate = " <<  m_barcode_proxy.getFrameRate() << " and resolution = " << m_barcode_proxy.getResolution() << std::endl;



    while (1)
    {

//      g.acquire(I);
//      vpDisplay::display(I);

      AL::ALValue result = m_memProxy.getData("ALTracker/ColorBlobDetected");

      if (result.getSize() > 0)
      {
        std::string name = result[0][0];

        std::cout << "Name: " << name << std::endl;

      }
      else
        std::cout << "No Red Blob Found" << std::endl;

       AL::ALValue cog = color_proxy.getCircle();
       std::cout << "cog" << cog[0] << std::endl;



//      vpDisplay::flush(I);
//      if (vpDisplay::getClick(I, false))
//        break;

    }


    color_proxy.unsubscribe("barcode_detector");

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

