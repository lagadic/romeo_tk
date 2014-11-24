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
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using ViSP.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example data-acquisition.cpp */
#include <iostream>
#include <string>
#include <ostream>
#include <sstream>
#include <iomanip>

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

/*!

   Connect to Nao or Romeo robot, grab images and save the transformation
   between the torso and the end-effector frame that supports the camera.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./data-acquisition -ip <robot ip address> -outpath <data output path>

   Example:

   ./data-acquisition -ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip;
    std::string opt_outpath = "./";
    std::string filename;

    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "-ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "-outpath")
        opt_outpath = argv[i+1];
    }

    vpNaoqiGrabber g;
    g.setCamera(0); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }
    g.open();

    std::cout << "Camera parameters: " << g.getCameraParameters() << std::endl;

    vpNaoqiRobot robot;
    if (! opt_ip.empty())
      robot.setRobotIp(opt_ip);
    robot.open();

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;

    unsigned int cpt_data = 1;
    vpMouseButton::vpMouseButtonType button;

    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      std::ostringstream text;
      text << "Left click to acquire data " << cpt_data << ", right click to exit...";
      vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);

      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button3)
          break;
        else if(button == vpMouseButton::button1) {
          {
            std::ostringstream s;
            s.setf(std::ios::right, std::ios::adjustfield);
            s << "I" << std::setw(4) << std::setfill('0') << cpt_data << ".pgm";
            filename = vpIoTools::createFilePath(opt_outpath, s.str());
            std::cout << "Save image: " << filename << std::endl;
            vpImageIo::write(I, filename);
          }
          {
            // Get the transformation from torso to the head end effector
            vpHomogeneousMatrix tMe(robot.getProxy()->getTransform("HeadRoll", 0, true));
            vpXmlParserHomogeneousMatrix p;

            std::ostringstream s;
            s << "M" << std::setw(4) << std::setfill('0') << cpt_data << ".xml";
            filename = vpIoTools::createFilePath(opt_outpath, s.str());

            std::cout << "Save tMe: " << filename << std::endl;
            if (p.save(tMe, filename, "tMe") != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
              std::cout << "Cannot save the homogeneous matrix" << std::endl;
            }
          }
          cpt_data ++;
        }
      }

      //std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
      vpDisplay::flush(I);
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

  vpXmlParser::cleanup();

  return 0;
}

