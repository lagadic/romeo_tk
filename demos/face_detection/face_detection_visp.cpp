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
 * to detect a face.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example face_detection_visp.cpp */

/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to detect a face.
 *
 */

#include <iostream>
#include <string>

#include <visp_naoqi/vpNaoqiGrabber.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerWarpSRT.h>

#include <vpFaceTracker.h>

/*!

  Connect to Nao or Romeo robot, grab, display images using ViSP and start
  face detection with OpenCV and tracking with ViSP when the detection fails.
  By default, this example connect to a robot with ip address: 198.18.0.1.
  If you want to connect on an other robot, run:

  ./face_detection_visp -ip <robot ip address> -haar <haar cascade .xml file>

  Example:

  ./face_detection_visp -ip 169.254.168.230 -haar ./haarcascade_frontalface_alt.xml
 */

int main(int argc, const char* argv[])
{
  try {
    std::string opt_ip = "198.18.0.1";
    std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";

    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "--haar")
        opt_face_cascade_name = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--ip <robot address>] [--haar <haarcascade xml filename>] [--help]" << std::endl;
        return 0;
      }
    }

    vpNaoqiGrabber g;
    if (! opt_ip.empty())
      g.setRobotIp(opt_ip);
    g.open();

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    vpFaceTracker face_tracker;
    face_tracker.setFaceCascade(opt_face_cascade_name);

    while(1) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      bool face_found = face_tracker.track(I);
      if (face_found)
        vpDisplay::displayRectangle(I, face_tracker.getFace(), vpColor::red, false, 4);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
}
