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


#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpFaceTrackerOkao.h>





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

    vpFaceTrackerOkao face_tracker(opt_ip,9559);


    // Open Proxy for the speech
    AL::ALTextToSpeechProxy tts(opt_ip, 9559);
    tts.setLanguage("English");

    std::string phraseToSay = "Hi!";
    //    int id = tts.post.say(phraseToSay);
    //    tts.wait(id,2000);

    while (1)
    {
      g.acquire(I);
      vpDisplay::display(I);

      bool result = face_tracker.detect();

      if (result)
      {
        std::ostringstream text;
        text << "Found " << face_tracker.getNbObjects() << " face(s)";
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        for(size_t i=0; i < face_tracker.getNbObjects(); i++) {
          std::vector<vpImagePoint> p = face_tracker.getPolygon(i);
          vpRect bbox = face_tracker.getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 1);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(), face_tracker.getMessage(i) , vpColor::red);
        }
      }

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
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



  return 0;
}

