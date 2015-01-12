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

/*! \example speech_recognition.cpp */
#include <iostream>
#include <string>

#include <alproxies/alspeechrecognitionproxy.h>
#include <alproxies/almemoryproxy.h>

#include <visp_naoqi/vpNaoqiRobot.h>


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
    std::string opt_ip = "198.18.0.1";;

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
    }

    //    vpNaoqiRobot robot;
    //    if (! opt_ip.empty()) {
    //      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    //      robot.setRobotIp(opt_ip);
    //    }

    //    robot.open();


    // Open Proxy for the speech
    AL::ALSpeechRecognitionProxy asr(opt_ip, 9559);
    asr.setLanguage("English");
    std::vector<std::string> vocabulary;
    vocabulary.push_back("yes");
    vocabulary.push_back("no");

    // Set the vocabulary
    asr.setVocabulary(vocabulary,false);


    AL::ALMemoryProxy memProxy(argv[1], 9559);

    // Start the speech recognition engine with user Test_ASR
    asr.subscribe("Test_ASR");
    std::cout << "Speech recognition engine started" << std::endl;

    vpTime::sleepMs(10000);

    asr.unsubscribe("Test_ASR");


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

