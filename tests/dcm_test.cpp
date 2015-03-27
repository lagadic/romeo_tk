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

#include <alproxies/dcmproxy.h>
#include <alcommon/albroker.h>

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

    // Create a general proxy to motion to use new functions not implemented in the specialized proxy

    std::string 	myIP = "";		// IP du portable (voir /etc/hosts)
    int 	myPort = 0 ;			// Default broker port

    //    AL::ALProxy *m_proxy;
    boost::shared_ptr<AL::ALBroker> broker = AL::ALBroker::createBroker("Broker", myIP, myPort, opt_ip, 9559);
    //    m_proxy = new AL::ALProxy(broker, "ALVideoDevice");

    //    m_proxy->callVoid("setCameraGroup",1 ,true);

    //AL::ALProxy* dcm = new AL::ALProxy(broker, "DCM_video");



    boost::shared_ptr<AL::ALProxy> proxy = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker, "DCM_video"));
    AL::DCMProxy* dcm = new AL::DCMProxy(proxy);

    //boost::shared_ptr<AL::ALProxy> dcm = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker, "DCM_video"));



    AL::ALValue commands;
    commands.arraySetSize(3);
    commands[0] = std::string("FaceBoard/CameraSwitch/Value");
    commands[1] = std::string("Merge");
    commands[2].arraySetSize(1);
    commands[2][0].arraySetSize(2);
    commands[2][0][0] = 1;
    //commands[2][0][1] = dcm->call<int>("getTime",0);// dcm->getTime(0);
    commands[2][0][1] = dcm->getTime(0);
    //dcm->callVoid("set",commands);
    dcm->set(commands);


    //int time = dcm->call<int>("getTime",0);
    int time = dcm->getTime(10);
    std::cout <<"Time " << time << std::endl;

    //dcm = naoqitools.myGetProxy( "DCM_video" );^M
    //dcm.set( ["FaceBoard/CameraSwitch/Value", "Merge",  [[rGroupNumber, dcm.getTime( 0 ) ]] ] );^M
    //dcm.set(\["ChestBoard/Led/Red/Actuator/Value", "Merge", \[[1.0, dcm.getTime(10000)]] ])




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

