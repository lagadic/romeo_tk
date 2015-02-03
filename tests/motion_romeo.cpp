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
 * Fabien Spindler
 *
 *****************************************************************************/

/*! \example motion_romeo.cpp */
#include <iostream>
#include <string>

#include <visp/vpMath.h>
#include <visp/vpTime.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpRomeoTkConfig.h>


void moveArmCartesianPosition(/*const*/ vpNaoqiRobot &robot, const vpColVector &cart_delta_pos,
                           const std::string &end_effector_name, float delta_t, vpVelocityTwistMatrix oVe)
{
  // compute the time
  vpHomogeneousMatrix M;
  vpRxyzVector rxyz;

  for (unsigned int i=0; i< 3; i++) {
    M[i][3] = cart_delta_pos[i];
    rxyz[i] = cart_delta_pos[i+3];
  }
  vpRotationMatrix R(rxyz);
  M.insert(R);

  std::vector<std::string> jointNames =  robot.getBodyNames(end_effector_name);
  jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo

  vpColVector v_o = vpExponentialMap::inverse(M, delta_t);
  vpMatrix oJo;

  double t_initial = vpTime::measureTimeSecond();
  while (vpTime::measureTimeSecond() < t_initial+delta_t)
  {
    oJo = oVe * robot.get_eJe(end_effector_name);

    vpColVector q_dot = oJo.pseudoInverse() * v_o;

    std::cout << "q_dot: " << q_dot.t() << std::endl;

    robot.setVelocity(jointNames, q_dot);
  }

  robot.stop(jointNames);
}
/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion -ip <robot ip address>

   Example:

   ./motion -ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip;

    if (argc == 3) {
      if (std::string(argv[1]) == "-ip")
        opt_ip = argv[2];
    }

    vpNaoqiRobot robot;
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      robot.setRobotIp(opt_ip);
    }

    robot.open();


    vpColVector cart_delta_pos(6);
    cart_delta_pos = 0;
    cart_delta_pos[2] = 0.03;

    vpHomogeneousMatrix oMe_LArm;

    std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
    //std::string name_transform = "qrcode_M_e_LArm";

    std::string name_transform = "qrcode_M_e_LArm_bakfrom2015_01_26";
     {
      vpXmlParserHomogeneousMatrix pm; // Create a XML parser

      if (pm.parse(oMe_LArm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
        return 0;
      }
      else
        std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_LArm << std::endl;
    }






//    for(unsigned int i=0; i<3; i++)
//      oMe_LArm[i][i] = 0; // remove identity
//    oMe_LArm[0][2] =  1;
//    oMe_LArm[1][0] = -1;
//    oMe_LArm[2][1] = -1;

//    oMe_LArm[0][3] = -0.04;
//    oMe_LArm[1][3] =  0.045;
//    oMe_LArm[2][3] = -0.045;

    vpVelocityTwistMatrix oVe_LArm(oMe_LArm);



    moveArmCartesianPosition(robot, cart_delta_pos, "LArm", 5.0, oVe_LArm);

    return 0;



    vpNaoqiGrabber g;
    g.setCamera(0); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    g.open();
    std::cout << "Camera parameters: " << g.getCameraParameters() << std::endl;

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;



    // Test with a vector of joints
    std::vector<std::string> jointNames;
    jointNames.push_back("NeckYaw");
    jointNames.push_back("NeckPitch");

    std::cout << "Test " << jointNames << " velocity control" << std::endl;

    vpColVector jointVel( jointNames.size() );
    for (unsigned int i=0; i < jointVel.size(); i++)
      jointVel[i] = vpMath::rad(2);

    robot.setStiffness(jointNames, 1.f);

    std::string nameChain_larm = "LArm";

    std::vector<float> handPos = robot.getProxy()->getPosition(nameChain_larm, 0, false);
    handPos[2] =  handPos[2] + 0.10;
    robot.getProxy()->post.setPositions(nameChain_larm, 0, handPos, 0.2, 7);


    //while(1)
    {
      double t_initial = vpTime::measureTimeSecond();
      g.acquire(I);
      vpDisplay::display(I);




      //if(vpTime::measureTimeSecond() < t_initial+3)
        for(int i=0; i<10; i++)
      {
        robot.setVelocity(jointNames, jointVel);
        usleep(100000);
      }

    //  robot.getProxy()->setPositions(nameChain_larm, 0, handPos, 0.2, 7);

      vpDisplay::flush(I);
//      if (vpDisplay::getClick(I, false))
//        break;

    }

//    robot.stop(jointNames);

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

