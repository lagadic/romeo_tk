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
 * This example demonstrates how get the joint velocities.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example speech_recognition.cpp */
#include <iostream>
#include <string>

#include <alproxies/alspeechrecognitionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>

#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp/vpPlot.h>


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

        vpNaoqiRobot robot;
        if (! opt_ip.empty()) {
            std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
            robot.setRobotIp(opt_ip);
        }

        robot.open();

        // Head
        std::vector <std::string > names_head = robot.getBodyNames("Head");

        vpPlot plotter_head (4);
        plotter_head.initGraph(0, 1);
        plotter_head.initGraph(1, 1);
        plotter_head.initGraph(2, 1);
        plotter_head.initGraph(3, 1);
        plotter_head.setTitle(0,  "NeckYaw");
        plotter_head.setTitle(1,  "NeckPitch");
        plotter_head.setTitle(2,  "HeadPitch");
        plotter_head.setTitle(3,  "HeadRoll");

        //        // Eyes -- NOT WORKING

        //        std::vector<std::string> names_eyes = robot.getBodyNames("LEye");
        //        std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");
        //        names_eyes.insert(names_eyes.end(),jointNamesREye.begin(), jointNamesREye.end() );

        //        vpPlot plotter_eyes (4);
        //        plotter_eyes.initGraph(0, 1);
        //        plotter_eyes.initGraph(1, 1);
        //        plotter_eyes.initGraph(2, 1);
        //        plotter_eyes.initGraph(3, 1);
        //        plotter_eyes.setTitle(0,  "LEyeYaw");
        //        plotter_eyes.setTitle(1,  "LEyePitch");
        //        plotter_eyes.setTitle(2,  "REyeYaw");
        //        plotter_eyes.setTitle(3,  "REyePitch");



        // LEFT ARM
        std::vector <std::string > names_larm = robot.getBodyNames("LArm");
        names_larm.pop_back(); // We don't consider the last joint of the hand (open/close)

        vpPlot plotter_larm (4);
        plotter_larm.initGraph(0, 1);
        plotter_larm.initGraph(1, 1);
        plotter_larm.initGraph(2, 1);
        plotter_larm.initGraph(3, 1);

        vpPlot plotter_larm_a (3);
        plotter_larm_a.initGraph(0, 1);
        plotter_larm_a.initGraph(1, 1);
        plotter_larm_a.initGraph(2, 1);


        plotter_larm.setTitle(0,  "LshouderPitch");
        plotter_larm.setTitle(1,  "LShoulderYaw");
        plotter_larm.setTitle(2,  "LElbowRoll");
        plotter_larm.setTitle(3,  "LElbowYaw");
        plotter_larm_a.setTitle(0,  "LWristRoll");
        plotter_larm_a.setTitle(1,  "LWristYaw");
        plotter_larm_a.setTitle(2,  "LWristPitch");

        //        AL::ALMemoryProxy proxy(opt_ip, 9559);
        //        std::vector <std::string> lista =  proxy.getDataList("Speed");
        //        std::cout << lista << std::endl;

        unsigned long loop_iter = 0;


        while (1)
        {
            //      float result = memProxy.getData("Device/SubDeviceList/HeadPitch/Speed/Actuator/Value");
            //      std::cout <<"VEL: " << result << std::endl;


            vpColVector vel_head = robot.getJointVelocity(names_head);

            //      std::cout <<"getVel: " << vel << std::endl;
            //      std::cout <<"__________________________: " << result << std::endl;

            for (unsigned int i=0 ; i < names_head.size() ; i++) {
                plotter_head.plot(i,0,loop_iter,vel_head[i]);
            }

            //            vpColVector vel_eyes = robot.getJointVelocity(names_eyes);

            //            for (unsigned int i=0 ; i < names_eyes.size() ; i++) {
            //                plotter_eyes.plot(i,0,loop_iter,vel_eyes[i]);
            //            }

            vpColVector vel_larm = robot.getJointVelocity(names_larm);

            for (unsigned int i=0 ; i < 4 ; i++) {

                plotter_larm.plot(i,0,loop_iter,vel_larm[i]);
                if (i < 3)
                    plotter_larm_a.plot(i, 0, loop_iter, vel_larm[i + 4]);
            }

            loop_iter ++;
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

