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

#include <alproxies/almemoryproxy.h>
#include <qi/session.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anymodule.hpp>

#include <visp/vpPlot.h>
#include <visp/vpDisplayX.h>
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>


#include <visp_naoqi/vpNaoqiRobot.h>



const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H.%M.%S", &tstruct);

  return buf;
}


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

    qi::SessionPtr session = qi::makeSession();
    session->connect("tcp://131.254.10.126:9559");
    qi::AnyObject proxy = session->service("pepper_control");
    proxy.call<void >("start");


    std::string jointName;

    if (robot.getRobotType() == vpNaoqiRobot::Pepper)
      jointName = "HeadYaw";
    else if (robot.getRobotType() == vpNaoqiRobot::Romeo)
      jointName = "NeckYaw";
    else
    {
      std::cout << "Type of robot not valid" << std::endl;
      return 0;
    }

    bool servoing = true;

    // Define values
    float y = 1.0; //
    float l = 2.0; //
    float d = 0.13; //
    float x = 0.0;
    float L = 0.0;

    //float lambda = 0.03;
    float lambda = 0.08; // 0.050;

    float s_star = 1.0;

    vpColVector vel(1);

    AL::ALMemoryProxy memProxy(opt_ip, 9559);

    bool start = false;

    // Previuos ratio and velocity
    float ratio_prev = 0.;
    float vel_prev = 0.;
    float delta_t = 0.170;

    vpImage<unsigned char> I(320, 320);
    vpDisplayX dd(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    vpPlot plotter (2);
    plotter.initGraph(0, 2);
    plotter.initGraph(1, 1);
    //plotter_eyes.initGraph(2, 1);
    //plotter_eyes.initGraph(3, 1);
    plotter.setTitle(0,  "Ratio");
    plotter.setTitle(1,  "Ratio sound");
    //plotter_eyes.setTitle(2,  "REyeYaw");
    //plotter_eyes.setTitle(3,  "REyePitch");


    vpPlot plotter_e (1);
    plotter_e.initGraph(0, 2);

    plotter_e.setLegend(0, 0, "Right");
    plotter_e.setLegend(0, 1, "Left");

    plotter_e.setTitle(0,  "Energy");


    vpPlot plotter_vel (1);
    plotter_vel.initGraph(0, 1);
    plotter_vel.setTitle(0,  "HeadYaw");



    std::vector<float> vel_ (2);
    std::vector<std::string> names = robot.getBodyNames("Head");

    unsigned long loop_iter = 0;


    //    // Kalman filter
    //    vpLinearKalmanFilterInstantiation kalman;
    //    // Select a constant velocity state model with colored noise
    //    // Measures are velocities
    //    //kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel);
    //    // Initialise the filter for a one dimension signal
    //    int signal = 1;
    //    vpColVector sigma_state(2);   // State vector size is 2
    //    vpColVector sigma_measure(1); // Measure vector size is 1
    //    double rho = 0.9;
    //    double dt = 0.2; // non used parameter for the selected state model
    //    sigma_state[0] = 0.000000000001;//0.000001;
    //    sigma_measure[0] = 0.000000000001;//0.000001;
    //   // kalman.initFilter(signal, sigma_state, sigma_measure, rho, dt);
    //     kalman.initStateConstAccWithColoredNoise_MeasureVel(signal, sigma_state, sigma_measure, rho, dt);


   vpLinearKalmanFilterInstantiation kalman;

    double rho=0.3;
    vpColVector sigma_state;
    vpColVector sigma_measure(1);
    int signal = 1;

    kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstAccWithColoredNoise_MeasureVel);
    //kf.init(nbSrc,nbSrc,nbSrc);
    int state_size = kalman.getStateSize();
    sigma_state.resize(2);
    sigma_state= 0.00001; // Same variance for all the signals
    sigma_measure = 0.05; // Same measure variance for all the signals
    double dt = 0.172;
    kalman.initFilter(signal, sigma_state , sigma_measure, rho, dt );


    kalman.verbose(true);


    while (1)
    {
      std::cout << "----------------------------------" << std::endl;
      double t = vpTime::measureTimeMs();
      float ratio = memProxy.getData("ALSoundProcessing/ratioRightLeft");

      float sound_detected = memProxy.getData("ALSoundProcessing/soundDetectedEnergy");
      std::cout << "SoundDetected:" << sound_detected << std::endl;

      if ( (sound_detected > 0.5) && !start)
        start = true;

      if (start)
      {
        plotter_e.plot(0, 0,loop_iter, float( memProxy.getData("ALSoundProcessing/rightMicEnergy")));
        plotter_e.plot(0, 1,loop_iter, float(  memProxy.getData("ALSoundProcessing/leftMicEnergy")));


        if (!sound_detected )//|| ratio > 3.0)
        {
          ratio = 20 *L*vel_prev*delta_t + ratio_prev;
          //ratio = 1;

          plotter.plot(1, 0, loop_iter,0);

          // std::cout << "Ratio pred: " << ratio << std::endl;
        }
        else
        {
          plotter.plot(1, 0, loop_iter,ratio);

          std::cout << "Ratio:" << ratio << std::endl;
        }
        vpColVector ratio_v(1);
        ratio_v[0] = ratio;

        kalman.filter(ratio_v);
        std::cout << "Estimated x velocity: kalman.Xest[0]" <<  kalman.Xest[0]<< std::endl;
        plotter.plot(0, 0, loop_iter, ratio);
        //plotter.plot(0, 1, loop_iter, kalman.Xest[0]);
        //   ratio = kalman.Xest[0];
        ratio = (4*ratio + 3*kalman.Xest[0])/7;

        plotter.plot(0, 1, loop_iter,ratio);

        if (ratio >= 1.)
          x = 1;
        else
          x = -1;

        // Compute Interaction matrix
        L = (y * d * (ratio+1)) / (l*l + d*d/4 - d*x);

        //lambda = 0.0216404* log(1+abs(ratio - s_star));
        //Compute joint velocity NeckYaw
        vel[0] =  - lambda/L * (ratio - s_star);

        vel_[0] = vel[0];

        plotter_vel.plot(0, 0, loop_iter, vel[0]);


        std::cout << "vel: " << vel << std::endl;

        if (servoing)
        {
          if (robot.getRobotType() == vpNaoqiRobot::Pepper)
            proxy.async<void >("setDesJointVelocity", names, vel_ );
          else if (robot.getRobotType() == vpNaoqiRobot::Romeo)
            robot.setVelocity(jointName, vel);
        }
      }

      // Save current values
      ratio_prev = ratio;
      vel_prev = vel[0];

      loop_iter ++;

      vpTime::sleepMs(170);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

      if (vpDisplay::getClick(I, false))
        break;
    }


    //   plotter->saveData(0, "ratio.dat");


    if (robot.getRobotType() == vpNaoqiRobot::Pepper)
    {
      proxy.call<void>("stopJoint");

      proxy.call<void>("stop");
    }
    else if (robot.getRobotType() == vpNaoqiRobot::Romeo)
      robot.stop(jointName);

    vpDisplay::getClick(I, true);
    plotter.saveData(0, "ratio.dat");
    plotter_vel.saveData(0, "vel.dat");

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

