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


    std::vector<std::string> names = robot.getBodyNames("Body");
    std::vector<float> values(names.size(),0.0);
    robot.getProxy()->setMoveArmsEnabled(false,false);
    robot.getPosition(names,values,false);


    bool servoing = true;

    //    bool servoing = false;

    // Define values
    float y = 1.0; //
    float ell1 = 2.0; //
    float ell2 = 2.0; //
    float ell = 2.0;
    float d = 0.7; //
    float x = 0.0;
    vpMatrix L(2,6);

    L = 0.0;

    //float lambda = 0.03;
    float lambda = 0.09;//0.09; // 0.050;

    vpColVector s(2);
    vpColVector s_star(2);
    s_star[0] = 1.;
    s_star[1] = 13;// 2.5;
    vpColVector vel(6);
    vpColVector vel_(3);
    //    vpMatrix J (6,6);

    //    J =0.0;

    //    j[]


    AL::ALMemoryProxy memProxy(opt_ip, 9559);

    bool start = false;


    vpImage<unsigned char> I(320, 320);
    vpDisplayX dd(I);
    vpDisplay::setTitle(I, "ViSP viewer");


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



    vpLinearKalmanFilterInstantiation kalman1;


    kalman1.setStateModel(vpLinearKalmanFilterInstantiation::stateConstAccWithColoredNoise_MeasureVel);
    //kf.init(nbSrc,nbSrc,nbSrc);
    sigma_state.resize(2);
    sigma_state= 0.00001; // Same variance for all the signals
    sigma_measure = 0.05; // Same measure variance for all the signals
    kalman1.initFilter(signal, sigma_state , sigma_measure, rho, dt );
    kalman1.verbose(true);


    vpPlot plotter (2);
    plotter.initGraph(0, 1);
    plotter.initGraph(1, 2);
    //plotter_eyes.initGraph(2, 1);
    //plotter_eyes.initGraph(3, 1);
    plotter.setTitle(0,  "Ratio");
    plotter.setTitle(1,  "Error");
    //plotter_eyes.setTitle(2,  "REyeYaw");
    //plotter_eyes.setTitle(3,  "REyePitch");

    vpPlot plotter_kal (2);
    plotter_kal.initGraph(0, 2);
    plotter_kal.initGraph(1, 2);
    //plotter_eyes.initGraph(2, 1);
    //plotter_eyes.initGraph(3, 1);
    plotter_kal.setTitle(0,  "Ratio");
    plotter_kal.setTitle(1,  "Em");
    plotter_kal.setLegend(0, 0, "Ratio");
    plotter_kal.setLegend(0, 1, "Ratio kal");
    plotter_kal.setLegend(1, 0, "Em");
    plotter_kal.setLegend(1, 1, "Em kal");


    //    vpPlot plotter_e (1);
    //    plotter_e.initGraph(0, 2);

    //    plotter_e.setLegend(0, 0, "Right");
    //    plotter_e.setLegend(0, 1, "Left");

    //    plotter_e.setTitle(0,  "Energy");


    vpPlot plotter_vel (1);
    plotter_vel.initGraph(0, 3);
    plotter_vel.setTitle(0,  "Velocity");
    plotter_vel.setLegend(0, 0, "vx");
    plotter_vel.setLegend(0, 1, "vy");
    plotter_vel.setLegend(0, 2, "wz");

    unsigned long loop_iter = 0;

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

        float E1 = memProxy.getData("ALSoundProcessing/rightMicEnergy");
        float E2= memProxy.getData("ALSoundProcessing/leftMicEnergy");

        E1 = E1/1e9;
        E2 = E2/1e9;

        //        plotter_e.plot(0, 0,loop_iter, float( memProxy.getData("ALSoundProcessing/rightMicEnergy")));
        //        plotter_e.plot(0, 1,loop_iter, float(  memProxy.getData("ALSoundProcessing/leftMicEnergy")));




        //  plotter.plot(0, 1, loop_iter,ratio);

        if (ratio >= 1.)
          x = 1;
        else
          x = -1;


        float c_x=d/2.*(E1+E2)/(E1-E2);
        float c_r=std::abs((d*(sqrt(E1*E2)/(E1-E2)))) ;
        ell=2.;//sqrt(sqrt(sqr(d*c_x))-d*d/4.+1);
        ell1=sqrt((c_x-d/2)*(c_x-d/2) + c_r * c_r);
        ell2=sqrt((c_x+d/2)*(c_x+d/2) + c_r * c_r);
        //float Em=4.*E1*ell1*ell1/(2*ell1*ell1+2*ell2*ell2-d*d);
        float Em=(E1+E2)/2;
        c_x=c_x/abs(c_x);
        //x=c_x;
        if (ratio >= 1.)
          x = 1;
        else
          x = -1;
        y=1;//c_r;
        std::cout<<"ell:"<<ell<<std::endl;
        std::cout<<"Em:"<<Em<<std::endl;


        vpColVector ratio_v(1);
        ratio_v[0] = ratio;
        vpColVector Em_v(1);
        Em_v[0] = Em;
        //ratio_v[1] = Em;
        kalman.filter(ratio_v);
        kalman1.filter(Em_v);
        std::cout << "Estimated x velocity: kalman.Xest[0]" <<  kalman.Xest[0]<< std::endl;
        std::cout << "Estimated x velocity: kalman.Xest[1]" <<  kalman.Xest[1]<< std::endl;

        plotter_kal.plot(0, 0, loop_iter, ratio);
        plotter_kal.plot(1, 0, loop_iter, Em);

        ratio = (4*ratio + 3*kalman.Xest[0])/7;
        Em = (4*Em + 3*kalman1.Xest[0])/7;

        plotter_kal.plot(0, 1, loop_iter, ratio);
        plotter_kal.plot(1, 1, loop_iter, Em);

        // Compute Interaction matrix
        L[0][0]= (2*x*(ratio-1)-d*(1+ratio))/(ell*ell-d*x+d*d/4.);
        L[0][1]=(2*y*(ratio-1))/(ell*ell-d*x+d*d/4.);
        L[0][5]=(y*d*(ratio+1))/(ell*ell-d*x+d*d/4.);
        L[1][0]= (2.*x*Em)/(ell*ell);
        L[1][1]= (2.*y*Em)/(ell*ell);

        s[0] = ratio;
        s[1] = Em;

        std::cout << "L: " << L << std::endl;

        //lambda = 0.0216404* log(1+abs(ratio - s_star));
        //Compute joint velocity NeckYaw
        vpColVector e = (s-s_star);
        vel =-lambda*L.pseudoInverse()*e;/* Compute V_m*/


        std::cout << "e :" << e << std::endl;
        vel_[0] = vel[1];
        vel_[1] = -vel[0];
        vel_[2] = vel[5];

        plotter_vel.plot(0, 0, loop_iter, vel_[0]);
        plotter_vel.plot(0, 1, loop_iter, vel_[1]);
        plotter_vel.plot(0, 2, loop_iter, vel_[2]);


        plotter.plot(0, 0, loop_iter,ratio);
        plotter.plot(1, 0, loop_iter,e[0]);
        plotter.plot(1, 1, loop_iter,e[1]);


        std::cout << "Em: " << Em << std::endl;
        std::cout << "vel: " << vel << std::endl;

        if (servoing)
        {
          robot.getProxy()->move(vel_[0],vel_[1],vel_[2]);
          robot.getProxy()->setAngles(names,values,1.0);
        }
      }

      // Save current values

      loop_iter ++;

      vpTime::sleepMs(170);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

      if (vpDisplay::getClick(I, false))
        break;
    }

    robot.getProxy()->move(0.0,0.0,0.0);
    //   plotter->saveData(0, "ratio.dat");



    vpDisplay::getClick(I, true);
    plotter_kal.saveData(0, "ratio.dat");
    plotter_kal.saveData(1,"em.dat");
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

