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

/*! \example .cpp */
#include <iostream>
#include <string>

#include <vpPepperFollowPeople.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <visp/vpTime.h>




int main(int argc, const char* argv[])
{

  std::string opt_ip = "131.254.64.27";

  if (argc == 3) {
    if (std::string(argv[1]) == "--ip")
      opt_ip = argv[2];
  }

  vpNaoqiRobot robot;
  // Connect to the robot
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();
  

  vpPepperFollowPeople task(opt_ip, 9559, robot);
  double dist = 1.0;
  task.setDesiredDistance(dist);
  task.setReverse(false);

  double time = vpTime::measureTimeSecond();

  while(vpTime::measureTimeSecond() - time < 30.0)
  {

    double t = vpTime::measureTimeMs();

    task.computeAndApplyServo();

    std::cout << "Loop TOTAL time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    std::cout << "******************************************************" << std::endl;

  }

  task.stop();

  return 0;
}

