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

#include <visp/vpImagePoint.h>


#include <alproxies/almemoryproxy.h>
#include <alproxies/alpeopleperceptionproxy.h>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include <almath/types/alrotation3d.h>
#include <almath/types/alposition6d.h>
#include <almath/types/alposition3d.h>
#include <almath/types/altransform.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>


/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion --ip <robot ip address>

   Example:

   ./motion --ip 169.254.168.230
 */

#include <sstream>

namespace patch
{
template < typename T > std::string to_string( const T& n )
{
  std::ostringstream stm ;
  stm << n ;
  return stm.str() ;
}
}




int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip = "131.254.10.126";

    if (argc == 3) {
      if (std::string(argv[1]) == "--ip")
        opt_ip = argv[2];
    }


    vpNaoqiGrabber g;
    if (! opt_ip.empty())
      g.setRobotIp(opt_ip);
    g.open();
    std::string camera_name = "CameraTopPepper";
    vpCameraParameters cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA,camera_name, vpCameraParameters::perspectiveProjWithDistortion);
    vpHomogeneousMatrix eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name,vpCameraParameters::perspectiveProjWithDistortion);

    std::cout << "cam:" << std::endl << cam << std::endl;

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    //    vpNaoqiRobot robot;
    //    if (! opt_ip.empty()) {
    //      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
    //      robot.setRobotIp(opt_ip);
    //    }

    //    robot.open();


    AL::ALPeoplePerceptionProxy people_proxy(opt_ip, 9559);
    AL::ALMemoryProxy m_memProxy(opt_ip, 9559);
    people_proxy.subscribe("People", 1000, 0.0);
    std::cout << "period: " << people_proxy.getCurrentPeriod() << std::endl;

    float  mImageHeight = g.getHeight();
    float  mImageWidth = g.getWidth();

    while (1)
    {

      g.acquire(I);
      vpDisplay::display(I);


      AL::ALValue result = m_memProxy.getData("PeoplePerception/VisiblePeopleList");


      if (result.getSize() > 0)
      {
        int id = result[0];

        std::cout << "Id: " << id << std::endl;

        AL::ALValue info = m_memProxy.getData("PeoplePerception/PeopleDetected");
        std::cout << "NumPeople: " << info[1].getSize() << std::endl;

        // std::cout << "Id: " << info[1][0][0] << std::endl;
        std::cout << "DistanceToCamera: " << info[1][0][1] << std::endl;
        std::cout << "PitchAngleInImage: " << info[1][0][2] << std::endl;
        std::cout << "YawAngleInImage: " << info[1][0][3] << std::endl;

        float alpha =  info[1][0][2];
        float beta =  info[1][0][3];
        // Centre of face into the image
        float x = cam.get_u0()  - mImageWidth * beta;
        float y = cam.get_v0()  + mImageHeight * alpha;

        vpDisplay::displayCross(I, y, x, 10, vpColor::red);

        std::string  pos_key = "PeoplePerception/Person/" + patch::to_string(id) + "/PositionInTorsoFrame" ;
        AL::ALValue position = m_memProxy.getData(pos_key.c_str());

        vpPoint pose_person(position[0], position[1], position[2] ) ;

        //        std::vector<float> cMt =  info[2][0];

        std::cout << info[2][0] << " " << info[2][1] << " " << info[2][2] << std::endl;

        //        std::vector <float> poseCam_ (6);
        //        for (unsigned int i = 0; i < poseCam_.size(); i++)
        //          poseCam_[i] = info[2][i];

        //        AL::Math::Position3D poseP =  AL::Math::Position3D(position[0],position[1],position[2]) ;
        //        AL::Math::Position6D poseCam =  AL::Math::Position6D(poseCam_) ;
        //        AL::Math::Position3D poseP_c;
        //        AL::Math::changeReferencePosition3D	(	poseCam,	poseP, poseP_c );

        AL::Math::Transform tMc_AL = AL::Math::Transform::fromPosition(info[2][0], info[2][1], info[2][2], info[2][3], info[2][4], info[2][5]);
        std::vector <float> tMc_(16);
        tMc_AL.toVector(tMc_);
        vpHomogeneousMatrix tMc_al;
        unsigned int count = 0;
        for (unsigned int i = 0; i < 4; i++)
          for (unsigned int j = 0; j < 4; j++)
          {
            tMc_al[i][j] = tMc_[count];
            count++;
          }

        std::cout << "tMc_al" << std::endl << tMc_al << std::endl;
        vpHomogeneousMatrix aMv; // From aldebaran to visp frame convention
        vpHomogeneousMatrix tMc;
        for(unsigned int i=0; i<3; i++)
          aMv[i][i] = 0; // remove identity
        //Set Rotation
        aMv[0][2] = 1.;
        aMv[1][0] = -1.;
        aMv[2][1] = -1.;
        //Set Translation:
        aMv[0][3] = 0. ;
        aMv[1][3] = 0.;
        aMv[2][3] = 0.;

        tMc = tMc_al * aMv;

        std::cout << "tMc" << std::endl << tMc << std::endl;

        pose_person.changeFrame(tMc.inverse()) ; // set A.cP, the 3D point coordinates in the camera frame.
        pose_person.project() ; // set A.p, the point 2D homogeneous coordinates in the image plane.

        pose_person.display(I,cam,vpColor::blue) ;
        std::cout << "pose_person" << std::endl << pose_person << std::endl;

        //       vpPoint pose_person_visp = tMc.inverse() * pose_person;
        //       std::cout << "pose_person_visp" << std::endl << pose_person_visp << std::endl;


        //        poseCam.x = info[2][0] ; poseCam.y =  info[2][1]; poseCam.z = info[2][2] ;
        //        poseCam.wx =  info[2][3]; poseCam.wy =  info[2][4]; poseCam.wz =  info[2][5];
        // AL::Math::transformFromPosition6D()


        //        vpPoseVector handMbox_desired_ (handMbox_desired);
        //        AL::ALValue pos3;
        //        pos3.arraySetSize(6);

        //        // Transform in Yall Pitch and Yaw
        //        AL::Math::Rotation3D rot;
        //        AL::Math::Transform transform(handMbox_desired);
        //        rot = AL::Math::rotation3DFromTransform(transform);


      }





      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;

    }

    people_proxy.unsubscribe("People");

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

