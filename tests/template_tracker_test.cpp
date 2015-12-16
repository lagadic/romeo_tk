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
 * to display them on your screen using ViSP.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example template_tracker_test.cpp */
#include <iostream>
#include <string>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImage.h>
#include <visp3/ar/vpAROgre.h>


#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiConfig.h>
#include <vpRomeoTkConfig.h>


#include <vpTemplateLocatization.h>


/*!

   Connect to Nao or Romeo robot, grab and display images using ViSP.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./image_viewer_visp -ip <robot ip address>

   Example:

   ./image_viewer_visp -ip 169.254.168.230

 */


/*!
  Check the validity of the pose of the box.
*/

bool checkValiditycMo(vpHomogeneousMatrix cMo)
{

  double x = cMo[0][3];
  double y = cMo[1][3];
  double z = cMo[2][3];

  //std::cout << "x: " << x <<". Limits: -0.40 > y > 0.40" << std::endl;
  //std::cout << "y: " << y <<". Limits: -0.50 > y > 0.50" << std::endl;
  //std::cout << "z: " << z <<". Limits: 0.10 > y > 0.40" << std::endl;
  if (z < 0.10 || z > 0.40
      || x < - 0.20 || x > 0.10
      || y < -0.10 || y > 0.10 )
    return false;
  else
    return true;
}

void printPose(const std::string &text, const vpHomogeneousMatrix &cMo)
{
    vpTranslationVector t;
    cMo.extract(t);
    vpRotationMatrix R;
    cMo.extract(R);
    vpThetaUVector tu(R);

    std::cout << text;
    for (unsigned int i=0; i < 3; i++)
        std::cout << t[i] << " ";
    for (unsigned int i=0; i < 3; i++)
        std::cout << vpMath::deg(tu[i]) << " ";
    std::cout << std::endl;
}

int main(int argc, const char* argv[])
{
  try
  {
    std::string opt_ip = "198.18.0.1";
    int opt_cam = 0;
    std::string opt_box_name = "star_wars_pic";
    std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);

  for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--box-name")
        opt_box_name = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--data-folder")
        opt_data_folder = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--box-name] [--data-folder]" << std::endl;

        return 0;
      }
    }

    vpNaoqiGrabber g;
    g.setCamera(opt_cam); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    g.open();
    std::cout << "Open camera parameters: " << g.getCameraParameters() << std::endl;
    std::cout << "Dimension image: " << g.getHeight() <<"x" << g.getWidth() << std::endl;


    vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithoutDistortion);
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;




    std::string objects_folder = "objects/" + opt_box_name  + "/";
    std::string box_folder = opt_data_folder +"/" + objects_folder;
    std::string config_detection_file_folder = box_folder + "detection/";
    std::string objects_folder_det_learning = config_detection_file_folder + "learning/20/";
    std::string opt_model = box_folder + "model/" + opt_box_name;
    std::string learning_detection_file = "learning_data.bin";
    std::string learning_data_file_name = objects_folder_det_learning + learning_detection_file;

    std::cout << learning_data_file_name << std::endl;




    // Initialize the template tracker
    bool status_template_tracker;
    vpHomogeneousMatrix cMo_t;
    vpTemplateLocatization t_tracker(opt_model, config_detection_file_folder, cam);
    t_tracker.setTemplateSize(0.175,0.12);
    t_tracker.initDetection(learning_data_file_name);
    t_tracker.setValiditycMoFunction(checkValiditycMo);
    bool onlyDetection = true;
    t_tracker.setOnlyDetection(onlyDetection);

    // Create a vpRAOgre object with color background
    vpAROgre ogre(cam, g.getWidth(), g.getHeight());
    // Initialize it
    ogre.init(I);
    ogre.load("Robot", "robot.mesh");
    ogre.setScale("Robot", 0.001f,0.001f,0.001f);
    ogre.setRotation("Robot", vpRotationMatrix(vpRxyzVector(M_PI/2, -M_PI/2, M_PI)));
   // ogre.setRotation("Robot", vpRotationMatrix(vpRxyzVector(0, 0, 0)));

    // Add an optional point light source
    Ogre::Light * light = ogre.getSceneManager()->createLight();
    light->setDiffuseColour(1, 1, 1); // scaled RGB values
    light->setSpecularColour(1, 1, 1); // scaled RGB values
    light->setPosition(-5, -5, 10);
    light->setType(Ogre::Light::LT_POINT);


    t_tracker.getCameraParameters().printParameters();

    vpDisplay::getClick(I,true);



    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      // track qrcode
      status_template_tracker = t_tracker.track(I);



      if (status_template_tracker ) { // display the tracking results
          cMo_t = t_tracker.get_cMo();
          printPose("cMo qrcode: ", cMo_t);
          vpDisplay::displayFrame(I, cMo_t, cam, 0.04, vpColor::none, 3);
          vpDisplay::displayPolygon(I, t_tracker.getCorners(), vpColor::green, 2);

          // Display with ogre
          ogre.display(I,cMo_t);

      }


        vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

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


