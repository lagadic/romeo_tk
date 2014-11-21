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
 * to detect a face.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to detect a face.
 *
 */


#include <iostream>
#include <string>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerWarpSRT.h>

#include <visp_naoqi/vpNaoqiGrabber.h>

typedef enum {
  detection,
  init_tracking,
  tracking,
  none
} state_t;



int main(int argc, char* argv[])
{
  //-- 1. Load the cascades
  cv::CascadeClassifier face_cascade;
  /** Global variables */
  cv::String face_cascade_name = "/local/soft/romeo/cpp/workspace/romeo_face_detection/romeo_face_detection/haarcascade_frontalface_alt.xml";

  if (argc == 2)
  {
    face_cascade_name = cv::String(argv[1]);
  }

  if( !face_cascade.load( face_cascade_name ) ) {
    std::cout << "--(!)Error loading default " << face_cascade_name << std::endl;
    std::cout << "Usage : " << argv[0] << " <haarcascade_file.xml>" << std::endl;
    return -1;
  };

  vpNaoqiGrabber g;
  g.setFramerate(15);
  g.setCamera(0);
  g.open();

  vpTemplateTrackerWarpSRT warp;
  vpTemplateTrackerSSDInverseCompositional tracker(&warp);
  tracker.setSampling(2,2);
  tracker.setLambda(0.001);
  tracker.setIterationMax(5);
  tracker.setPyramidal(2, 1);

  std::vector<cv::Rect> faces;
  size_t larger_face_index = 0;

  state_t state = detection;
  vpTemplateTrackerZone zone_ref, zone_cur;
  double area_zone_ref, area_zone_cur, area_zone_prev;
  vpColVector p; // Estimated parameters


  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  cv::Mat frame_gray;

  vpRect target;
  int iter = 0;
  try
  {
    while(1) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      vpImageConvert::convert(I, frame_gray);

      std::cout << "state: " << state << std::endl;
      //-- Detect faces
      bool target_found = false;
      if (1) {//state == detection) {
        faces.clear();
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
        std::cout << "Detect " << faces.size() << " faces" << std::endl;
        if (faces.size()) {

          state = init_tracking;

          // Display all the detected faces
          int face_max_area = 0;
          for( size_t i = 0; i < faces.size(); i++ ) {
            if (faces[i].area() > face_max_area) {
              face_max_area = faces[i].area();
              larger_face_index = i;
            }
          }
          target_found = true;
          // Display the larger face
          size_t i=larger_face_index;
          target.set(faces[i].tl().x, faces[i].tl().y, faces[i].size().width, faces[i].size().height);
          //                vpDisplay::displayRectangle(I, target, vpColor::green, false, 4);
        }
      }

      //-- Track the face
      if (state == init_tracking) {
        vpDisplay::displayCharString(I, 10,10, "state: detection", vpColor::red);
        size_t i=larger_face_index;
        double scale = 0.05; // reduction factor
        int x = faces[i].tl().x;
        int y = faces[i].tl().y;
        int width  = faces[i].size().width;
        int height = faces[i].size().height;
        std::vector<vpImagePoint> corners;
        corners.push_back( vpImagePoint(y+scale*height    , x+scale*width) );
        corners.push_back( vpImagePoint(y+scale*height    , x+(1-scale)*width) );
        corners.push_back( vpImagePoint(y+(1-scale)*height, x+(1-scale)*width) );
        corners.push_back( vpImagePoint(y+(1-scale)*height, x+scale*width) );
        try {
          tracker.resetTracker();
          tracker.initFromPoints(I, corners, true);
          tracker.track(I);
          //tracker.display(I, vpColor::green);
          zone_ref = tracker.getZoneRef();
          area_zone_ref = zone_ref.getArea();
          p = tracker.getp();
          warp.warpZone(zone_ref, p, zone_cur);
          area_zone_prev = area_zone_cur = zone_cur.getArea();
          state = tracking;
        }
        catch(...) {
          std::cout << "Exception init tracking" << std::endl;
          state = detection;
        }
      }
      else if (state == tracking) {
        try {
          vpDisplay::displayCharString(I, 10,10, "state: tracking", vpColor::red);
          tracker.track(I);

          //tracker.display(I, vpColor::blue);
          {
            // Instantiate and get the reference zone
            p = tracker.getp();
            warp.warpZone(zone_ref, p, zone_cur);
            area_zone_cur = zone_cur.getArea();

            //          std::cout << "Area ref: " << area_zone_ref << std::endl;
            std::cout << "Area tracked: " << area_zone_cur << std::endl;

            double size_percent = 0.6;
            //if (area_zone_cur/area_zone_ref < size_percent || area_zone_cur/area_zone_ref > (1+size_percent)) {
            if (area_zone_cur/area_zone_prev < size_percent || area_zone_cur/area_zone_prev > (1+size_percent)) {
              std::cout << "reinit caused by size" << std::endl;
              state = detection;
            }
            else {
              target = zone_cur.getBoundingBox();
              target_found = true;
            }

            area_zone_prev = area_zone_cur;
          }
        }
        catch(...) {
          std::cout << "Exception tracking" << std::endl;
          state = detection;
        }
      }

      if (target_found)
        vpDisplay::displayRectangle(I, target, vpColor::red, false, 4);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  return 0;
}

