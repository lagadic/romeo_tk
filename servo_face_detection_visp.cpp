/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
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
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

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

  g.open();

  vpNaoqiRobot robot;

  std::vector<std::string> jointNames;
  jointNames.push_back("NeckYaw");
  jointNames.push_back("NeckPitch");

  robot.open();

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


  vpImage<unsigned char> I(240,320);
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  cv::Mat frame_gray;

  vpRect target;
  int iter = 0;

  vpServo task; // Visual servoi    vpServo task; // Visual servoing task
  vpFeaturePoint sd; //The desired point feature.
  //Set the desired features x and y
  double xd = 0;
  double yd = 0;
  //Set the depth of the point in the camera frame.
  double Zd = 1.8;
  //Set the point feature thanks to the desired parameters.
  sd.buildFrom(xd, yd, Zd);
  vpFeaturePoint s; //The current point feature.
  //Set the current features x and y
  double x = xd; //You have to compute the value of x.
  double y = yd; //You have to compute the value of y.
  double Z = Zd; //You have to compute the value of Z.
  //Set the point feature thanks to the current parameters.
  s.buildFrom(x, y, Z);
  //In this case the parameter Z is not necessary because the interaction matrix is computed
  //with the desired visual feature.
  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::DESIRED);
  // Add the 2D point feature to the task
  task.addFeature(s, sd);

  vpAdaptiveGain lambda(0.5, 0.01, 5); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
  //task.setLambda(lambda);
  task.setLambda(0.05);

  vpColVector q_dot;
  vpMatrix eJe(6,2);
  eJe = 0.;
  eJe[4][0] = -1.;
  eJe[5][1] =  1.;

  vpHomogeneousMatrix cMe;
  cMe[0][3] = -0.0319;
  cMe[1][3] = -0.1326;
  cMe[2][3] =  0.998;


  std::vector<float> jointVel(2);

  for(unsigned int i=0; i< jointVel.size(); i++)
    jointVel[i] = 0.0f;

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(323.2023246,323.6059094,169.0936523, 119.5883104);

  robot.setStiffness(jointNames, 1.f);

  double tinit = 0; // initial time in second

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

      if (target_found) {
        vpDisplay::displayRectangle(I, target, vpColor::red, false, 4);

        vpImagePoint cog = target.getCenter();
        vpDisplay::displayCross(I, cog, 12, vpColor::red, 3);
        double x=0, y=0;
        vpPixelMeterConversion::convertPoint(cam, cog, x, y);
        s.buildFrom(x, y, Z);

        task.set_eJe(eJe);
        task.set_cVe( vpVelocityTwistMatrix(cMe) );

        q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - tinit);
        task.print();

        vpImagePoint cog_desired;
        vpMeterPixelConversion::convertPoint(cam, sd.get_x(), sd.get_y(), cog_desired);
        vpDisplay::displayCross(I, cog_desired, 10, vpColor::green, 2);
        std::cout << "q dot: " << q_dot.t() << " in deg/s: "
                  << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;
        jointVel[0] = q_dot[0]*4.5;
        jointVel[1] = q_dot[1]*0.5;
        robot.setVelocity(jointNames, jointVel);
      }
      else {
        std::cout << "Stop the robot..." << std::endl;
        robot.stop(jointNames);

      }
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

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames);

  task.kill();

  return 0;
}

