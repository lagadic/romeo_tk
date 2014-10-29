/**
 *
 * This example demonstrates how to get images from the robot remotely, how
 * to track a blob using all the four joints of the Romeo Head;
 *
 */

// Aldebaran includes.
#include <alproxies/altexttospeechproxy.h>

// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <visp/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>
#include <visp/vpFeatureBuilder.h>

#include <iostream>
#include <string>
#include <list>
#include <iterator>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp/vpPose.h>

#define SAVE 0

#define USE_PLOTTER
#define L 0.015


using namespace AL;




int main(int argc, char* argv[])
{
  std::string robotIp = "198.18.0.1";

  if (argc < 2) {
    std::cerr << "Usage: almotion_setangles robotIp "
              << "(optional default \"198.18.0.1\")."<< std::endl;
  }
  else {
    robotIp = argv[1];
  }



  /** Open the grabber for the acquisition of the images from the robot*/
  vpNaoqiGrabber g;
  g.open();

  /** Create a new istance NaoqiRobot*/
  vpNaoqiRobot robot;
  robot.open();


  /** Initialization Visp Image, display and camera paramenters*/
  vpImage<unsigned char> I(240,320);
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(342.82,342.60,174.552518, 109.978367);



  // Introduce a matrix to pass from camera frame of Aldebaran to visp camera frame
  vpHomogeneousMatrix cam_alMe_camvisp;
  for(unsigned int i=0; i<3; i++)
    cam_alMe_camvisp[i][i] = 0; // remove identity
  cam_alMe_camvisp[0][2] = 1.;
  cam_alMe_camvisp[1][0] = -1.;
  cam_alMe_camvisp[2][1] = -1.;



  // Motion

  std::vector<std::string> jointNames =  robot.getBodyNames("LArm");
  jointNames.pop_back(); // Delete last joints LHand, that we don't consider in the servo
  const unsigned int numJoints = jointNames.size();

  std::cout << "The " << numJoints << " joints of the Arm:" << std::endl << jointNames << std::endl;

  // Declarate Jacobian
  vpHomogeneousMatrix torsoMlcam_visp;
  vpHomogeneousMatrix torsoMo;
  //Set the stiffness
  robot.setStiffness(jointNames, 1.f);

  vpImage<vpRGBa> O;


  while(1)
  {


#if 0
    showImages(camProxy,clientName, I);
    if(vpDisplay::getClick(I, false)) {
      vpImageIo::write(I, "/tmp/I.png");
    }
  }
#else
    try
    {
      g.acquire(I);
      vpDisplay::display(I);


        // get the torsoMe_head tranformation from NaoQi api

        vpHomogeneousMatrix torsoMlcam_al;
        std::vector<float> torsoMlcam_al_ = robot.getProxy()->getTransform("CameraLeft", 0, true);
        unsigned int k=0;
        for(unsigned int i=0; i< 4; i++)
          for(unsigned int j=0; j< 4; j++)
            torsoMlcam_al[i][j] = torsoMlcam_al_[k++];

        torsoMlcam_visp = torsoMlcam_al * cam_alMe_camvisp;
        std::cout << "torso M camera visp:\n" << torsoMlcam_visp << std::endl;


        //############################################################################################################
        //                                                  LARM
        //############################################################################################################

        // LWristPitch tranformation -----------------------------------------------------

        std::vector<float> torsoMLWristPitch_ = robot.getProxy()->getTransform("LWristPitch", 0, true); // get torsoMLWristPitch of Aldebaran
        vpHomogeneousMatrix torsoMLWristPitch;
        k=0;
        for(unsigned int i=0; i< 4; i++)
          for(unsigned int j=0; j< 4; j++)
            torsoMLWristPitch[i][j] = torsoMLWristPitch_[k++];

        std::cout << "Torso M LWristPitch:\n" << torsoMLWristPitch << std::endl;

       vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLWristPitch, cam, 0.04, vpColor::none);


       // -----------------------------------------------------------------------------------

       // LElbowRoll tranformation -----------------------------------------------------

       std::vector<float> torsoMLElbowRoll_ = robot.getProxy()->getTransform("LElbowRoll", 0, true); // get LElbowRoll of Aldebaran
       vpHomogeneousMatrix torsoMLElbowRoll;
       k=0;
       for(unsigned int i=0; i< 4; i++)
         for(unsigned int j=0; j< 4; j++)
           torsoMLElbowRoll[i][j] = torsoMLElbowRoll_[k++];

       std::cout << "Torso M LElbowRoll:\n" << torsoMLElbowRoll << std::endl;

      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLElbowRoll, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------

      // LElbowYaw tranformation -----------------------------------------------------

      std::vector<float> torsoMLElbowYaw_ = robot.getProxy()->getTransform("LElbowYaw", 0, true); // get LElbowYaw of Aldebaran
      vpHomogeneousMatrix torsoMLLElbowYaw;
      k=0;
      for(unsigned int i=0; i< 4; i++)
        for(unsigned int j=0; j< 4; j++)
          torsoMLLElbowYaw[i][j] = torsoMLElbowYaw_[k++];

      std::cout << "Torso M LElbowYaw:\n" << torsoMLLElbowYaw << std::endl;

     vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLLElbowYaw, cam, 0.04, vpColor::none);


     // -----------------------------------------------------------------------------------



     //############################################################################################################
     //                                                  RARM
     //############################################################################################################

     // RWristPitch tranformation -----------------------------------------------------

     std::vector<float> torsoMRWristPitch_ = robot.getProxy()->getTransform("RWristPitch", 0, true); // get torsoMRWristPitch of Aldebaran
     vpHomogeneousMatrix torsoMRWristPitch;
     k=0;
     for(unsigned int i=0; i< 4; i++)
       for(unsigned int j=0; j< 4; j++)
         torsoMRWristPitch[i][j] = torsoMRWristPitch_[k++];

     std::cout << "Torso M RWristPitch:\n" << torsoMRWristPitch << std::endl;

    vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMRWristPitch, cam, 0.04, vpColor::none);


    // -----------------------------------------------------------------------------------

    // RElbowRoll tranformation -----------------------------------------------------

    std::vector<float> torsoMRElbowRoll_ = robot.getProxy()->getTransform("RElbowRoll", 0, true); // get RElbowRoll of Aldebaran
    vpHomogeneousMatrix torsoMRElbowRoll;
    k=0;
    for(unsigned int i=0; i< 4; i++)
      for(unsigned int j=0; j< 4; j++)
        torsoMRElbowRoll[i][j] = torsoMRElbowRoll_[k++];

    std::cout << "Torso M RElbowRoll:\n" << torsoMRElbowRoll << std::endl;

   vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMRElbowRoll, cam, 0.04, vpColor::none);


   // -----------------------------------------------------------------------------------

   // RElbowYaw tranformation -----------------------------------------------------

   std::vector<float> torsoMRElbowYaw_ = robot.getProxy()->getTransform("RElbowYaw", 0, true); // get RElbowYaw of Aldebaran
   vpHomogeneousMatrix torsoMRElbowYaw;
   k=0;
   for(unsigned int i=0; i< 4; i++)
     for(unsigned int j=0; j< 4; j++)
       torsoMRElbowYaw[i][j] = torsoMRElbowYaw_[k++];

   std::cout << "Torso M RElbowYaw:\n" << torsoMRElbowYaw << std::endl;

  vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMRElbowYaw, cam, 0.04, vpColor::none);


  // -----------------------------------------------------------------------------------




       vpDisplay::flush(I) ;
       //vpTime::sleepMs(20);



    }
    catch (const AL::ALError& e)
    {
      std::cerr << "Caught exception " << e.what() << std::endl;
    }

    if (vpDisplay::getClick(I, false))
      break;

    vpDisplay::flush(I);
    vpDisplay::getImage(I, O);

  }

  std::cout << "The end: stop the robot..." << std::endl;

#endif

  return 0;
}

