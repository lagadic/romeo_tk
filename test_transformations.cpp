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


      // get the torsoMlcam_al tranformation from NaoQi api
      vpHomogeneousMatrix torsoMlcam_al(robot.getProxy()->getTransform("CameraLeft", 0, false));
      torsoMlcam_visp = torsoMlcam_al * cam_alMe_camvisp;
      std::cout << "torso M camera visp:\n" << torsoMlcam_visp << std::endl;


      vpHomogeneousMatrix torsoMHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, false));
      std::cout << "torsoMHeadRoll:\n" << torsoMHeadRoll << std::endl;


      vpHomogeneousMatrix HeadRollMcam_visp;
      HeadRollMcam_visp = torsoMHeadRoll.inverse()* torsoMlcam_visp ;
      std::cout << "HeadRoll M camera visp:\n" << HeadRollMcam_visp << std::endl;



      //############################################################################################################
      //                                                  LARM
      //############################################################################################################
      // LWristPitch tranformation -----------------------------------------------------


      vpHomogeneousMatrix torsoMLWristPitch( robot.getProxy()->getTransform("LWristPitch", 0, true));
      std::cout << "Torso M LWristPitch:\n" << torsoMLWristPitch << std::endl;
      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLWristPitch, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------
      // LElbowRoll tranformation -----------------------------------------------------


      vpHomogeneousMatrix torsoMLElbowRoll(robot.getProxy()->getTransform("LElbowRoll", 0, true));
      std::cout << "Torso M LElbowRoll:\n" << torsoMLElbowRoll << std::endl;
      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLElbowRoll, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------

      // LElbowYaw tranformation -----------------------------------------------------

      vpHomogeneousMatrix torsoMLLElbowYaw(robot.getProxy()->getTransform("LElbowYaw", 0, true));
      std::cout << "Torso M LElbowYaw:\n" << torsoMLLElbowYaw << std::endl;
      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLLElbowYaw, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------


      //############################################################################################################
      //                                                  RARM
      //############################################################################################################

      // RWristPitch tranformation -----------------------------------------------------

      vpHomogeneousMatrix torsoMRWristPitch(robot.getProxy()->getTransform("RWristPitch", 0, true));
      std::cout << "Torso M RWristPitch:\n" << torsoMRWristPitch << std::endl;
      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMRWristPitch, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------

      // RElbowRoll tranformation -----------------------------------------------------

      vpHomogeneousMatrix torsoMRElbowRoll(robot.getProxy()->getTransform("RElbowRoll", 0, true));
      std::cout << "Torso M RElbowRoll:\n" << torsoMRElbowRoll << std::endl;
      vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMRElbowRoll, cam, 0.04, vpColor::none);


      // -----------------------------------------------------------------------------------

      // RElbowYaw tranformation -----------------------------------------------------


      vpHomogeneousMatrix torsoMRElbowYaw(robot.getProxy()->getTransform("RElbowYaw", 0, true));
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

