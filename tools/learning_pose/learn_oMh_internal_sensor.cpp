/**
 *
 * This program compute the desired position of the hand with respect to an object (target of 4 blobs) using the internal sensor.
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
#include <visp/vpCameraParameters.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpPose.h>

#include <iostream>
#include <string>
#include <list>
#include <iterator>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>
#define SAVE 0

#define USE_PLOTTER
#define L 0.015


using namespace AL;


bool computeCentroidBlob(const vpImage<unsigned char> &I,std::list<vpDot2> &blob_list,vpImagePoint &cog_tot,bool &init_done )
{
  vpImagePoint cog;
  cog_tot.set_uv(0,0);
  try
  {
    if (! init_done)
    {
      vpDisplay::flush(I);
      blob_list.clear();
      blob_list.resize(4);

      vpDisplay::displayCharString(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);

      for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
      {

        (*it).setGraphics(true);
        (*it).setGraphicsThickness(2);
        (*it).initTracking(I);
        (*it).track(I);
        vpDisplay::flush(I);
        cog = (*it).getCog();
        cog_tot = cog_tot + cog;

      }

      cog_tot = cog_tot * ( 1.0/ (blob_list.size()) );
      init_done = true;
      std::cout << "init done: " << init_done << std::endl;
    }
    else
    {
      for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
      {

        (*it).track(I);
        cog = (*it).getCog();
        cog_tot = cog_tot + cog;


      }

      // Compute the center of gravity of the object
      cog_tot = cog_tot * ( 1.0/ (blob_list.size()) );
    }
  }
  catch(...)
  {
    init_done = false;
    return false;
  }
  return true;

}


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


  /** Open Proxy for the speech*/
  AL::ALTextToSpeechProxy tts(robotIp, 9559);
  tts.setLanguage("English");
  const std::string phraseToSay = "Yes";
  bool speech = true;

  /** Open the grabber for the acquisition of the images from the robot*/
  vpNaoqiGrabber g;
  g.setFramerate(15);
  g.setCamera(0);
  g.open();

  /** Create a new istance NaoqiRobot*/
  vpNaoqiRobot robot;
  robot.open();


  /** Initialization Visp Image, display and camera paramenters*/
  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");
  //  vpCameraParameters cam;
  //  cam.initPersProjWithoutDistortion(342.82,342.60,174.552518, 109.978367);


  vpCameraParameters cam = g.getCameraParameters();

  std::cout << "Camera parameters: " << cam << std::endl;


  /** Initialization Visp blob*/
  std::list<vpDot2> blob_list;
  vpImagePoint cog_tot(0,0);

  bool init_done = false;


  /** Point to track*/
  int nbPoint =4 ;

  // Position of the points on the target
  vpPoint point[nbPoint] ;
  point[0].setWorldCoordinates(-L,-L, 0) ;
  point[1].setWorldCoordinates(-L,L, 0) ;
  point[2].setWorldCoordinates(L,L, 0) ;
  point[3].setWorldCoordinates(L,-L,0) ;


  std::cout << "Put the hand in the desired position.Object Pose Detection: Click to start." << std::endl;
  // Compute pose of the target on the object
  while(1)
  {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I) ;
    if (vpDisplay::getClick(I, false))
      break;


  }
  std::cout << "Now we can compute the position of the object. Click into the blobs" << std::endl;

  // Detect the blobs
  computeCentroidBlob(I, blob_list, cog_tot, init_done);

  vpHomogeneousMatrix cMo;
  {
    vpImagePoint cog;
    vpPose pose ;
    pose.clearPoint();
    unsigned int kk = 0;
    for (std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
    {
      cog =  (*it).getCog();
      double x=0, y=0;
      vpPixelMeterConversion::convertPoint(cam, cog, x, y) ;
      point[kk].set_x(x) ;
      point[kk].set_y(y) ;
      pose.addPoint(point[kk]) ;
      kk++;
    }
    // compute the initial pose using Dementhon method followed by a non linear
    // minimisation method
    pose.computePose(vpPose::LAGRANGE, cMo) ;
    pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
    std::cout << cMo << std::endl ;
  }

  vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
  vpDisplay::flush(I) ;



  std::cout << "Hand Pose Detection using internal sensor" << std::endl;

  // Trasformation from Torso to Hand sensor
  vpHomogeneousMatrix tMh_sens (robot.getProxy()->getTransform("LArm",0,true));

  // Trasformation from Torso to HeadRoll
  vpHomogeneousMatrix torsoMHeadRoll(robot.getProxy()->getTransform("HeadRoll", 0, true));

  // Trasformation from Headroll to Camera (use estimated extrinsic parameters)
  vpHomogeneousMatrix eMc = g.get_eMc();

  vpHomogeneousMatrix torsoMlcam_visp = torsoMHeadRoll *eMc;



  // Now we can compute the transformation between oMh

  vpHomogeneousMatrix oMh_sens = cMo.inverse() * torsoMlcam_visp.inverse() *tMh_sens;




  std::cout << "oMh_sens:" << std::endl << oMh_sens <<std::endl ;

  vpXmlParserHomogeneousMatrix p_; // Create a XML parser
  std::string name_oMh =  "oMh_Tea_Box_off_set";
  char filename_m[FILENAME_MAX];
  sprintf(filename_m, "%s",VISP_NAOQI_GENERAL_M_FILE );

  if (p_.save(oMh_sens, filename_m, name_oMh) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
  {
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
  }



  while(1)
  {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
    vpDisplay::displayFrame(I, cMo *oMh_sens, cam, 0.05, vpColor::none);
    vpDisplay::flush(I) ;

    if (vpDisplay::getClick(I, false))
      break;

  }


  return 0;
}

