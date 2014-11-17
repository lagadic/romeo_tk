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
#include <visp/vpCameraParameters.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpPose.h>

#include <iostream>
#include <string>
#include <list>
#include <iterator>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpXmlParserHomogeneousMatrix.h>
#include<visp_naoqi/vpNaoqiConfig.h>
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

  char filename[FILENAME_MAX];
  vpCameraParameters cam;
  vpXmlParserCamera p; // Create a XML parser
  vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
  // Use a perspective projection model without distortion
  projModel = vpCameraParameters::perspectiveProjWithDistortion;
  // Parse the xml file "myXmlFile.xml" to find the intrinsic camera
  // parameters of the camera named "myCamera" for the image sizes 640x480,
  // for the projection model projModel. The size of the image is optional
  // if camera parameters are given only for one image size.
  sprintf(filename, "%s", "camera.xml");
  if (p.parse(cam, filename, "Camera", projModel, I.getWidth(), I.getHeight()) != vpXmlParserCamera::SEQUENCE_OK) {
    std::cout << "Cannot found camera parameters in file: " << filename << std::endl;
    std::cout << "Loading default camera parameters" << std::endl;
    cam.initPersProjWithoutDistortion(342.82, 342.60, 174.552518, 109.978367);
  }

  std::cout << "Camera parameters: " << cam << std::endl;


  /** Load transformation between teabox and desired position of the hand to grasp it*/

  vpHomogeneousMatrix oMe_d;
  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  std::string name_oMe_d =  "oMh_Small_Tea_Box1";

  char filename_[FILENAME_MAX];
  sprintf(filename_, "%s", VISP_NAOQI_GENERAL_M_FILE);

  if (pm.parse(oMe_d,filename_, name_oMe_d) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the Homogeneous matrix named " << name_oMe_d<< "." << std::endl;
    return 0;
  }
  else
    std::cout << "Homogeneous matrix " << name_oMe_d <<": " << std::endl << oMe_d << std::endl;



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

  g.acquire(I);
  vpDisplay::display(I);

  // Detect the blobs
  computeCentroidBlob(I, blob_list, cog_tot, init_done);

  vpHomogeneousMatrix cMo, cdMo ;
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
  cMo.print() ;

  /*------------------------------------------------------------------
  --  Learning the desired position
  --  or reading the desired position
  ------------------------------------------------------------------
  */
  std::cout << " Learning 0/1 " <<std::endl ;
  char name[FILENAME_MAX] ;
  sprintf(name,"cdMo_PB_grasp.dat") ;
  int learning ;
  std::cin >> learning ;
  if (learning ==1)
  {
    vpHomogeneousMatrix cMhand_d;

    cMhand_d = cMo * oMe_d;


    vpDisplay::displayFrame(I, cMhand_d, cam, 0.05, vpColor::none);
    vpDisplay::flush(I) ;

    // save the object position
    vpTRACE("Save the location of the object in a file cdMo_PB_grasp.dat") ;
    std::ofstream f(name) ;
    cMhand_d.save(f) ;
    f.close() ;
    vpDisplay::getClick(I) ;

    exit(1) ;
  }


  {
    vpTRACE("Loading desired location from cdMo_PB_grasp.dat") ;
    std::ifstream f("cdMo_PB_grasp.dat") ;
    cdMo.load(f) ;
    f.close() ;
    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
    vpDisplay::displayFrame(I, cdMo, cam, 0.05, vpColor::none);
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I) ;
  }


  // Sets the desired position of the visual feature
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;
  vpFeatureTranslation t(vpFeatureTranslation::cdMc) ;
  vpFeatureThetaU tu(vpFeatureThetaU::cdRc); // current feature
  t.buildFrom(cdMc) ;
  tu.buildFrom(cdMc) ;



  /** Initialization Visual servoing */
  vpServo task; // Visual servoing task


  // We want to see a point on a point
  task.addFeature(t) ;   // 3D translation
  task.addFeature(tu) ; // 3D rotation


  task.setServo(vpServo::EYETOHAND_L_cVf_fVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);




  vpTRACE("Display task information " ) ;
  task.print() ;



  task.setLambda(0.10);


  //  // Set the proportional gain
  //  // - set the gain
  //  vpAdaptiveGain  lambda;
  //  lambda.initStandard(2, 0.2, 50);

  //  task.setLambda(lambda) ;

  vpColVector q_dot;


  // Constant transformation Target Frame to LArm end-effector (LWristPitch)
  vpHomogeneousMatrix oMe_LArm;
  for(unsigned int i=0; i<3; i++)
    oMe_LArm[i][i] = 0; // remove identity
  oMe_LArm[0][0] = 1;
  oMe_LArm[1][2] = 1;
  oMe_LArm[2][1] = -1;

  oMe_LArm[0][3] = -0.045;
  oMe_LArm[1][3] = -0.04;
  oMe_LArm[2][3] = -0.045;


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
  vpMatrix eJe_LArm;
  vpVelocityTwistMatrix oVe_LArm(oMe_LArm);
  vpMatrix oJo; // Jacobian in the target (=object) frame
  vpHomogeneousMatrix torsoMlcam_visp;
  vpHomogeneousMatrix torsoMo;
  //Set the stiffness
  robot.setStiffness(jointNames, 1.f);


  double tinit = 0; // initial time in second

  robot.getProxy()->openHand("LHand");

  vpImage<vpRGBa> O;

#ifdef USE_PLOTTER
  // Create a window (800 by 500) at position (400, 10) with 3 graphics
  vpPlot graph(2, 800, 500, 400, 10, "Curves...");
  // Init the curve plotter
  graph.initGraph(0, numJoints); // q_dot
  graph.initGraph(1, 2); // s-s*
  graph.setTitle(0, "Joint velocities");
  graph.setTitle(1, "Error s-s*");
  for(unsigned int i=0; i<numJoints; i++)
    graph.setLegend(0, i, jointNames[i].c_str());
  graph.setLegend(1, 0, "x");
  graph.setLegend(1, 1, "y");
#endif

  unsigned int iter = 0;
  while(1)
  {
    double time = vpTime::measureTimeMs();

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



      bool tracking_status = computeCentroidBlob(I, blob_list, cog_tot, init_done);
      if (! init_done)
        tinit = vpTime::measureTimeSecond();

      if (init_done && (tracking_status == true)) {

        // compute the initial pose using  a non linear minimisation method
        pose.clearPoint() ;

        kk = 0;

        for (std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
        {
          double x=0, y=0;
          cog = (*it).getCog();
          vpPixelMeterConversion::convertPoint(cam, cog, x, y)  ;
          point[kk].set_x(x) ;
          point[kk].set_y(y) ;

          vpColVector cP ;
          point[kk].changeFrame(cdMo, cP) ;


          pose.addPoint(point[kk]) ;

          //point[kk].display(I,cMo,cam, vpColor::green) ;
          //point[kk].display(I,cdMo,cam, vpColor::blue) ;
          kk++;
        }
        pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
        vpDisplay::displayFrame(I, cdMo, cam, 0.05, vpColor::none);


        // Update the features
        cdMc = cdMo*cMo.inverse();
        t.buildFrom(cdMc);
        tu.buildFrom(cdMc);


        //** Set task eJe matrix
        eJe_LArm = robot.get_eJe("LArm");
        oJo = oVe_LArm * eJe_LArm;
        task.set_eJe(oJo);

        //** Set task cVf matrix
        // get the torsoMe_head tranformation from NaoQi api

        vpHomogeneousMatrix torsoMlcam_al(robot.getProxy()->getTransform("CameraLeft", 0, true));

        std::cout << "torso M camera ald :\n" << torsoMlcam_al << std::endl;
        torsoMlcam_visp = torsoMlcam_al * cam_alMe_camvisp;
        std::cout << "torso M camera visp:\n" << torsoMlcam_visp << std::endl;


        vpVelocityTwistMatrix cVtorso(torsoMlcam_visp.inverse());
        task.set_cVf( cVtorso );

        //** Set task fVe matrix
        // get the torsoMe_LArm tranformation from NaoQi api

        vpHomogeneousMatrix torsoMLWristPitch(robot.getProxy()->getTransform("LWristPitch", 0, true));
        std::cout << "Torso M LWristPitch:\n" << torsoMLWristPitch << std::endl;


        torsoMo = torsoMLWristPitch * oMe_LArm.inverse();
        std::cout << "torso M object :\n" << torsoMo << std::endl;

        vpVelocityTwistMatrix torsoVo(torsoMo);
        task.set_fVe( torsoVo );

        q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - tinit);




#ifdef USE_PLOTTER
        graph.plot(0, iter, q_dot); // plot joint velocities applied to the robot
        graph.plot(1, iter, task.getError()); // plot error vector s-s*
        iter++;
#endif

        task.print();


        std::cout << "q dot: " << q_dot.t() << " in deg/s: "
                  << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;

        robot.setVelocity(jointNames, q_dot);


        vpDisplay::displayFrame(I, torsoMlcam_visp.inverse()*torsoMLWristPitch, cam, 0.04, vpColor::green);


        vpDisplay::flush(I) ;
        vpTime::sleepMs(20);


      }
      else {
        std::cout << "Stop the robot..." << std::endl;
        robot.stop(jointNames);

      }


    }
    catch (const AL::ALError& e)
    {
      std::cerr << "Caught exception " << e.what() << std::endl;
    }

    if (vpDisplay::getClick(I, false))

    {
      q_dot = 0.0 * q_dot;
      robot.setVelocity(jointNames, q_dot);

      break;
    }

    vpDisplay::flush(I);
    vpDisplay::getImage(I, O);
    std::cout << "Loop time: " << vpTime::measureTimeMs() - time << std::endl;
  }

  // Grasping

  //robot.stop(jointNames);

  std::string nameChain = "LArm";

  std::cout << "Click to Graps" << std::endl;
  vpDisplay::getClick(I);
  //robot.getProxy()->closeHand("LHand");
  robot.getProxy()->setStiffnesses("LHand", 1.0f);
  AL::ALValue angle = 0.15;
  robot.getProxy()->setAngles("LHand",angle,0.15);

  std::cout << "Click to take the object " << std::endl;
  vpDisplay::getClick(I);

  std::vector<float> handPos = robot.getProxy()->getPosition(nameChain, 0, false);
  handPos[2] =  handPos[2] + 0.07;
  robot.getProxy()->setPositions(nameChain,0,handPos,0.05,7);

  std::cout << "Click to put back the object " << std::endl;
  vpDisplay::getClick(I);

  handPos = robot.getProxy()->getPosition(nameChain, 0, false);
  handPos[2] =  handPos[2] - 0.06;
  robot.getProxy()->setPositions(nameChain,0,handPos,0.05,7);

  std::cout << "Click to Open the Hand" <<  std::endl;
  vpDisplay::getClick(I);

  //robot.getProxy()->openHand("LHand");


  robot.getProxy()->setStiffnesses("LHand", 1.0f);
  angle = 1.0f;
  robot.getProxy()->setAngles("LHand",angle,0.7);


  std::cout << "Click to Stop the demo" << std::endl;
  vpDisplay::getClick(I);

  std::cout << "The end: stop the robot..." << std::endl;
  robot.getProxy()->killMove();
  robot.stop(jointNames);
  task.kill();



#endif

  return 0;
}

