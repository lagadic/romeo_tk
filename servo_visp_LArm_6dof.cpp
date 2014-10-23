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
      // Display the ACTUAL center of gravity of the object
      vpDisplay::displayCross(I,cog_tot,10, vpColor::blue,2 );
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
  vpImage<unsigned char> I(240,320);
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(342.82,342.60,174.552518, 109.978367);

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
  pose.computePose(vpPose::DEMENTHON_LOWE, cMo) ;
  std::cout << cMo << std::endl ;
  cMo.print() ;

  /*------------------------------------------------------------------
  --  Learning the desired position
  --  or reading the desired position
  ------------------------------------------------------------------
  */
  std::cout << " Learning 0/1 " <<std::endl ;
  char name[FILENAME_MAX] ;
  sprintf(name,"cdMo.dat") ;
  int learning ;
  std::cin >> learning ;
  if (learning ==1)
  {
    // save the object position
    vpTRACE("Save the location of the object in a file cdMo.dat") ;
    std::ofstream f(name) ;
    cMo.save(f) ;
    f.close() ;
    exit(1) ;
  }


  {
    vpTRACE("Loading desired location from cdMo.dat") ;
    std::ifstream f("cdMo.dat") ;
    cdMo.load(f) ;
    f.close() ;
  }

  vpFeaturePoint p[nbPoint], pd[nbPoint] ;

  // set the desired position of the point by forward projection using
  // the pose cdMo
  for (unsigned int i=0 ; i < nbPoint ; i++)
  {
    vpColVector cP, p ;
    point[i].changeFrame(cdMo, cP) ;
    point[i].projection(cP, p) ;

    pd[i].set_x(p[0]) ;
    pd[i].set_y(p[1]) ;
  }

  /** Initialization Visual servoing task*/
  vpServo task; // Visual servoing task
  task.setServo(vpServo::EYETOHAND_L_cVf_fVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  task.setInteractionMatrixType(vpServo::CURRENT);


  for (unsigned int i=0 ; i < nbPoint ; i++)
  {
    task.addFeature(p[i],pd[i]) ;
  }


  vpTRACE("Display task information " ) ;
  task.print() ;

  vpDisplay::getClick(I) ;


  //  vpFeaturePoint sd; //The desired point feature.
  //  //Set the desired features x and y
  //  double xd = 0;
  //  double yd = 0;
  //  //Set the depth of the point in the camera frame.
  //  double Zd = 0.5;
  //Set the point feature thanks to the desired parameters.
  //  sd.buildFrom(xd, yd, Zd);
  //  vpFeaturePoint s; //The current point feature.
  //  //Set the current features x and y
  //  double x = xd; //You have to compute the value of x.
  //  double y = yd; //You have to compute the value of y.
  //  double Z = Zd; //You have to compute the value of Z.
  //  //Set the point feature thanks to the current parameters.
  //  s.buildFrom(x, y, Z);
  //  // Add the 2D point feature to the task
  //  task.addFeature(s, sd);

  //    vpAdaptiveGain lambda(2, 0.8, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
  //    task.setLambda(lambda);
  task.setLambda(0.3);

  vpColVector q_dot;


  // Constant transformation Target Frame to LArm end-effector (LWristPitch)
  vpHomogeneousMatrix oMe_LArm;
  oMe_LArm[0][3] = -0.05;
  oMe_LArm[1][3] = 0.0;
  oMe_LArm[2][3] = -0.026;


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
    double t = vpTime::measureTimeMs();

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

          p[kk].set_x(x) ;
          p[kk].set_y(y) ;
          p[kk].set_Z(cP[2]) ;

          pose.addPoint(point[kk]) ;

          point[kk].display(I,cMo,cam, vpColor::green) ;
          point[kk].display(I,cdMo,cam, vpColor::blue) ;
          kk++;
        }
        pose.computePose(vpPose::LOWE, cMo) ;
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
        vpDisplay::displayFrame(I, cdMo, cam, 0.05, vpColor::none);
        vpDisplay::flush(I) ;




        //s.buildFrom(x, y, Z);

        //** Set task eJe matrix
        eJe_LArm = robot.get_eJe("LArm");
        oJo = oVe_LArm * eJe_LArm;
        task.set_eJe(oJo);

        //** Set task cVf matrix
        // get the torsoMe_head tranformation from NaoQi api

        vpHomogeneousMatrix torsoMlcam_al;
        std::vector<float> torsoMlcam_al_ = robot.getProxy()->getTransform("CameraLeft", 0, true); // get torsoMhead_roll which is equal to our torsoMe_head
        unsigned int k=0;
        for(unsigned int i=0; i< 4; i++)
          for(unsigned int j=0; j< 4; j++)
            torsoMlcam_al[i][j] = torsoMlcam_al_[k++];


        std::cout << "torso M camera ald :\n" << torsoMlcam_al << std::endl;

        torsoMlcam_visp = torsoMlcam_al * cam_alMe_camvisp;
        std::cout << "torso M camera visp:\n" << torsoMlcam_visp << std::endl;


        vpVelocityTwistMatrix cVtorso(torsoMlcam_visp.inverse());
        task.set_cVf( cVtorso );

        //** Set task fVe matrix
        // get the torsoMe_LArm tranformation from NaoQi api

        std::vector<float> torsoMLWristPitch_ = robot.getProxy()->getTransform("LWristPitch", 0, true); // get torsoMLWristPitch of Aldebaran
        vpHomogeneousMatrix torsoMLWristPitch;
        k=0;
        for(unsigned int i=0; i< 4; i++)
          for(unsigned int j=0; j< 4; j++)
            torsoMLWristPitch[i][j] = torsoMLWristPitch_[k++];

        std::cout << "Torso M LWristPitch:\n" << torsoMLWristPitch << std::endl;


        torsoMo = torsoMLWristPitch * oMe_LArm.inverse();

        std::cout << "torso M object :\n" << torsoMo << std::endl;

        vpVelocityTwistMatrix torsoVo(torsoMo);
        task.set_fVe( torsoVo );

        q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - tinit);
        //q_dot = task.computeControlLaw();

#ifdef USE_PLOTTER
        graph.plot(0, iter, q_dot); // plot joint velocities applied to the robot
        graph.plot(1, iter, task.getError()); // plot error vector s-s*
        iter++;
#endif

        task.print();


        std::cout << "q dot: " << q_dot.t() << " in deg/s: "
                  << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;

       robot.setVelocity(jointNames, q_dot);

        // Compute the distance in pixel between the target and the center of the image
//        double distance = vpImagePoint::distance(cog_desired, cog_tot);

//        if (distance < 0.03*I.getWidth() && speech) // 3 % of the image witdh
//        {
//          /** Call the say method */
//          tts.post.say(phraseToSay);
//          speech = false;

//        }
//        else if (distance > 0.15*I.getWidth()) // 15 % of the image witdh
//          speech = true;
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
      break;

    vpDisplay::flush(I);
    vpDisplay::getImage(I, O);
    std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames);

  task.kill();
#endif

  return 0;
}

