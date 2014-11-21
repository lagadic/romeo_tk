/**
 * This example demonstrates how to get images from the robot remotely, how
 * to track a face using all the four joints of the Romeo Head avoiding
 * the joints limits.
 *
 */

/*! \example servo_face_detection_visp_head_joint_avoidance.cpp */

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
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#define USE_PLOTTER

typedef enum {
  detection,
  init_tracking,
  tracking,
  none
} state_t;

/*!

  Connect to Romeo robot, grab, display images using ViSP and start
  face detection with OpenCV and tracking with ViSP when the detection fails.
  More over all the four joints of Romeo's head are controlled by visual servoing to center
  the detected head in the image. A joint avoidance scheme is also used here.
  By default, this example connect to a robot with ip address: 198.18.0.1.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head_joint_avoidance -ip <robot ip address> -haar <haar cascade .xml file>

  Example:

  ./servo_face_detection_visp_head_joint_avoidance -ip 169.254.168.230 -haar ./haarcascade_frontalface_alt.xml
 */
int main(int argc, const char* argv[])
{
  std::string opt_ip = "198.18.0.1";;

  //-- 1. Load the cascades
  cv::CascadeClassifier face_cascade;
  /** Global variables */
  cv::String face_cascade_name = "./haarcascade_frontalface_alt.xml";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "-ip")
        opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "-haar")
      face_cascade_name = cv::String(argv[i+1]);
  }

  if( !face_cascade.load( face_cascade_name ) ) {
    std::cout << "--(!)Error loading default " << face_cascade_name << std::endl;
    std::cout << "Usage : " << argv[0] << " <haarcascade_file.xml>" << std::endl;
    return -1;
  };

  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.open();

  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
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

  vpImage<unsigned char> I(g.getHeight(), g.getWidth());
  vpDisplayX d(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  cv::Mat frame_gray;

  vpRect target;

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

  // Set the gain
  double lambda = 0.8;
  // set to -1 to suppress the lambda used in the vpServo::computeControlLaw()
  task.setLambda(-1) ;

  // Transformation HeadRoll to Camera Left
  vpHomogeneousMatrix eMc = g.get_eMc();



  std::string chainName = "Head";

  std::vector<std::string> jointNames =  robot.getBodyNames(chainName);
  const unsigned int numJoints = jointNames.size();

  std::vector<float> jointVel(numJoints);

  for(unsigned int i=0; i< numJoints; i++)
    jointVel[i] = 0.0f;

  // Initialize the joint avoidance scheme from the joint limits
  vpColVector jointMin = robot.getJointMin(chainName);
  vpColVector jointMax = robot.getJointMax(chainName);

  std::cout << "Joint limits: " << std::endl;
  for (unsigned int i=0; i< numJoints; i++)
    std::cout << " Joint " << i << " " << jointNames[i]
                 << ": min=" << vpMath::deg(jointMin[i])
                 << " max=" << vpMath::deg(jointMax[i]) << std::endl;

  vpColVector Qmin(numJoints), tQmin(numJoints) ;
  vpColVector Qmax(numJoints), tQmax(numJoints) ;
  vpColVector Qmiddle(numJoints);
  vpColVector data(numJoints + 4); // data to plot: q(t), Qmin, Qmax, tQmin and tQmax
  vpColVector q(numJoints), qpre(numJoints); // joint positions

  double rho = 0.25 ;
  for (unsigned int i=0 ; i < numJoints ; i++)
  {
    Qmin[i] = jointMin[i] + 0.5*rho*(jointMax[i]-jointMin[i]) ;
    Qmax[i] = jointMax[i] - 0.5*rho*(jointMax[i]-jointMin[i]) ;
  }
  Qmiddle = (Qmin + Qmax) /2.;
  double rho1 = 0.1 ;

  for (unsigned int i=0 ; i < numJoints ; i++) {
    tQmin[i]=Qmin[i]+ 0.5*(rho1)*(Qmax[i]-Qmin[i]) ;
    tQmax[i]=Qmax[i]- 0.5*(rho1)*(Qmax[i]-Qmin[i]) ;
  }

#ifdef USE_PLOTTER
  int iter = 0;

  // Create a window with two graphics
  // - first graphic to plot q(t), Qmin, Qmax, tQmin and tQmax
  // - second graphic to plot the cost function h_s
  vpPlot plot(2, 700, 700, 100, 200, "Curves...");

  // The first graphic contains 8 data to plot: q(t), Qmin, Qmax, tQmin and tQmax
  plot.initGraph(0, 8);//data.size());
  plot.initGraph(1, 4);//numJoints);
  // For the first graphic :
  // - along the x axis the expected values are between 0 and 200 and
  //   the step is 1
  // - along the y axis the expected values are between -1.2 and 1.2 and the
  //   step is 0.1
  //plot.initRange(0,0,200,1,-1.2,1.2,0.1);
  plot.setTitle(0, "Joint behavior");
  //plot.initRange(1,0,200,1,-0.01,0.01,0.05);
  plot.setTitle(1, "Joint velocity");

  // For the first graphic, set the curves legend
  char legend[10];
  for (unsigned int i=0; i < numJoints; i++) {
    sprintf(legend, "q%d", i+1);
    plot.setLegend(0, i, legend);
    sprintf(legend, "q%d", i+1);
    plot.setLegend(1, i, legend);
  }
  plot.setLegend(0, numJoints, "tQmin");
  plot.setLegend(0, numJoints+1, "tQmax");
  plot.setLegend(0, numJoints+2, "Qmin");
  plot.setLegend(0, numJoints+3, "Qmax");

  // Set the curves color
  plot.setColor(0, 0, vpColor::red);
  plot.setColor(0, 1, vpColor::green);
  plot.setColor(0, 2, vpColor::blue);
  plot.setColor(0, 3, vpColor::orange);
  for (unsigned int i= numJoints; i < numJoints+4; i++)
    plot.setColor(0, i, vpColor::black); // for Q and tQ [min,max]
  // Set the curves color

  plot.setColor(1, 0, vpColor::red);
  plot.setColor(1, 1, vpColor::green);
  plot.setColor(1, 2, vpColor::blue);
  plot.setColor(1, 3, vpColor::orange);
#endif // #ifdef USE_PLOTTER

  // Declate Jacobian
  vpMatrix eJe(6,numJoints);

  vpCameraParameters cam = g.getCameraParameters();
  //cam.initPersProjWithoutDistortion(342.82,342.60,174.552518, 109.978367);

  robot.setStiffness(chainName, 1.f);

  double tinit = 0; // initial time in second
  double t_0, t_1 = vpTime::measureTimeMs(), Tv;

  try
  {
    while(1) {
      t_0 = vpTime::measureTimeMs(); // t_0: loop start time
      // Update loop time in second
      Tv = (double)(t_0 - t_1) / 1000.0;
      // Update time for next iteration
      t_1 = t_0;
      double Tloop = Tv;

      g.acquire(I);
      vpDisplay::display(I);

      q = robot.getPosition(chainName);

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

        eJe = robot.get_eJe("Head");
        task.set_eJe(eJe);
        task.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

        vpColVector prim_task ;
        vpColVector e2(numJoints) ;
        // Compute the visual servoing skew vector
        prim_task = task.computeControlLaw() ;
        std::cout << "prim_task : " << prim_task.t() << std::endl;

        qpre = q ;
        qpre += -lambda*prim_task*(4*Tloop)  ;

        // Identify the joints near the limits
        vpColVector pb(numJoints) ; pb = 0 ;
        unsigned int npb =0 ;
        for (unsigned int i=0 ; i < numJoints ;i++) {
          if (q[i] < tQmin[i])
            if (fabs(Qmin[i]-q[i]) > fabs(Qmin[i]-qpre[i])) {
              pb[i] = 1 ; npb++ ;
              std::cout << "Joint " << i << " near limit " << std::endl ;
            }
          if (q[i]>tQmax[i]) {
            if (fabs(Qmax[i]-q[i]) > fabs(Qmax[i]-qpre[i])) {
              pb[i] = 1 ; npb++ ;
              std::cout << "Joint " << i << " near limit " << std::endl ;
            }
          }
        }

        try {
          vpColVector a0 ;
          vpMatrix J1 = task.getTaskJacobian();
          std::cout << "J1: \n" << J1 << std::endl;
          vpMatrix kernelJ1;
          J1.kernel(kernelJ1);
          std::cout << "kernelJ1: \n" << kernelJ1 << std::endl;

          unsigned int dimKernelL = kernelJ1.getCols() ;
          if (npb != 0) {
            // Build linear system a0*E = S
            vpMatrix E(npb, dimKernelL) ;
            vpColVector S(npb) ;

            unsigned int k=0 ;

            for (unsigned int j=0 ; j < numJoints ; j++) // j is the joint
              //if (pb[j]==1)	{
              if (std::fabs(pb[j]-1) <= std::numeric_limits<double>::epsilon())	{
                for (unsigned int i=0 ; i < dimKernelL ; i++)
                  E[k][i] = kernelJ1[j][i] ;

                S[k] = -prim_task[j]  ;
                k++ ;
              }
            vpMatrix Ep ;
            Ep = E.t()*(E*E.t()).pseudoInverse() ;
            a0 = Ep*S ;

            e2 = (kernelJ1*a0) ;
          }
          else {
            e2 = 0;
          }
        }
        catch(...) {
          e2 = 0;
        }

        //        std::cout << "e2: " << e2.t() << std::endl;

        vpColVector q_dot = -lambda * (prim_task + e2);

        vpImagePoint cog_desired;
        vpMeterPixelConversion::convertPoint(cam, sd.get_x(), sd.get_y(), cog_desired);
        vpDisplay::displayCross(I, cog_desired, 10, vpColor::green, 2);
        std::cout << "q dot: " << q_dot.t() << " in deg/s: "
                  << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;
        robot.setVelocity(jointNames, q_dot);

        {
#ifdef USE_PLOTTER
          // Add the material to plot curves

          // q normalized between (entre -1 et 1)
          for (unsigned int i=0 ; i < numJoints ; i++) {
            data[i] = (q[i] - Qmiddle[i]) ;
            data[i] /= (Qmax[i] - Qmin[i]) ;
            data[i]*=2 ;
          }
          unsigned int joint = 2;
          data[numJoints] = 2*(tQmin[joint]-Qmiddle[joint])/(Qmax[joint] - Qmin[joint]) ;
          data[numJoints+1] = 2*(tQmax[joint]-Qmiddle[joint])/(Qmax[joint] - Qmin[joint]) ;
          data[numJoints+2] = -1 ; data[numJoints+3] = 1 ;

          plot.plot(0, iter, data); // plot q, Qmin, Qmax, tQmin, tQmax
          plot.plot(1, iter, q_dot); // plot joint velocities applied to the robot
          iter ++;
#endif //#ifdef USE_PLOTTER
        }

      }
      else {
        std::cout << "Stop the robot..." << std::endl;
        robot.stop(jointNames);

      }
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t_0 << " ms" << std::endl;

    }
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;
  robot.stop(jointNames);
  g.cleanup();
  task.kill();

  return 0;
}

