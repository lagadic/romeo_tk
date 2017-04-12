#include <iostream>
#include <string>
#include <map>


#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

#include <alproxies/almemoryproxy.h>


// ViSP includes.
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>



/*!

  Connect to Pepper robot, grab, display images using ViSP and start
  face detection with Okao .
  More over all the two joints of Pepper's head are controlled by visual servoing to center
  the detected head in the image.
  By default, this example connect to a robot with ip address: 131.254.10.126.
  If you want to connect on an other robot, run:

  ./servo_face_detection_visp_head --ip <robot ip address>

  Example:

  ./servo_face_detection_visp_head --ip 169.254.168.230
 */



bool in_array(const std::string &value, const std::vector<std::string> &array)
{
  return std::find(array.begin(), array.end(), value) != array.end();
}

bool pred(const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs)
{
  return lhs.second < rhs.second;
}


int main(int argc, const char* argv[])
{
  std::string opt_ip = "131.254.10.126";
  bool opt_language_english = true;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--fr")
      opt_language_english = false;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--fr]" << std::endl;
      return 0;
    }
  }

  // USE NEW MODULE

  qi::SessionPtr session = qi::makeSession();
  session->connect("tcp://131.254.10.126:9559");
  qi::AnyObject proxy = session->service("pepper_control");

  proxy.call<void >("start");

  // -------------------------------------


  std::string camera_name = "CameraTopPepper";

  // Open the grabber for the acquisition of the images from the robot
  vpNaoqiGrabber g;
  if (! opt_ip.empty())
    g.setRobotIp(opt_ip);
  g.setFramerate(30);
  g.setCamera(0);
  g.open();
  vpCameraParameters cam = vpNaoqiGrabber::getIntrinsicCameraParameters(AL::kQVGA,camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  vpHomogeneousMatrix eMc = vpNaoqiGrabber::getExtrinsicCameraParameters(camera_name,vpCameraParameters::perspectiveProjWithDistortion);

  std::cout << "eMc:" << std::endl << eMc << std::endl;
  std::cout << "cam:" << std::endl << cam << std::endl;

  // Connect to the robot
  vpNaoqiRobot robot;
  if (! opt_ip.empty())
    robot.setRobotIp(opt_ip);
  robot.open();

  if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  {
    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
    return 0;
  }

  std::vector<std::string> jointNames_head = robot.getBodyNames("Head");


  vpPlot plotter_vel (1);
  plotter_vel.initGraph(0, 5);
  plotter_vel.setLegend(0, 0, "vx");
  plotter_vel.setLegend(0, 1, "vy");
  plotter_vel.setLegend(0, 2, "wz");
  plotter_vel.setLegend(0, 3, "q_yaw");
  plotter_vel.setLegend(0, 4, "q_pitch");

  vpPlot plotter_sec_vel (1);
  plotter_sec_vel.setTitle(0,"Secondary task");
  plotter_sec_vel.initGraph(0, 5);
  plotter_sec_vel.setLegend(0, 0, "vx");
  plotter_sec_vel.setLegend(0, 1, "vy");
  plotter_sec_vel.setLegend(0, 2, "wz");
  plotter_sec_vel.setLegend(0, 3, "q_yaw");
  plotter_sec_vel.setLegend(0, 4, "q_pitch");

  vpColVector v(6);
  v[0] = 0.0; // Vx
  v[1] = 0.0; // Vy
  v[2] = 0.0; // Vz
  v[3] = 0.0; // Wx
  v[4] = 0.0; // Wy
  v[5] = 0.6; // Wz

  vpColVector q_dot(6);

  try {
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");


    // Jacobian 6x5 (vx,vy,wz,q_yaq,q_pitch)
    vpMatrix tJe(6,5);
    tJe[0][0]= 1;
    tJe[1][1]= 1;
    tJe[5][2]= 1;
    vpMatrix eJe(6,5);

    unsigned long loop_iter = 0;

    vpColVector unos(5,1);
    vpMatrix Id;
    Id.diag(unos);

    AL::ALValue limit_yaw = robot.getProxy()->getLimits("HeadYaw");
    std::cout << limit_yaw[0][0] << " " << limit_yaw[0][1] << std::endl;
    double min = limit_yaw[0][0];
    double max = limit_yaw[0][1];

    double t_prev = vpTime::measureTimeSecond();


    while(1) {

      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      // Get Head Jacobian (6x2)
      vpMatrix torso_eJe_head;
      robot.get_eJe("Head",torso_eJe_head);

      // Add column relative to the base rotation (Wz)
      vpColVector col_wz(6);
      col_wz[5] = 1;
      for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < torso_eJe_head.getCols(); j++)
          tJe[i][j+3] = torso_eJe_head[i][j];

      std::cout << "tJe" << std::endl << tJe << std::endl;

      //        vpHomogeneousMatrix torsoMHeadPith( robot.getProxy()->getTransform(jointNames_head[jointNames_head.size()-1], 0, true));// get transformation  matrix between torso and HeadRoll
      vpHomogeneousMatrix torsoMHeadPith( robot.getProxy()->getTransform("HeadPitch", 0, true));// get transformation  matrix between torso and HeadRoll


      vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

      for(unsigned int i=0; i< 3; i++)
        for(unsigned int j=0; j< 3; j++)
          HeadPitchVLtorso[i][j+3] = 0;

      std::cout << "HeadPitchVLtorso: " << std::endl << HeadPitchVLtorso << std::endl;


      vpVelocityTwistMatrix cVe(vpVelocityTwistMatrix(eMc.inverse()));
      // Transform the matrix
      eJe = cVe * HeadPitchVLtorso *tJe;

      std::cout << "eJe" << std::endl << eJe << std::endl;

      q_dot = eJe.pseudoInverse() * v;

      //      task.set_eJe( eJe );
      //      task.set_cVe( vpVelocityTwistMatrix(eMc.inverse()) );

      vpMatrix P = Id - eJe.pseudoInverse()*eJe;
      double alpha = -15.5;
      std::cout << "P: " << P << "" << std::endl;

      double min = limit_yaw[0][0];
      double max = limit_yaw[0][1];

      std::cout << "min: " << min << "" << std::endl;
      std::cout << "max: " << max << "" << std::endl;

      vpColVector q_yaw = robot.getPosition(jointNames_head[0]);

      std::cout << "q3: " << jointNames_head[0] <<" " << q_yaw << std::endl;

      vpColVector z_q2 (q_dot.size());
      z_q2[3] = 2 * alpha * q_yaw[0]/ pow((max - min),2);

      vpColVector q3 = P * z_q2;
      //std::cout << "q3: " << q3 << std::endl;
      //if (q3.euclideanNorm()<10.0)
      q_dot =  q_dot + q3;


      std::vector<float> vel(jointNames_head.size());

      vel[0] = q_dot[3];
      vel[1] = q_dot[4];

      //std::cout << "q_dot" << std::endl << q_dot << std::endl;



      proxy.async<void >("setDesJointVelocity", jointNames_head, vel );
      robot.getProxy()->move(q_dot[0], q_dot[1], q_dot[2]);

//      //HACK
//      vel[0] = 0.2;
//      vel[1] = 0.0;


//      proxy.async<void >("setDesJointVelocity", jointNames_head, vel );
//      robot.getProxy()->move(0.0, 0.0, -0.2);

      vpColVector vel_head = robot.getJointVelocity(jointNames_head);
      //      for (unsigned int i=0 ; i < jointNames_head.size() ; i++) {
      //        plotter_diff_vel.plot(i, 1, loop_iter, q_dot_head[i]);
      //        plotter_diff_vel.plot(i, 0, loop_iter, vel_head[i]);
      //      }

      plotter_vel.plot(0,loop_iter, q_dot);
      plotter_sec_vel.plot(0,loop_iter, q3);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      loop_iter ++;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
    proxy.call<void >("stopJoint");
    robot.getProxy()->move(0.0, 0.0, 0.0);

    vpDisplay::getClick(I, true);



  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  std::cout << "The end: stop the robot..." << std::endl;

  proxy.call<void >("stopJoint");
  robot.getProxy()->move(0.0, 0.0, 0.0);
  proxy.call<void >("stop");



  return 0;
}





