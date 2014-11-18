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

#include <iostream>
#include <string>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>



using namespace AL;


void computeCentroidBlob(const vpImage<unsigned char> &I,vpDot2 &blob,vpImagePoint &cog,bool &init_done )
{
    try
    {
        if (! init_done)
        {
            vpImagePoint germ;
            vpDisplay::displayCharString(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);
            if (vpDisplay::getClick(I, germ, false))
            {
                blob.initTracking(I, germ);
                blob.track(I);
                cog = blob.getCog();
                init_done = true;
            }
        }
        else
        {
            blob.track(I);
            cog = blob.getCog();

        }

        std::cout << "init done: " << init_done << std::endl;
    }

    catch(...)
    {
        init_done = false;
    }

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
    vpCameraParameters cam;
    //cam.initPersProjWithoutDistortion(323.2023246,323.6059094,169.0936523, 119.5883104);
    cam.initPersProjWithoutDistortion(342.82,342.60,174.552518, 109.978367);

    /** Initialization Visp blob*/
    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);
    vpImagePoint cog;



    bool init_done = false;




    /** Initialization Visual servoing task*/
    vpServo task; // Visual servoing task
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

    vpAdaptiveGain lambda(2, 0.8, 30); // lambda(0)=2, lambda(oo)=0.1 and lambda_dot(0)=10
    task.setLambda(lambda);
    //task.setLambda(0.8);

    vpColVector q_dot;

    // Transformation HeadRoll to Camera Left
    vpHomogeneousMatrix cMe;

    cMe = robot.get_cMe("CameraLeft_aldebaran");



    // Motion

    std::vector<std::string> jointNames =  robot.getBodyNames("Head");
    const unsigned int numJoints = jointNames.size();

    // Declarate Jacobian
    vpMatrix eJe(6,numJoints);

    //Set the stiffness
    robot.setStiffness(jointNames, 1.f);

    double tinit = 0; // initial time in second

    vpImage<vpRGBa> O;

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

          computeCentroidBlob(I,blob,cog, init_done);
            if (! init_done)
                tinit = vpTime::measureTimeSecond();

            if (init_done) {

                std::cout << "Centroid: X = " << cog.get_i() << ". Y =  " << cog.get_j() << std::endl;
                double x=0,y=0;
                vpPixelMeterConversion::convertPoint(cam, cog, x, y);
                s.buildFrom(x, y, Z);

                eJe = robot.get_eJe("Head");
                task.set_eJe(eJe);
                task.set_cVe( vpVelocityTwistMatrix(cMe) );

                q_dot = task.computeControlLaw(vpTime::measureTimeSecond() - tinit);


                task.print();
                vpImagePoint cog_desired;
                vpMeterPixelConversion::convertPoint(cam, sd.get_x(), sd.get_y(), cog_desired);
                vpDisplay::displayCross(I, cog_desired, 10, vpColor::green, 2);

                std::cout << "q dot: " << q_dot.t() << " in deg/s: "
                          << vpMath::deg(q_dot[0]) << " " << vpMath::deg(q_dot[1]) << std::endl;

                robot.setVelocity(jointNames, q_dot);

                // Compute the distance in pixel between the target and the center of the image
                double distance = vpImagePoint::distance(cog_desired, cog);

                if (distance < 0.03*I.getWidth() && speech) // 3 % of the image witdh
                {
                  /** Call the say method */
                  tts.post.say(phraseToSay);
                  speech = false;

                }
                else if (distance > 0.15*I.getWidth()) // 15 % of the image witdh
                  speech = true;
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

