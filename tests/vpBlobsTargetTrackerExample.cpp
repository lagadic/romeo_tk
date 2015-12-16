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
 * This example demonstrates detect and track a target with blobs
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example vpColoDetectionExample.cpp */
#include <iostream>
#include <string>

#include <alproxies/altexttospeechproxy.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Visp
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPlot.h>

#include <vpBlobsTargetTracker.h>

using namespace cv;



int main(int argc, const char* argv[])
{
    std::string opt_ip = "198.18.0.1";

    std::vector<std::string> opt_names;
    std::string opt_name_file;
    unsigned int num_objects = 0;


    bool opt_learning = false;

    for (unsigned int i=0; i<argc; i++) {
        if (std::string(argv[i]) == "--ip")
            opt_ip = argv[i+1];
        else if (std::string(argv[i]) == "--learn-color")
            opt_learning = true;
        else if (std::string(argv[i]) == "--object_name")
        {

            int num = atoi(argv[i+1]);

            for (unsigned int k = 2;k <=num+1;k++)
            {
                opt_names.push_back(std::string(argv[i+k]));
            }

        }
        else if (std::string(argv[i]) == "--file_name")
            opt_name_file = std::string(argv[i+1]);
        else if (std::string(argv[i]) == "--help") {
            std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn-color] [--object_name <numberObjects> <name1 name2 ...>]" << std::endl;
            std::cout <<                         "[--file_name <path>]" << std::endl;
            return 0;
        }
    }

    // opt_name_file = opt_name + "HSV.txt";

    if (!opt_names.size())
    {
        opt_names.push_back("larm");
        // opt_names.push_back("rarm");
    }
    std::cout <<"Names " << opt_names << std::endl;
    num_objects = opt_names.size();

    if (opt_name_file.empty())
        // opt_name_file = "redHSV.txt";
        opt_name_file = "../data/target/LArm/color.txt";

    //std::vector<vpColor> color_rects(num_objects);


    vpNaoqiGrabber g;
    if (! opt_ip.empty()) {
        std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
        g.setRobotIp(opt_ip);
    }
    g.setFramerate(15);
    g.setCamera(3);
    g.open();

    vpCameraParameters cam = g.getCameraParameters();


    //g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,0);
    //g.setCameraParameter(AL::kCameraAutoExpositionID,0);

    // g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraBrightnessID);
    //g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraContrastID);
    //g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraSaturationID);
    //g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraHueID);
    //g.getProxy()->setCameraParameterToDefault(g.getClientName(),AL::kCameraGainID);


    try
    {

        std::vector <bool> firstTime(num_objects);


        vpImage<unsigned char> I(g.getHeight(), g.getWidth());
        vpDisplayX d(I);
        vpDisplay::setTitle(I, "ViSP viewer");
        bool click_done = false;
        vpMouseButton::vpMouseButtonType button;


        std::vector <vpBlobsTargetTracker> objects (num_objects);

        const double L = 0.025/2;
        std::vector <vpPoint> points(4);
        points[2].setWorldCoordinates(-L,-L, 0) ;
        points[1].setWorldCoordinates(-L,L, 0) ;
        points[0].setWorldCoordinates(L,L, 0) ;
        points[3].setWorldCoordinates(L,-L,0) ;

        for (unsigned int i= 0; i < num_objects ; i++)
        {
            objects[i].setName(opt_names[i]);
            objects[i].setCameraParameters(cam);
            firstTime[i] = true;
            objects[i].setPoints(points);
            objects[i].setLeftHandTarget(true);
           // objects[i].setManualBlobInit(true);

            //      color_rects.at(i).id = vpColor::vpColorIdentifier( std::rand() % ( 18 + 1 ) );

            if(!objects[i].loadHSV(opt_name_file))
            {
                std::cout << "Error opening the file "<< opt_name_file << std::endl;
            }

        }


        Mat cvI = Mat(Size(g.getWidth(), g.getHeight()), CV_8UC3);
        g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,1);
        g.setCameraParameter(AL::kCameraAutoExpositionID,1);


        vpPlot *plotter_qrcode_pose;

        plotter_qrcode_pose = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Qrcode pose");
        plotter_qrcode_pose->initGraph(0, 3); // translations
        plotter_qrcode_pose->initGraph(1, 3); // rotations
        plotter_qrcode_pose->setTitle(0, "Pose translation");
        plotter_qrcode_pose->setTitle(1, "Pose theta u");
        plotter_qrcode_pose->setLegend(0, 0, "tx");
        plotter_qrcode_pose->setLegend(0, 1, "ty");
        plotter_qrcode_pose->setLegend(0, 2, "tz");
        plotter_qrcode_pose->setLegend(1, 0, "tux");
        plotter_qrcode_pose->setLegend(1, 1, "tuy");
        plotter_qrcode_pose->setLegend(1, 2, "tuz");

        unsigned long loop_iter = 0;

        while(1)
        {
            double t = vpTime::measureTimeMs();

            g.acquire(cvI);
            vpImageConvert::convert(cvI, I);
            vpDisplay::display(I);
            click_done = vpDisplay::getClick(I, button, false);

            //      std::vector<int> result(4);
            //      result[0] = g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraContrastID);
            //      result[1] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraSaturationID);
            //      result[2] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraHueID);
            //      result[3] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraGainID);

            //      std::cout<< "Result constrast, sat, Hue, Gain " << std::endl << result << std::endl;



            for (unsigned int k = 0; k < num_objects ; k++)
            {

                char key[10];
                bool ret = vpDisplay::getKeyboardEvent(I, key, false);
                 std::string s = key;
                if (ret && s== "h")
                {
                    objects[k].setManualBlobInit(true);
                    objects[k].setForceDetection(true);
                }


                bool obj_found =  objects[k].track(cvI,I);

                if (obj_found) {
                    vpHomogeneousMatrix cMo = objects[k].get_cMo();
                    vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 2);

                    cMo.print();

                    vpPoseVector p(cMo);
                    vpColVector cto(3);
                    vpColVector cthetauo(3);
                    for(size_t i=0; i<3; i++) {
                        cto[i] = p[i];
                        cthetauo[i] = vpMath::deg(p[i+3]);
                    }

                    plotter_qrcode_pose->plot(0, loop_iter, cto);
                    plotter_qrcode_pose->plot(1, loop_iter, cthetauo);




                }


            }



            vpDisplay::flush(I);

            if (click_done && button == vpMouseButton::button3) {
                click_done = false;
                g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,1);
                g.setCameraParameter(AL::kCameraAutoExpositionID,1);
                break;
            }

            std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;



            loop_iter ++;

        }


        delete plotter_qrcode_pose;
    }
    catch (const vpException &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
    catch (const AL::ALError &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    return 0;
}


