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

#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPlot.h>

//VispNaoqi
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

//RomeoTk
#include <vpRomeoTkConfig.h>
#include <vpBlobsTargetTracker.h>
#include <vpServoHead.h>

using namespace cv;



int main(int argc, const char* argv[])
{
    std::string opt_ip = "198.18.0.1";

    std::vector<std::string> opt_names;
    std::string opt_name_file;
    std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);

    unsigned int num_objects = 0;


    bool opt_learning = false;

    for (unsigned int i=0; i<argc; i++) {
        if (std::string(argv[i]) == "--ip")
            opt_ip = argv[i+1];
        else if (std::string(argv[i]) == "--learn-color")
            opt_learning = true;
        else if (std::string(argv[i]) == "--help") {
            std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--learn-color] [--object_name <numberObjects> <name1 name2 ...>]" << std::endl;
            std::cout <<                         "[--file_name <path>]" << std::endl;
            return 0;
        }
    }

    // opt_name_file = opt_name + "HSV.txt";


    opt_names.push_back("larm");
    opt_names.push_back("rarm");

    std::cout <<"Names " << opt_names << std::endl;
    num_objects = opt_names.size();

    if (opt_name_file.empty())
        // opt_name_file = "redHSV.txt";
        opt_name_file = "../data/target/LArm/color.txt";

    //std::vector<vpColor> color_rects(num_objects);

    try
    {
        // GRABBER
        vpNaoqiGrabber g;
        g.setCamerasMulti(1); // eyes cameras
        g.setFramerate(15);

        if (! opt_ip.empty()) {
            std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
            g.setRobotIp(opt_ip);
        }
        g.openMulti();

        std::vector<std::string> cameraNames(2);
        cameraNames[0] = "CameraLeftEye";
        cameraNames[1] = "CameraRightEye";

        // Initialize constant transformations
        std::vector<vpHomogeneousMatrix> eMc(2);
        eMc[0] = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraLeftEye");
        eMc[1] = g.get_eMc(vpCameraParameters::perspectiveProjWithDistortion,"CameraRightEye");
        std::cout << "CameraLeftEye extrinsic parameters: "  << std::endl << eMc[0]  << std::endl;
        std::cout << "CameraRightEye extrinsic parameters: " << std::endl  << eMc[1] << std::endl;

        //Get cameras extrinsic parameters:
        std::vector<vpCameraParameters> cam(2);
        //Get cameras instrinsic parameters:
        cam[0] = g.getCameraParameters(AL::kQVGA,"CameraLeftEye",vpCameraParameters::perspectiveProjWithoutDistortion);
        cam[1] = g.getCameraParameters(AL::kQVGA,"CameraRightEye",vpCameraParameters::perspectiveProjWithoutDistortion);
        std::cout << "CameraLeftEye instrinsic parameters: " << std::endl << cam[0] << std::endl;
        std::cout << "CameraRightEye instrinsic parameters: " << std::endl << cam[1] << std::endl;

        vpImage<unsigned char> Ir(g.getHeight(), g.getWidth());
        vpImage<unsigned char> Il(g.getHeight(), g.getWidth());

        //  std::vector < vpImage<unsigned char>* > I;
        //  I.push_back(new vpImage<unsigned char>(g.getHeight(), g.getWidth()));
        //  I.push_back(new vpImage<unsigned char>(g.getHeight(), g.getWidth()));

        std::vector < vpImage<unsigned char> > I;
        I.push_back(Ir);
        I.push_back(Il);

        std::cout << I.size();

        vpDisplayX dl(I[0]);
        vpDisplay::setTitle(I[0], "Left camera");

        vpDisplayX dr(I[1]);
        vpDisplay::setTitle(I[1], "Rigth camera");
        vpDisplay::setWindowPosition(I[1], 2 * I[0].getHeight(), 0);


        //Initialize opencv color images
        cv::Mat cvI_l = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);
        cv::Mat cvI_r = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);

        std::vector<cv::Mat> cvI;
        cvI.push_back(cvI_l);
        cvI.push_back(cvI_r);

        // Common
        vpMouseButton::vpMouseButtonType button;
        std::vector<bool> click_done(2);

        std::vector <bool> firstTime(num_objects);


        std::vector<std::string> chain_name(2); // LArm or RArm
        chain_name[0] = "LArm";
        chain_name[1] = "RArm";


        std::string opt_name_file_color_target_path = opt_data_folder + "/" +"target/";
        std::string opt_name_file_color_target_l = opt_name_file_color_target_path + chain_name[0] +"/color.txt";
        std::string opt_name_file_color_target_r = opt_name_file_color_target_path + chain_name[1] +"/color.txt";

        std::string opt_name_file_color_target1_l = opt_name_file_color_target_path + chain_name[0] +"/color1.txt";
        std::string opt_name_file_color_target1_r = opt_name_file_color_target_path + chain_name[1] +"/color1.txt";

        std::vector<vpBlobsTargetTracker*> hand_tracker;
        std::vector<bool>  status_hand_tracker(2);
        std::vector<vpHomogeneousMatrix>  cMo_hand(2);

        const double L = 0.025/2;
        std::vector <vpPoint> points(4);
        points[2].setWorldCoordinates(-L,-L, 0) ;
        points[1].setWorldCoordinates(-L,L, 0) ;
        points[0].setWorldCoordinates(L,L, 0) ;
        points[3].setWorldCoordinates(L,-L,0) ;

        vpBlobsTargetTracker hand_tracker_l;
        hand_tracker_l.setName(chain_name[0]);
        hand_tracker_l.setCameraParameters(cam[0]);
        hand_tracker_l.setPoints(points);
        hand_tracker_l.setLeftHandTarget(true);

        hand_tracker_l.setFullManual(true);

        if(!hand_tracker_l.loadHSV(opt_name_file_color_target_l))
            std::cout << "Error opening the file "<< opt_name_file_color_target_l << std::endl;


        vpBlobsTargetTracker hand_tracker_r;
        hand_tracker_r.setName(chain_name[1]);
        hand_tracker_r.setCameraParameters(cam[1]);
        hand_tracker_r.setPoints(points);
        hand_tracker_r.setLeftHandTarget(false);

        hand_tracker_r.setFullManual(true);

        if(!hand_tracker_r.loadHSV(opt_name_file_color_target1_l))
            std::cout << "Error opening the file "<< opt_name_file_color_target_l << std::endl;

        hand_tracker.push_back(&hand_tracker_l);
        hand_tracker.push_back(&hand_tracker_r);


        /************************************************************************************************/

        /** Create a new istance NaoqiRobot*/
        vpNaoqiRobot robot;
        if (! opt_ip.empty())
            robot.setRobotIp(opt_ip);
        robot.open();


        /************************************************************************************************/


        /************************************************************************************************/
        // Initialize head-eye servoing
        std::vector<vpServoHead*> servo_head_eye;
        vpServoHead l_eye;
        vpServoHead r_eye;

        l_eye.setCameraParameters(cam[0]);
        r_eye.setCameraParameters(cam[1]);

        servo_head_eye.push_back(&l_eye);
        servo_head_eye.push_back(&r_eye);

        std::vector<vpMatrix> oJo_d(2);
        std::vector<vpColVector> e_d(2);
        std::vector<vpMatrix> L_d(2);

        //
        vpMatrix J_b; // 12x8
        vpColVector e_b;// 4x1
        vpMatrix L_b (4,12,0.0); // 4x12

        std::vector <bool> teabox_servo_converged(2);
        teabox_servo_converged[0] = false;
        teabox_servo_converged[1] = false;

        std::vector< double> servo_arm_time_init(2);
        servo_arm_time_init[0] = 0.0;
        servo_arm_time_init[1] = 0.0;


        std::vector<bool> first_time_arm_servo(2);
        first_time_arm_servo[0] = true;
        first_time_arm_servo[1] = true;

        std::vector<std::string> jointNames_head =  robot.getBodyNames("Head");

        std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
        std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

        jointNames_head.insert(jointNames_head.end(), jointNamesLEye.begin(), jointNamesLEye.end());
        std::vector<std::string> jointNames_tot = jointNames_head;
        jointNames_tot.push_back(jointNamesREye.at(0));
        jointNames_tot.push_back(jointNamesREye.at(1));
        vpColVector head_pose(jointNames_tot.size(), 0);

        //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","REyeYaw", "REyePitch" );
        AL::ALValue angles_head;
        angles_head      = AL::ALValue::array(vpMath::rad(0.0), vpMath::rad(31.0), vpMath::rad(2.6), vpMath::rad(0), vpMath::rad(14.0), vpMath::rad(3.2), vpMath::rad(-7.4), vpMath::rad(1.8)  );
        float fractionMaxSpeed  = 0.1f;
        robot.getProxy()->setAngles(jointNames_tot, angles_head, fractionMaxSpeed);


        // Map to don't consider the HeadRoll
        vpMatrix MAP_head(6,5);
        for (unsigned int i = 0; i < 3 ; i++)
            MAP_head[i][i]= 1;
        MAP_head[4][3]= 1;
        MAP_head[5][4]= 1;

        /************************************************************************************************/




        //Mat cvI = Mat(Size(g.getWidth(), g.getHeight()), CV_8UC3);
        //g.setCameraParameter(AL::kCameraAutoWhiteBalanceID,1);
        //g.setCameraParameter(AL::kCameraAutoExpositionID,1);


        //        vpPlot *plotter_qrcode_pose;

        //        plotter_qrcode_pose = new vpPlot(2, I.getHeight()*2, I.getWidth()*2, I.display->getWindowXPosition()+I.getWidth()+90, I.display->getWindowYPosition()+10, "Qrcode pose");
        //        plotter_qrcode_pose->initGraph(0, 3); // translations
        //        plotter_qrcode_pose->initGraph(1, 3); // rotations
        //        plotter_qrcode_pose->setTitle(0, "Pose translation");
        //        plotter_qrcode_pose->setTitle(1, "Pose theta u");
        //        plotter_qrcode_pose->setLegend(0, 0, "tx");
        //        plotter_qrcode_pose->setLegend(0, 1, "ty");
        //        plotter_qrcode_pose->setLegend(0, 2, "tz");
        //        plotter_qrcode_pose->setLegend(1, 0, "tux");
        //        plotter_qrcode_pose->setLegend(1, 1, "tuy");
        //        plotter_qrcode_pose->setLegend(1, 2, "tuz");

        unsigned long loop_iter = 0;

        while(1)
        {
            double t = vpTime::measureTimeMs();

            g.acquireMulti(cvI[0], cvI[1]);
            vpImageConvert::convert(cvI[0], I[0]);
            vpImageConvert::convert(cvI[1], I[1]);

            vpDisplay::display(I[0]);
            vpDisplay::display(I[1]);

            click_done[0] = false;
            click_done[1] = false;

            click_done[0] = vpDisplay::getClick(I[0], button, false);
            click_done[1] = vpDisplay::getClick(I[1], button, false);


            //      std::vector<int> result(4);
            //      result[0] = g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraContrastID);
            //      result[1] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraSaturationID);
            //      result[2] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraHueID);
            //      result[3] =g.getProxy()->getCameraParameter(g.getClientName(),AL::kCameraGainID);

            //      std::cout<< "Result constrast, sat, Hue, Gain " << std::endl << result << std::endl;



            for (unsigned int i = 0; i < num_objects ; i++)
            {

                char key[10];
                bool ret = vpDisplay::getKeyboardEvent(I[i], key, false);
                std::string s = key;
                if (ret && s== "h")
                {
                    hand_tracker[i]->setManualBlobInit(true);
                    hand_tracker[i]->setForceDetection(true);
                }


                status_hand_tracker[i] = hand_tracker[i]->track(cvI[i],I[i]);




                if (status_hand_tracker[i])  // display the tracking results
                {
                    cMo_hand[i] = hand_tracker[i]->get_cMo();
                    vpDisplay::displayFrame(I[i], cMo_hand[i], cam[i], 0.04, vpColor::none, 3);
                }



                if (status_hand_tracker[i]) {


                    vpImagePoint head_cog_cur = hand_tracker[i]->getCog();
                    vpImagePoint head_cog_des(I[i].getHeight()/2, I[i].getWidth()/2);

                    vpMatrix eJe;
                    if (i)
                        eJe = robot.get_eJe("REye");
                    else
                        eJe = robot.get_eJe("LEye");

                    std::cout << "eJe" << std::endl << eJe << std::endl;
                    servo_head_eye[i]->set_eJe( eJe );
                    servo_head_eye[i]->set_cVe( vpVelocityTwistMatrix(eMc[i].inverse()) );


                    //          vpAdaptiveGain lambda(2, 2.0, 30); // lambda(0)=2, lambda(oo)=0.8 and lambda_dot(0)=30
                    //          servo_head.setLambda(lambda);

                    servo_head_eye[i]->setCurrentFeature(head_cog_cur);
                    servo_head_eye[i]->setDesiredFeature(head_cog_des);
                    vpServoDisplay::display(servo_head_eye[i]->m_task_head, cam[i], I[i], vpColor::green, vpColor::red, 3);


                    servo_head_eye[i]->computeControlLaw();

                    L_d[i] = servo_head_eye[i]->m_task_head.getInteractionMatrix();
                    e_d[i] = servo_head_eye[i]->m_task_head.getError();

                    oJo_d[i] =  servo_head_eye[i]->m_task_head.get_cVe() * eJe;


                    // Add mirroring eyes
                    //          q_dot_tot = q_dot_head;
                    //          std::cout << "q = " << q_dot_tot << std::endl;
                    //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-2]);
                    //          q_dot_tot.stack(q_dot_head[q_dot_head.size()-1]);

                    //          robot.setVelocity(jointNames_tot, q_dot_tot);

                }

                else {
                    robot.stop(jointNames_tot);
                    // reinit_servo = true;
                }


                if (i && status_hand_tracker[0] && status_hand_tracker[1])
                {

                    // We build the new Jacobian
                    vpMatrix J_b_1l = oJo_d[0];

                    //std::cout << "J_b_1l" << std::endl << J_b_1l << std::endl;std::cout << std::endl;
                    unsigned int colJ =  J_b_1l.getCols();
                    J_b_1l.resize(J_b_1l.getRows(),colJ+2,false);

                    std::cout << "new J_b_1l" << std::endl << J_b_1l << std::endl;std::cout << std::endl;


                    vpMatrix J_b_1r = oJo_d[1];
                    // std::cout << "J_b_1r" << std::endl << J_b_1r << std::endl;std::cout << std::endl;

                    J_b_1r.resize(J_b_1r.getRows(),colJ+2,false);

                    J_b_1r.insert((J_b_1r.getCol(4,0,6)),0,6);
                    J_b_1r.insert((J_b_1r.getCol(5,0,6)),0,7);

                    vpMatrix zeroCols(6,2,0.0);

                    J_b_1r.insert(zeroCols,0,4);

                    std::cout << "new J_b_1r" << std::endl << J_b_1r << std::endl;std::cout << std::endl;

                    J_b = J_b_1l;
                    J_b.stackMatrices( J_b_1r);

                    std::cout << "J_b" << std::endl << J_b << std::endl;std::cout << std::endl;

                    // Build the new interaction matrix
                    for(unsigned int k = 0; k<L_d[1].getRows(); k++)
                    {
                        for(unsigned int p = 0; p<L_d[1].getCols(); p++)
                        {
                            L_b[k][p] = L_d[0][k][p];
                            L_b[k+L_d[1].getRows()][p+L_d[1].getCols()] = L_d[1][k][p];
                        }
                    }
                    std::cout << "L_b" << std::endl << L_b << std::endl;std::cout << std::endl;


                    // Build new error vector
                    e_b = e_d[0];
                    e_b.stack(e_d[1]);

                    std::cout << "e_b" << std::endl << e_b << std::endl;std::cout << std::endl;


                    vpColVector q = -0.5 * ( L_b * J_b).pseudoInverse() * e_b;
                    robot.setVelocity(jointNames_tot,q);
                }


                if (click_done[i] && button == vpMouseButton::button1 ) {
                    robot.stop(jointNames_tot);
                    click_done[i] = false;
                }

                vpDisplay::flush(I[i]);

            }


            if (click_done[0] && button == vpMouseButton::button3 ) { // Quit the loop
                //robot.stop(jointNames_tot);
                break;
            }

            std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
            loop_iter ++;

        }

        //delete plotter_qrcode_pose;
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


