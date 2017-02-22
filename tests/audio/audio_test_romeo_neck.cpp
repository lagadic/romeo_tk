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
 * This example demonstrates how to control the robot remotely in position and velocity.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/

/*! \example speech_recognition.cpp */
#include <iostream>
#include <string>
#include<fstream>
//#include <alproxies/almemoryproxy.h>
#include <math.h>
#include "robot/Experiment/experiment.h"
#include <visp/vpPlot.h>
#include "RtAudio.h"

#include <visp_naoqi/vpNaoqiRobot.h>

using namespace std;
int loop=0;
bool rec=true;
int nbframe=8;
vpMatrix record_data;
RtAudio audio;
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H.%M.%S", &tstruct);

    return buf;
}

int record(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames,
           double streamTime, RtAudioStreamStatus status, void *userData)
{


    //cout << "record time " << streamTime << endl;
    //cout << "record inputBuffer " << inputBuffer << endl;
    //cout << "record nBufferFrames " << nBufferFrames << endl;
    //cout << "record status " << status << endl;

    //#1 - Let's create the float data for processing
    int nb_channels=2;
    int framesize=256;

    //AudioCapture *capture = reinterpret_cast<AudioCapture*>(userData);
    //float* val=(float*) inputBuffer;
    // Since the number of input and output channels is equal, we can do
    // a simple buffer copy operation here.
    //if ( status ) std::cout << "Stream over/underflow detected." << std::endl;
    //std::cout << "inputBuffer:" <<inputBuffer<< std::endl;
    //std::cout << "loop:"<<loop<<endl ;
    //std::cout << "nbframe:"<<nbframe<<endl ;
    //std::cout << "rec:"<<rec<<endl ;
    if(inputBuffer!=0 && rec==true && loop<nbframe){
        //std::cout << "nBufferFrames:" <<nBufferFrames<< std::endl;
        if(nBufferFrames==256){
            short* frames=(short*)inputBuffer;


            for (unsigned int channel = 0; channel < nb_channels; channel++)
            {
                for (unsigned int frame_index = 0; frame_index < framesize; frame_index++)
                {

                    record_data[channel][frame_index+loop*framesize] = ((float) frames[channel + (nb_channels * frame_index)])/32768.0;
                }

            }
            loop++;
            //std::cout << "loop:" <<loop<< std::endl;
        }
        //for(int i=0; i<256;i++)
        //std::cout << "loop2:" <<loop<< std::endl;
    }


    return 0;

}

double hamming(int val, int N){
    float alpha=0.54;
    float beta=0.46;
    return(alpha-beta*cos((2.*M_PI*val*1.0)/(N*1.0-1.)));
}

vpRowVector sgn(vpRowVector sig){
    vpRowVector output;
    output.resize(sig.size());
    for(int i=0;i<sig.size();i++){
        if(sig[i]>0)
            output[i]=1;
        else
            output[i]=-1;
    }
    return output;
}

vpRowVector absDiff(vpRowVector sig){
    vpRowVector output,shift;
    output.resize(sig.size());
    shift.resize(sig.size());
    for(int i=0;i<sig.size()-1;i++)
        shift[i+1]=sig[i];
    vpRowVector s_sig= sgn(sig);
    vpRowVector s_shift= sgn(shift);
    for(int i=0;i<sig.size()-1;i++)
        output[i]= fabs(s_sig[i]-s_shift[i]);

    //std::cout << "output:" << output<< std::endl;
    return output;

}

vpRowVector shortTimeEnergy(vpRowVector signal, int wlen){
    vpRowVector output, extSignal,Ener;
    output.resize(signal.size());
    extSignal.resize(signal.size()+wlen);
    Ener.resize(signal.size()+wlen);
    vpRowVector zc=absDiff(signal);
    for(int i=0;i<signal.size();i++){
        extSignal[wlen/2-1+i]=zc[i];
        Ener[wlen/2-1+i]=signal[i];
    }

    double sum=0;
    for(int i=0;i<signal.size();i++){
        for(int j=0;j<wlen;j++){
            //sum+=hamming(j,wlen)*extSignal[i+wlen/2-1+j]*Ener[i+wlen/2-1+j]*hamming(j,wlen)*extSignal[i+wlen/2-1+j]*Ener[i+wlen/2-1+j];
            sum+=hamming(j,wlen)*Ener[i+wlen/2-1+j]*hamming(j,wlen)*Ener[i+wlen/2-1+j];
            //std::cout << "extSignal = " << extSignal[i+wlen/2-1+j]<< std::endl;
        }
        output[i]=sum;
        sum=0;
    }


    return output;
}

void startRecording(){
    string deviceName="hw:8 Sounds USB Audio 2.0,0";
    unsigned int bufferBytes, bufferFrames = 256;
    RtAudio::StreamParameters parameters;
    // Determine the number of devices available
    unsigned int devices = audio.getDeviceCount();
    // Scan through devices for various capabilities
    RtAudio::DeviceInfo info;
    for ( unsigned int i=0; i<devices; i++ ) {
        info = audio.getDeviceInfo( i );
        if ( info.probed == true ) {
            // Print, for example, the maximum number of output channels for each device

            //std::cout << ": maximum output channels = " << info.outputChannels << "\n";
            if(strcmp(info.name.c_str(),deviceName.c_str())==0){
                parameters.deviceId =i;
                std::cout << "chosen device = " << info.name << std::endl;
            }
        }

    }

    parameters.nChannels = 2;
    parameters.firstChannel = 0;
    unsigned int sampleRate = 48000;
    RtAudio::StreamOptions options;

    try {
        audio.openStream(  NULL,  &parameters, RTAUDIO_SINT16,
                           sampleRate, &bufferFrames, &record,(void *)&bufferBytes,&options );
        audio.startStream();
    }
    catch ( RtAudioError& e ) {
        std::cout << '\n' << e.getMessage() << '\n' << std::endl;
        exit( 0 );
    }





}


void writemat(string filename, vpMatrix data){
    ofstream myfile(filename.c_str());
    if (myfile.is_open()){
        for(int i=0;i< data.getRows();i++){
            for(int j=0; j<data.getCols();j++){
                if(j<data.getCols()-1)
                    myfile<<data[i][j]<<" ";
                else
                    myfile<<data[i][j];
            }
            myfile<<endl;
        }
        myfile.close();
    }
    else
        cout<<" Unable to open file";
}


float getRatio(AL::ALValue l, AL::ALValue r, int treshold, int nb_slices){
    float sum_r,sum_l;
    sum_l=0;
    sum_r=0;
    for(int i=0;i<nb_slices;i++)
        if(float(l[i])>treshold || float(r[i])>treshold ){
            sum_l+=float(l[i]);
            sum_r+=float(r[i]);
        }
    // std::cout << "l: " << l[0] << std::endl;
    if(sum_r>1 && sum_l>1){
        float ratio=sum_r/sum_l;
        //std::cout << "ratio: " << ratio << std::endl;
        return ratio;
    }
    return -1;
}


float computeRatio(float l, float r, float treshold){

    if(l>treshold || r>treshold){
        float ratio=r/l;
        //std::cout << "ratio: " << ratio << std::endl;
        return ratio;
    }
    return -1;
}

/*!

   Connect toRomeo robot, and apply some motion.
   By default, this example connect to a robot with ip address: 198.18.0.1.
   If you want to connect on an other robot, run:

   ./motion --ip <robot ip address>

   Example:

   ./motion --ip 169.254.168.230
 */
int main(int argc, const char* argv[])
{
    try
    {
        std::string opt_ip = "198.18.0.1";;

        if (argc == 3) {
            if (std::string(argv[1]) == "--ip")
                opt_ip = argv[2];
        }

        vpNaoqiRobot robot;
        if (! opt_ip.empty()) {
            std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
            robot.setRobotIp(opt_ip);
        }

        robot.open();

        bool servoing = true;

        std::string nameNeckYaw = "NeckYaw";

        // Define values
        float y = 1.0; //
        float l = 2.0; //
        float d = 0.13; //
        float x = 0.0;
        float L = 0.0;

        //float lambda = 0.03;
        float lambda = 0.03;
        float E_left;
        float E_right;
        float s_star = 1.0;
        float err;

        vpColVector vel(1);

        AL::ALMemoryProxy memProxy(opt_ip, 9559);

        bool start = false;
        bool pred=false;
        // Previuos ratio and velocity
        float ratio_prev = 0.;
        float vel_prev = 0.;
        float delta_t = 0.170;


        /*vpPlot plotter (2);
        plotter.initGraph(0, 1);
        plotter.initGraph(1, 1);
        //plotter_eyes.initGraph(2, 1);
        //plotter_eyes.initGraph(3, 1);
        plotter.setTitle(0,  "Ratio");
        plotter.setTitle(1,  "Ratio sound");
        //plotter_eyes.setTitle(2,  "REyeYaw");
        //plotter_eyes.setTitle(3,  "REyePitch");


        vpPlot plotter_e (1);
        plotter_e.initGraph(0, 2);

        plotter_e.setLegend(0, 0, "Right");
        plotter_e.setLegend(0, 1, "Left");

        plotter_e.setTitle(0,  "Energy");*/
        experiment exp(1,0.31,48000);
        experiment::rec=true;

        double t_start,t_end;
        string deviceName="hw:8 Sounds USB Audio 2.0,0";
        RtAudio audio; unsigned int bufferBytes, bufferFrames = 256;
        RtAudio::StreamParameters parameters;
        // Determine the number of devices available
        unsigned int devices = audio.getDeviceCount();
        // Scan through devices for various capabilities
        RtAudio::DeviceInfo info;
        for ( unsigned int i=0; i<devices; i++ ) {
            info = audio.getDeviceInfo( i );
            if ( info.probed == true ) {
                // Print, for example, the maximum number of output channels for each device

                //std::cout << ": maximum output channels = " << info.outputChannels << "\n";
                if(strcmp(info.name.c_str(),deviceName.c_str())==0){
                    parameters.deviceId =i;
                    std::cout << "chosen device = " << info.name << std::endl;
                }
            }

        }


        //RtAudio::StreamParameters oParams;
        //oParams.deviceId = 0; // first available device
        //oParams.nChannels = 2;
        parameters.nChannels = 2;
        parameters.firstChannel = 0;
        unsigned int sampleRate = 48000;
        RtAudio::StreamOptions options;
        //options.flags = RTAUDIO_NONINTERLEAVED;


        try {
            audio.openStream(  NULL,  &parameters, RTAUDIO_SINT16,
                               sampleRate, &bufferFrames, &experiment::record,(void *)&bufferBytes,&options );
            audio.startStream();
        }
        catch ( RtAudioError& e ) {
            std::cout << '\n' << e.getMessage() << '\n' << std::endl;
            exit( 0 );
        }
        vec obs;
        //recording
        /*while(loop<nbframe){
            std::cout << "" ;
        }
        rec=false;
        writemat("right_speech2.txt",record_data.transpose());*/
        int initLoop=0;
        while(initLoop<40){
            std::cout << "" ;
            if(experiment::iter==experiment::nbframe){
                    //audio.stopStream();
                    experiment::rec=false;
                    t_start=vpTime::measureTimeMs();
                    obs=exp.computeTau(0);
                    t_end=vpTime::measureTimeMs();
                    //std::cout <<"t-compute:"<< t_end-t_start<<std::endl;
                    //saveInit.ins_row(saveInit.rows(),obs);
                    std::cout <<"obs"<< acos(obs*343./0.31)*180./pi<<std::endl;
                    exp.obsTracking(obs);
                    t_end=vpTime::measureTimeMs();
                    //std::cout <<"t-track:"<< t_end-t_start<<std::endl;
                    experiment::iter=0;
                    experiment::rec=true;
                    //audio.startStream();
                    initLoop++;
                }
            }
            std::cout << "END INIT" <<std::endl;

            exp.selectBestTau(initLoop);
            exp.initTau();
            exp.initEll();
            int nb=0;
            //std::cout <<"v"<<exp.compute(obs,(t_end-t_start)/1000.)<<std::endl;
            while (nb<400){//nb<150){  //!exp.isFinished()
                t_start=vpTime::measureTimeMs();
                std::cout << "" ;
                if(experiment::iter==experiment::nbframe){
                    // audio.stopStream();
                    experiment::rec=false;
                    vec obs=exp.computeTau(nb);
                    std::cout <<"obs"<< acos(obs*343./0.31)*180./pi<<std::endl;
                    //saveTDOA.ins_row(saveTDOA.rows(),exp.getTau());
                    //saveObs.ins_row(saveObs.rows(),obs);
                    t_end=vpTime::measureTimeMs();
                    robot.setVelocity(nameNeckYaw,exp.compute(obs,(t_end-t_start)/1000.));
                    experiment::iter=0;
                    std::cout << "tau16khz = " << acos(exp.getTau()*343./0.31)*180./pi << std::endl;
                    //std::cout << "ctau16khz = " << acos(exp.ctau*343./0.31)*180./pi << std::endl;
                    //std::cout << "btau16khz = " << acos(exp.bestTau*343./0.31)*180./pi << std::endl;
                    experiment::rec=true;
                    // audio.startStream();
                    nb++;
                }
            }

            /*while(i<2000){
            //std::cout << "loop1:" << loop<< std::endl;
            //std::cout << "nbframe:" << nbframe<< std::endl;
            std::cout << "" ;
            if(loop==nbframe){
                rec=false;
                E_right=shortTimeEnergy(record_data.getRow(0),512).sum();
                E_left=shortTimeEnergy(record_data.getRow(1),512).sum();
                ratio=computeRatio(E_left,E_right,tresh);
                //std::cout << "ratio:" << ratio<< std::endl;
                std::cout << "E1:" << E_left<< std::endl;
                std::cout << "Er:" << E_right<< std::endl;
                std::cout << "ratio:" << ratio<< std::endl;
                if (ratio==-1)//|| ratio > 3.0)
                {

                    if(start)
                        ratio = 25*L*vel_prev*delta_t + ratio_prev;
                    else
                        ratio=1;
                    //ratio = 1;
                    pred=true;


                    std::cout << "Ratio pred: " << ratio << std::endl;
                }
                else
                {
                    pred=false;
                    start=true;
                    //std::cout << "Ratio2:" << ratio << std::endl;
                }
                if (ratio >= 1.)
                    x = 1;
                else
                    x = -1;

                // Compute Interaction matrix
                L = (y * d * (ratio+1)) / (l*l + d*d/4 - d*x);

                //lambda = 0.005;

                if(ratio>=s_star)
                    err=(ratio - s_star);
                else
                    err=-(1./ratio - s_star);
                err=(ratio - s_star);
                lambda=0.05;
                //lambda = 0.025/(1+exp(-1.75*(sqrt(err*err)-2.)));
                //Compute joint velocity NeckYaw
                vel[0] = - lambda/L * err;

                //std::cout << "err: " << err << std::endl;

                //robot.setVelocity(nameNeckYaw,vel);
                // Save current values
                ratio_prev = ratio;
                vel_prev = vel[0];
                //vpTime::sleepMs(130);
                loop=0;
                rec=true;
                i++;
            }
        }*/

            /* while (1)
        {
            std::cout << "----------------------------------" << std::endl;
            double t = vpTime::measureTimeMs();
            float ratio = memProxy.getData("ALSoundProcessing/ratioRightLeft");
            float sound_detected = memProxy.getData("ALSoundProcessing/soundDetectedEnergy");
            AL::ALValue left=memProxy.getData("ALSoundProcessing/leftEnergyVec");
            AL::ALValue right=memProxy.getData("ALSoundProcessing/rightEnergyVec");
            std::cout << "SoundDetected:" << sound_detected << std::endl;

            if ( (sound_detected > 0.5) && !start)
                start = true;

            if (start)
            {

                plotter_e.plot(0, 0,loop_iter, float( memProxy.getData("ALSoundProcessing/rightMicEnergy")));
                plotter_e.plot(0, 1,loop_iter, float(  memProxy.getData("ALSoundProcessing/leftMicEnergy")));
                ratio=getRatio(left,right,3e7,19);

                if (ratio==-1)//|| ratio > 3.0)
                {

                    ratio = 10*L*vel_prev*delta_t + ratio_prev;
                    //ratio = 1;
                    pred=true;
                    plotter.plot(1, 0, loop_iter,0);


                    std::cout << "Ratio pred: " << ratio << std::endl;
                }
                else
                {
                    plotter.plot(1, 0, loop_iter,ratio);
                    pred=false;
                    std::cout << "Ratio2:" << ratio << std::endl;
                }

                plotter.plot(0, 0, loop_iter,ratio);


                if (ratio >= 1.)
                    x = 1;
                else
                    x = -1;

                // Compute Interaction matrix
                L = (y * d * (ratio+1)) / (l*l + d*d/4 - d*x);

                //lambda = 0.005;

                if(ratio>=s_star)
                    err=(ratio - s_star);
                else
                    err=-(1./ratio - s_star);
                lambda = 0.025/(1+exp(-2.75*(sqrt(err*err)-2.)));
                //Compute joint velocity NeckYaw
                vel[0] = - lambda/L * err;

                std::cout << "err: " << err << std::endl;

                if (servoing)
                    robot.setVelocity(nameNeckYaw,vel);

            }

            // Save current values
            ratio_prev = ratio;
            vel_prev = vel[0];

            loop_iter ++;

            vpTime::sleepMs(130);

            //std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

        }*/

            robot.stop(nameNeckYaw);

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

