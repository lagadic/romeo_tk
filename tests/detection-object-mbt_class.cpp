//! \example tutorial-detection-object-mbt.cpp
//!
//!

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpVideoReader.h>
#include <visp/vpKeyPoint.h>
#include <visp/vpImage.h>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpMBDetection.h>

#include <vpRomeoTkConfig.h>



bool checkValiditycMo(vpHomogeneousMatrix cMo)
{

  double x = cMo[0][3];
  double y = cMo[1][3];
  double z = cMo[2][3];


  //std::cout << "x: " << x <<". Limits: -0.40 > y > 0.40" << std::endl;
  //std::cout << "y: " << y <<". Limits: -0.50 > y > 0.50" << std::endl;
  //std::cout << "z: " << z <<". Limits: 0.10 > y > 0.40" << std::endl;
  if (z < 0.10 || z > 0.40
      || x < - 0.20 || x > 0.10
      || y < -0.10 || y > 0.10 )
    return false;
  else
    return true;
}


int main(int argc, char ** argv) {
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
  //! [MBT code]
  try {

    std::string opt_ip;

    std::string opt_folder = std::string(ROMEOTK_DATA_FOLDER) + "/milkbox/";
    std::string opt_model = opt_folder + "milkbox";
    std::string opt_learning_data_file_name = "teabox_learning_data_test.bin";

    bool opt_learning = false;


    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "--model")
        opt_model = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--learning-data-file-name")
        opt_learning_data_file_name = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--learning")
        opt_learning = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << "[--ip <robot address>] [--model <path to mbt cao model>]" << std::endl;
        std::cout << "       [--learning ] [--help]" << std::endl;
        return 0;
      }
    }

    vpNaoqiGrabber g;
    g.setCamera(0); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }

    g.setFramerate(15);
    g.setCamera(0);
    vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithoutDistortion);
    g.open();
    std::cout << "Camera parameters: " << g.getCameraParameters() << std::endl;

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX display(I);
    vpDisplay::setTitle(I, "ViSP viewer");
    g.acquire(I);


    vpHomogeneousMatrix cMo;
    vpPoseVector cPo;

    vpMBDetection tracker_box(opt_model, opt_folder, cam);

    bool click_done = false;
    vpMouseButton::vpMouseButtonType button;


    // Learning phase
    if (opt_learning)
    {

      while(1)
      {
        g.acquire(I);
        vpDisplay::display(I);
        vpDisplay::displayText(I, 10, 10, "Click when you are ready to learn", vpColor::red);
        vpDisplay::flush(I);

        click_done = vpDisplay::getClick(I, button, false);

        if (click_done && button == vpMouseButton::button1) {
          click_done = false;
          tracker_box.learnObject(I);
          vpDisplay::displayText(I, 30, 10, "Click left to continue with detection...", vpColor::red);
          vpDisplay::displayText(I, 30, 30, "Click right to stop...", vpColor::red);
          vpDisplay::flush(I);
          vpDisplay::getClick(I, true);
        }

        else if (click_done && button == vpMouseButton::button3)
        {
          tracker_box.saveLearningData(opt_learning_data_file_name);
          click_done = false;
          break;
        }

      }
    }
    // END LEARNING

    //Start Detection:

    tracker_box.initDetection(opt_learning_data_file_name);

    tracker_box.setValiditycMoFunction(checkValiditycMo);


    vpPoseVector r;

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      click_done = vpDisplay::getClick(I, button, false);
      try {
        if (tracker_box.track(I))
        {
        cMo = tracker_box.get_cMo();
        r.buildFrom(cMo);
        std::cout << "Pose:" << std::endl;
        std::cout << r.t() << std::endl;
        }

      }
      catch(...) {
      }
      vpDisplay::flush(I);

      if (click_done && button == vpMouseButton::button3) {
        click_done = false;
        break;
      }
      else if (click_done && button == vpMouseButton::button1)
      {
        click_done = false;
        tracker_box.setForceDetection();
      }
      else if(click_done && button == vpMouseButton::button2)
      {
        click_done = false;
        tracker_box.setManualDetection();
        tracker_box.setForceDetection();
      }

    }




    if (0)
    {

      unsigned int counter_detection = 0;
      vpMatrix Poses;



      while(counter_detection < 6) {
        g.acquire(I);
        vpDisplay::display(I);
        vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

        //Matching and pose estimation
        if(tracker_box.detectObject(I,cMo))
        {
          //! [Tracker set pose]
          tracker_box.getTracker()->setPose(I, cMo);

          //! [Display]
          tracker_box.getTracker()->display(I, cMo, cam, vpColor::red, 2);
          vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
          if (checkValiditycMo(cMo))
          {
            cPo.buildFrom(cMo);
            Poses.stackMatrices(cPo.t());
            counter_detection ++;
          }
        }

        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      }


      std::cout<<Poses<<std::endl;

      vpPoseVector cMo_f;

      for (unsigned int i = 0; i<6; i++)
        cMo_f[i] = vpColVector::median( Poses.getCol(i));

      std::cout<< "Result: " << std::endl << cMo_f<<std::endl;

      vpHomogeneousMatrix cMo_f_;
      cMo_f_.buildFrom(cMo_f);

      tracker_box.getTracker()->setPose(I,cMo_f_);


      while(1) {
        g.acquire(I);
        vpDisplay::display(I);
        try {
          tracker_box.getTracker()->track(I);
          tracker_box.getTracker()->getPose(cMo);
          //printPose("cMo teabox: ", cMo_teabox);
          //        if (!checkValiditycMo(cMo))


          tracker_box.getTracker()->display(I, cMo, cam, vpColor::red, 2);
          vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

        }
        catch(...) {


        }
        vpDisplay::flush(I);

        if (vpDisplay::getClick(I, false)) {
          click_done = true;
          break;
        }

      }
    }

    //    if (! click_done)
    //      vpDisplay::getClick(I);



#ifdef VISP_HAVE_XML2
    vpXmlParser::cleanup();
#endif

#if defined(VISP_HAVE_COIN) && (COIN_MAJOR_VERSION == 3)
    SoDB::finish();
#endif




  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif

  return 0;
}
