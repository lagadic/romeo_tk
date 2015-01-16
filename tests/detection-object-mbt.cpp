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

#include <vpRomeoTkConfig.h>


int main(int argc, char ** argv) {
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
  //! [MBT code]
  try {
    //    std::string videoname = "teabox.mpg";

    //    for (int i=0; i<argc; i++) {
    //      if (std::string(argv[i]) == "--name")
    //        videoname = std::string(argv[i+1]);
    //      else if (std::string(argv[i]) == "--help") {
    //        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
    //        return 0;
    //      }
    //    }
    //    std::string parentname = vpIoTools::getParent(videoname);
    //    std::string opt_model = vpIoTools::getNameWE(videoname);

    //    if(! parentname.empty())
    //       opt_model = parentname + "/" + opt_model;

    //    std::cout << "Video name: " << videoname << std::endl;
    //    std::cout << "Tracker requested config files: " << opt_model
    //              << ".[init,"
    //#ifdef VISP_HAVE_XML2
    //              << "xml,"
    //#endif
    //              << "cao or wrl]" << std::endl;
    //    std::cout << "Tracker optional config files: " << opt_model << ".[ppm]" << std::endl;

    //    vpImage<unsigned char> I;
    //    vpCameraParameters cam;
    //    vpHomogeneousMatrix cMo;

    //    vpVideoReader g;
    //    g.setFileName(videoname);
    //    g.open(I);

    //#if defined(VISP_HAVE_X11)
    //    vpDisplayX display;
    //#elif defined(VISP_HAVE_GDI)
    //    vpDisplayGDI display;
    //#elif defined(VISP_HAVE_OPENCV)
    //    vpDisplayOpenCV display;
    //#else
    //    std::cout << "No image viewer is available..." << std::endl;
    //    return 0;
    //#endif

    //    display.init(I, 100, 100,"Model-based edge tracker");


    std::string opt_ip;

    std::string opt_folder = std::string(ROMEOTK_DATA_FOLDER) + "/milkbox/";
    std::string opt_model = opt_folder + "milkbox";



    bool opt_learning = false;


    for (unsigned int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--ip")
        opt_ip = argv[i+1];
      else if (std::string(argv[i]) == "--model")
        opt_model = std::string(argv[i+1]);
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
    vpHomogeneousMatrix eMc = g.get_eMc();
    g.open();
    std::cout << "Camera parameters: " << g.getCameraParameters() << std::endl;

    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX display(I);
    vpDisplay::setTitle(I, "ViSP viewer");
    g.acquire(I);

    vpHomogeneousMatrix cMo;
    vpPoseVector cPo;

    vpMbEdgeTracker tracker;

    if(vpIoTools::checkFilename(opt_model + ".xml")) {
      tracker.loadConfigFile(opt_model + ".xml");
    }

    tracker.setCameraParameters(cam);
    tracker.setOgreVisibilityTest(false);
    if(vpIoTools::checkFilename(opt_model + ".cao"))
      tracker.loadModel(opt_model + ".cao");
    else if(vpIoTools::checkFilename(opt_model + ".wrl"))
      tracker.loadModel(opt_model + ".wrl");
    tracker.setDisplayFeatures(true);

    //! [MBT code]


    bool click_done = false;


    //! [Keypoint selection]
    std::string detectorName = "FAST";
    std::string extractorName = "ORB";
    std::string matcherName = "BruteForce-Hamming";
    std::string configurationFile = opt_folder + "detection-config.xml";

#if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
    detectorName = "SIFT";
    extractorName = "ORB";
    matcherName = "BruteForce";
    configurationFile = opt_folder + "detection-config-SIFT.xml";
#endif
    double elapsedTime;




    if (opt_learning)
    {

      vpKeyPoint keypoint_learning;
      //! [Keypoint declaration]
      //! [Keypoint xml config]
#ifdef VISP_HAVE_XML2
      keypoint_learning.loadConfigFile(configurationFile);
#endif

      for (unsigned int i = 0;i<3; i++)
      {

        while(1)
        {
          g.acquire(I);
          vpDisplay::display(I);
          vpDisplay::displayText(I, 10, 10, "Click when you are ready to learn", vpColor::red);
          vpDisplay::flush(I);

          if (vpDisplay::getClick(I, false)) {
            click_done = true;
            break;
          }

        }

        tracker.initClick(I, opt_model + ".init", true);
        //! [Keypoint declaration and initialization]

        //! [Keypoints reference detection]
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
        //! [Keypoints reference detection]

        //! [Keypoints selection on faces]
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
        polygons = pair.first;
        roisPt = pair.second;

        std::vector<cv::Point3f> points3f;
        tracker.getPose(cMo);
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
        //! [Keypoints selection on faces]

        //! [Keypoints build reference]
        keypoint_learning.buildReference(I, trainKeyPoints, points3f,true);
        //! [Keypoints build reference]

        //! [Display reference keypoints]

        for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
          vpDisplay::displayCross(I, it->pt.y, it->pt.x, 4, vpColor::red);
        }
        //vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
        vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
        vpDisplay::flush(I);
        vpDisplay::getClick(I, true);

      }


      //! [Save learning data]
      keypoint_learning.saveLearningData("teabox_learning_data.bin", true);

    }
    // END LEARNING


    //! [Init keypoint detection]
    vpKeyPoint keypoint_detection;
#ifdef VISP_HAVE_XML2
    keypoint_detection.loadConfigFile(configurationFile);
#endif

    //! [Load teabox learning data]
    keypoint_detection.loadLearningData("teabox_learning_data.bin", true);
    //! [Load teabox learning data]

    double error;
    unsigned int counter_detection = 0;
    vpMatrix Poses;

    while(counter_detection < 10) {
      g.acquire(I);
      vpDisplay::display(I);

      vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

      //! [Matching and pose estimation]
      if(keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
        //! [Matching and pose estimation]

        //! [Tracker set pose]
        tracker.setPose(I, cMo);
        //! [Tracker set pose]
        //! [Display]
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
        //! [Display]
      }

      cPo.buildFrom(cMo);
      Poses.stackMatrices(cPo.t());

      // vpDisplay::displayText(I, 30, 10, "A click to start the tracking.", vpColor::red);
      vpDisplay::flush(I);
      //      if (vpDisplay::getClick(I, false)) {
      //        click_done = true;
      //        break;
      //      }

      counter_detection ++;
    }


    std::cout<<Poses<<std::endl;

     vpPoseVector cMo_f;

     for (unsigned int i = 0; i<6; i++)
     {
     std::cout<< "vector : " << i << std::endl << Poses.getCol(i)<<std::endl;
     cMo_f[i] = vpColVector::median( Poses.getCol(i));

     }

   std::cout<< "rislutato " << std::endl << cMo_f<<std::endl;

   vpHomogeneousMatrix cMo_f_;


   cMo_f_.buildFrom(cMo_f);


    tracker.setPose(I,cMo_f_);


    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      try {
        tracker.track(I);
        tracker.getPose(cMo);
        //printPose("cMo teabox: ", cMo_teabox);
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

      }
      catch(...) {

        std::cout << "error";
      }
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) {
        click_done = true;
        break;
      }

    }









    if (! click_done)
      vpDisplay::getClick(I);






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
