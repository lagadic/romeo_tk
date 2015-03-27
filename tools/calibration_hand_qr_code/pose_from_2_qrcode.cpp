#include <sstream>

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>


int main(int argc, const char** argv)
{

  bool opt_right_arm = 0;

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--right-arm")
      opt_right_arm = 1;
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << "[--right-arm] to use the right arm. Default: left arm" << std::endl;
      return 0;
    }
  }

  std::string name_o2Me;

  if (opt_right_arm)
  {
    std::cout << "Use Right Hand" << std::endl;
    name_o2Me = "qrcode_M_e_RArm";
  }
  else
  {
    std::cout << "Use Left Hand" << std::endl;
    name_o2Me = "qrcode_M_e_LArm";
  }


#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11) && defined(VISP_HAVE_ZBAR)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
  g.acquire(I);

  vpDisplayX d(I);

  vpDetectorQRCode detector;
  std::vector<vpPose> pose(2);

  int npoints = 4;


  std::vector<vpHomogeneousMatrix> cMo(pose.size());

  // Define coordinates points of the qr-code of different size
  std::vector<vpPoint> Pm(npoints);
  double L1 = 0.045 / 2.;
  Pm[0].setWorldCoordinates(-L1, -L1, 0);
  Pm[1].setWorldCoordinates(-L1,  L1, 0);
  Pm[2].setWorldCoordinates( L1,  L1, 0);
  Pm[3].setWorldCoordinates( L1, -L1, 0);

  std::vector<vpPoint> Ps(npoints);
  double L2 = 0.035 / 2.;
  Ps[0].setWorldCoordinates(-L2, -L2, 0);
  Ps[1].setWorldCoordinates(-L2,  L2, 0);
  Ps[2].setWorldCoordinates( L2,  L2, 0);
  Ps[3].setWorldCoordinates( L2, -L2, 0);


  /*
   *
CASE LEFT ARM:


    0       3

       qr o1 ----> x
          |
    1     |   2 = small dot
         \|/
         y


            z
           /|\
            |
            |                  qr o2 ----> x
          e ----\ x                  |
                /                    |
                                    \|/


 CASE RIGHT ARM:

                             small   /|\ y
                             dot      |
                                      |
                               <------01
                               x



                                      z
   y /|\                             /|\
      |                               |
      |                               |
<-----02                         <----e
 x                              x



  */



  // From QRcode to robot end effector
  vpHomogeneousMatrix o1Me;

  if(opt_right_arm)
  {
    for(size_t i=0; i<3; i++)
      o1Me[i][i] = 0.;
    o1Me[0][3] = 0.0;//tx
    o1Me[1][3] = -0.055; //ty
    o1Me[2][3] = 0.0; //tz
    o1Me[0][0] = 1;
    o1Me[2][1] = -1;
    o1Me[1][2] = 1;
  }

  else
  {

    for(size_t i=0; i<3; i++)
      o1Me[i][i] = 0.;
    o1Me[0][3] = 0.0;//tx
    o1Me[1][3] = 0.055; //ty
    o1Me[2][3] = 0.0; //tz
    o1Me[0][0] =  1;
    o1Me[2][1] = 1;
    o1Me[1][2] = -1;

  }

  vpHomogeneousMatrix o2Me, o2Mo1, o1Mo2, eMo2;

  vpCameraParameters cam;
  vpXmlParserCamera cameraParser;

  std::string cameraXmlFile = "/udd/fspindle/Public/const_camera_Viper850.xml";

  cameraParser.parse(cam, cameraXmlFile,
                     "PTGrey-Flea2-6mm",
                     vpCameraParameters::perspectiveProjWithDistortion,
                     640, 480);
  std::cout << cam << std::endl;

  for(;;) {
    g.acquire(I);
    vpDisplay::display(I);

    bool status = detector.detect(I);
    std::ostringstream legend;
    legend << detector.getNbObjects() << " bar code detected";
    vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);

    if (status) {
      for(size_t i=0; i<detector.getNbObjects(); i++) {
        std::vector<vpImagePoint> p = detector.getPolygon(i);
        vpRect bbox = detector.getBBox(i);
        vpDisplay::displayRectangle(I, bbox, vpColor::green);
        vpDisplay::displayText(I, bbox.getTop()-20, bbox.getLeft(), "Message: \"" + detector.getMessage(i) + "\"", vpColor::red);
        for(size_t j=0; j < p.size(); j++) {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j]+vpImagePoint(10,0), number.str(), vpColor::blue);
        }
      }
      if (detector.getNbObjects() == pose.size()) {
        for(size_t i=0; i<detector.getNbObjects(); i++) {
          size_t index; // 0 is for cMo1=LArm_ref_pose, 1 is for cMo2=romeo_left_arm (or romeo_right_arm )
          unsigned int size; // 0 for small(3.5cm) 1 for big (4.5)
          if (detector.getMessage(i) == "LArm_ref_pose")
          {
            index = 0;
            size = 0;
          }
          else if (detector.getMessage(i) == "romeo_left_arm")
          {
            index = 1;
            size = 1;
          }
          else if (detector.getMessage(i) == "romeo_right_arm")
          {
            index = 1;
            size = 1;
          }
          else {
            std::cout << detector.getMessage(i)  << " Qrcode messages not recognized" << std::endl;
            return 0;
          }
          pose[index].clearPoint();
          std::vector<vpImagePoint> p = detector.getPolygon(i);
          for(size_t j=0; j < p.size(); j++) {
            double x=0., y=0.;
            vpPixelMeterConversion::convertPoint(cam, p[j], x, y);
            if (size == 0)
            {
              Ps[j].set_x(x);
              Ps[j].set_y(y);
              pose[index].addPoint(Ps[j]);
            }

            else if (size == 1)
            {
              Pm[j].set_x(x);
              Pm[j].set_y(y);
              pose[index].addPoint(Pm[j]);
            }
          }
          pose[index].computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo[index]);
          std::cout << "Pose " << index << " (" << detector.getMessage(i) << "):\n" << cMo[index] << std::endl;
          vpDisplay::displayFrame(I, cMo[index], cam, 0.03, vpColor::none, 3);

          vpDisplay::displayFrame(I, cMo[0] * o1Me , cam, 0.03, vpColor::none, 3);

          //vpPoseVector r(cMo[1] ) ;
          //std::cout << "--- Pose LeftARM (" << detector.getMessage(i) << "):\n" << r << std::endl;

        }
        //        o2Mo1 = cMo[1].inverse() * cMo[0];
        //        o2Me = o2Mo1 * o1Me;

        o1Mo2 = cMo[0].inverse() * cMo[1];
        std::cout << "Transformation from \"" << detector.getMessage(0)
                  << "\" to \"" << detector.getMessage(1) << "\": \n" << o1Mo2 << std::endl;
        //        eMo2 = (cMo[0] * o1Me).inverse() * cMo[1];
        eMo2 = o1Me.inverse() * o1Mo2;

      }

      o2Me = eMo2.inverse();
      std::cout << "eMo2: \n" << eMo2 << std::endl;
      std::cout << "o2Me: \n" << o2Me << std::endl;

    }
    vpDisplay::displayText(I, I.getHeight()-15, 10, "Click to save and quit...", vpColor::red);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) { // a click to save and exit
      vpXmlParserHomogeneousMatrix p;
      std::string filename = "transformation.xml";
      if (p.save(o2Me, filename, name_o2Me) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot save the homogeneous matrix eMo2" << std::endl;
      }
      break;
    }
  }

#else
  (void)argc;
  (void)argv;
#endif
}
