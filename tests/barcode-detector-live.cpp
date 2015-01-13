/*! \example tutorial-barcode-detector-live.cpp */
#include <stdlib.h>
#include <map>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpAffine.h>

#include <visp_naoqi/vpNaoqiGrabber.h>


typedef enum {
  detection,
  init_tracking,
  tracking,
  none
} state_t;

void orderPointsCounterClockWise(std::vector<vpImagePoint> &p)
{
  // Now we create a map of VpPoint in order to order the points
  std::map< double, vpImagePoint> poly_verteces;
  vpImagePoint mid_point(0, 0);
  for(unsigned int i=0; i < p.size(); i++)
    mid_point += p[i];
  mid_point /= p.size();

  double theta;
  for(unsigned int i=0; i < p.size(); i++)
  {
    theta = atan2(p[i].get_j() - mid_point.get_j(), p[i].get_i() - mid_point.get_i());
    // Insert the vertexes in the map (ordered)
    poly_verteces.insert ( std::pair<double, vpImagePoint>(theta, p[i]) );
  }
  // Now we create a Vector containing the ordered vertexes
  std::vector<vpImagePoint> p_ordered;
  for( std::map<double, vpImagePoint>::iterator it = poly_verteces.begin(); it!=poly_verteces.end(); ++it )
  {
    p_ordered.push_back( it->second );
  }
  p = p_ordered;
}


std::vector<vpImagePoint> getTemplateTrackerCorners(const vpTemplateTrackerZone &zone)
{
  std::vector<vpImagePoint> corners_tracked;

  // Parse all the triangles that describe the zone
  for (int i=0; i < zone.getNbTriangle(); i++) {
    vpTemplateTrackerTriangle triangle;
    // Get a triangle
    zone.getTriangle(i, triangle);
    std::vector<vpImagePoint> corners;
    // Get the 3 triangle corners
    triangle.getCorners( corners );
    if (i==0)
      corners_tracked = corners;
    else {
      for(unsigned int m=0; m < corners.size(); m++) { // corners of the 2nd triangle
        bool already_exists = false;
        for(unsigned int n=0; n < corners_tracked.size(); n++) { // already registered corners from the 1st triangle
          if (corners[m] == corners_tracked[n]) {
            already_exists = true;
            break;
          }
        }
        if (! already_exists)
          corners_tracked.push_back(corners[m]);
      }

    }
  }
  return corners_tracked;
}

std::vector<int> computedTemplateTrackerCornersIndexes(const std::vector<vpImagePoint> &corners_detected, const std::vector<vpImagePoint> &corners_tracked)
{
  std::vector<int> corners_tracked_index;
  for(unsigned int i=0; i < corners_tracked.size(); i++) {
    for(unsigned int j=0; j < corners_detected.size(); j++) {
      if (vpImagePoint::distance(corners_tracked[i], corners_detected[j]) < 4) {
        corners_tracked_index.push_back(j);
        break;
      }
    }
  }
  return corners_tracked_index;
}

std::vector<vpImagePoint> orderPointsFromIndexes(const std::vector<int> &indexes, const std::vector<vpImagePoint> &corners)
{
  std::vector<vpImagePoint> corners_ordered(corners.size());
  for(unsigned int i=0; i < corners.size(); i++) {
    corners_ordered[indexes[i]] = corners[i];
  }
  return corners_ordered;
}

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpPixelMeterConversion::convertPoint(cam, corners[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);
  else              pose.computePose(vpPose::VIRTUAL_VS, cMo) ;

  vpTranslationVector t;
  cMo.extract(t);
  vpRotationMatrix R;
  cMo.extract(R);
  vpThetaUVector tu(R);

  std::cout << "cMo: ";
  for (unsigned int i=0; i < 3; i++)
    std::cout << t[i] << " ";
  for (unsigned int i=0; i < 3; i++)
    std::cout << vpMath::deg(tu[i]) << " ";
  std::cout << std::endl;
}

int main(int argc, const char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) && (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX))
  int opt_device = 0;
  int opt_barcode = 0; // 0=QRCode, 1=DataMatrix
  std::string opt_ip = "198.18.0.1";

  for (unsigned int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = argv[i+1];
    else if (std::string(argv[i]) == "--code-type")
      opt_barcode = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--help")
      std::cout << "Usage: " << argv[0]
                << " [--ip <robot ip address>] [--code-type <0 for QRcode | 1 for DataMatrix>] [--help]"
                << std::endl;
  }
  try {

    vpNaoqiGrabber g;
    if (! opt_ip.empty())
      g.setRobotIp(opt_ip);
    g.open();

    vpImage<unsigned char> I; // for gray images
    g.acquire(I);
    vpDisplayX d(I);

    vpDetectorBase *detector;
    if (opt_barcode == 0)
      detector = new vpDetectorQRCode;
    else
      detector = new vpDetectorDataMatrixCode;

    vpTemplateTrackerWarpAffine warp;
    vpTemplateTrackerSSDInverseCompositional tracker(&warp);
    tracker.setSampling(2,2);
    tracker.setLambda(0.001);
    tracker.setIterationMax(5);
    tracker.setPyramidal(2, 1);

    state_t state = detection;
    vpTemplateTrackerZone zone_ref, zone_cur;
    double area_zone_ref, area_zone_cur, area_zone_prev;
    vpColVector p; // Estimated parameters
    std::vector<vpImagePoint> corners_detected;
    std::vector<vpImagePoint> corners_tracked;
    std::vector<int> corners_tracked_index;
    bool target_found = false;
    vpRect target_bbox; // BBox of the tracked qrcode

    vpCameraParameters cam = g.getCameraParameters();
    vpHomogeneousMatrix cMo;

    std::vector<vpPoint> P(4);
    double qrcode_size = 0.035;
    P[0].setWorldCoordinates( qrcode_size/2,  qrcode_size/2, 0); //                             / \ x
    P[1].setWorldCoordinates(-qrcode_size/2,  qrcode_size/2, 0); //                              |
    P[2].setWorldCoordinates(-qrcode_size/2, -qrcode_size/2, 0); // small dot on the qrcode y <--|
    P[3].setWorldCoordinates( qrcode_size/2, -qrcode_size/2, 0);


    for(;;) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      target_found = false;

      if (state == detection) {
        vpDisplay::displayCharString(I, 10,10, "state: detection", vpColor::red);
        bool status = detector->detect(I);
        if (status) {
          vpDisplay::displayText(I, 20, 10, "Bar code detected", vpColor::green);
          for (size_t i=0; i < detector->getNbObjects(); i++) {
            if (detector->getMessage(i) == "romeo_left_arm") {

              corners_detected = detector->getPolygon(i);
              state = init_tracking;
              target_found = true;

              vpDisplay::displayText(I, I.getHeight()-20, 10, "Bar code message: " + detector->getMessage(i), vpColor::green);
              for(size_t j=0; j < corners_detected.size(); j++) {
                std::ostringstream s;
                s << j;
                vpDisplay::displayText(I, corners_detected[j]+vpImagePoint(-20,-20), s.str(), vpColor::green);
                vpDisplay::displayCross(I, corners_detected[j], 25, vpColor::green, 2);
              }
            }
          }
        }
      }
      if (target_found && state == init_tracking) {
        vpDisplay::displayCharString(I, 10,10, "state: init tracking", vpColor::red);
        try {
          tracker.resetTracker();
          tracker.initFromPoints(I, corners_detected, true);
          tracker.track(I);
          //tracker.display(I, vpColor::green);
          zone_ref = tracker.getZoneRef();
          area_zone_ref = zone_ref.getArea();
          p = tracker.getp();
          warp.warpZone(zone_ref, p, zone_cur);
          area_zone_prev = area_zone_cur = zone_cur.getArea();
          corners_tracked = getTemplateTrackerCorners(zone_cur);
          corners_tracked_index = computedTemplateTrackerCornersIndexes(corners_detected, corners_tracked);
          corners_tracked = orderPointsFromIndexes(corners_tracked_index, corners_tracked);

          computePose(P, corners_tracked, cam, true, cMo);
          vpDisplay::displayFrame(I, cMo, cam, 0.04, vpColor::none, 3);

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

            double size_percent = 0.9;
            double max_target_size = I.getSize()/4;
            if (area_zone_cur/area_zone_prev < size_percent || area_zone_cur/area_zone_prev > (1+size_percent)) {
              std::cout << "reinit caused by size" << std::endl;
              state = detection;
            }
            else if(zone_cur.getBoundingBox().getSize() > max_target_size) {
              std::cout << "reinit caused by size area" << std::endl;
              state = detection;
            }
            else {
              corners_tracked = getTemplateTrackerCorners(zone_cur);
              corners_tracked = orderPointsFromIndexes(corners_tracked_index, corners_tracked);

              computePose(P, corners_tracked, cam, false, cMo);
              vpDisplay::displayFrame(I, cMo, cam, 0.04, vpColor::none, 3);

              for(unsigned int j=0; j<corners_tracked.size(); j++) {
                std::ostringstream s;
                s << corners_tracked_index[j];
                vpDisplay::displayText(I, corners_tracked[j]+vpImagePoint(-10,-10), s.str(), vpColor::blue);
                vpDisplay::displayCross(I, corners_tracked[j], 15, vpColor::green, 2);
              }


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

      //      if (target_found)
      //        vpDisplay::displayRectangle(I, target_bbox, vpColor::red, false, 2);

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
