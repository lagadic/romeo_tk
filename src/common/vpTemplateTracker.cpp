
#include <vpTemplateLocatization.h>


vpTemplateLocatization::vpTemplateLocatization()
  : m_detector(NULL), m_warp(), m_tracker(NULL), m_state(detection), m_target_found(false), m_P(4), m_force_detection(false), m_message("romeo_left_arm")
{
  //  if (barcode == 0)
  //  {
  //    m_detector = new vpDetectorQRCode;
  //    std::cout << "vpDetectorQRCode"<< std::endl;
  //   }
  //  else
  //    m_detector = new vpDetectorDataMatrixCode;

  m_tracker = new vpTemplateTrackerSSDInverseCompositional(&m_warp);
  m_tracker->setSampling(2,2);
  m_tracker->setLambda(0.001);
  m_tracker->setIterationMax(5);
  m_tracker->setPyramidal(2, 1);

  setTemplateSize(0.10,0.10);
}

vpTemplateLocatization::~vpTemplateLocatization()
{
  //  if (m_detector != NULL)
  //    delete m_detector;
  if (m_tracker != NULL)
    delete m_tracker;
};


/*!
    Return the center of gravity location of the tracked bar code.
    */
vpImagePoint vpTemplateLocatization::getCog()
{
  vpImagePoint cog(0,0);
  for(size_t i=0; i < m_corners_tracked.size(); i++) {
    cog += m_corners_tracked[i];
  }
  cog /= m_corners_tracked.size();
  return cog;
}


void vpTemplateLocatization::setTemplateSize(double x,double y)
{
  //  m_P[0].setWorldCoordinates( qrcode_size/2.,  qrcode_size/2., 0); //                             / \ x
  //  m_P[1].setWorldCoordinates(-qrcode_size/2.,  qrcode_size/2., 0); //                              |
  //  m_P[2].setWorldCoordinates(-qrcode_size/2., -qrcode_size/2., 0); // small dot on the qrcode y <--|
  //  m_P[3].setWorldCoordinates( qrcode_size/2., -qrcode_size/2., 0);
  m_P[0].setWorldCoordinates(-x/2., -y/2., 0); //                              |--> x
  m_P[1].setWorldCoordinates(-x/2.,  y/2., 0); //                              |
  m_P[2].setWorldCoordinates( x/2.,  y/2., 0); // small dot on the qrcode     \|/ y
  m_P[3].setWorldCoordinates( x/2., -y/2., 0);
}

//bool vpTemplateLocatization::track(const vpImage<unsigned char> &I)
//{
//  bool result = false;
//  bool status = m_detector->detect(I);
//  if (status)
//    result = track(I, m_detector);

//  return result;
//}



bool vpTemplateLocatization::track(const vpImage<unsigned char> &I)//, vpDetectorBase * &detector )
{
  vpColVector p; // Estimated parameters

  if (m_state == detection || m_force_detection) {
    //bool status = detector->detect(I);
    //    if (detector->getNbObjects()>0) {
    //      for (size_t i=0; i < detector->getNbObjects(); i++) {
    //        if (detector->getMessage(i) == m_message) {
    //          m_corners_detected = detector->getPolygon(i);

    //          m_state = init_tracking;

    //          //          vpDisplay::displayText(I, I.getHeight()-20, 10, "Bar code message: " + m_detector->get_message(i), vpColor::green);
    //          //          for(size_t j=0; j < m_corners_detected.size(); j++) {
    //          //            std::ostringstream s;
    //          //            s << j;
    //          //            vpDisplay::displayText(I, m_corners_detected[j]+vpImagePoint(-20,-20), s.str(), vpColor::green);
    //          //            vpDisplay::displayCross(I, m_corners_detected[j], 25, vpColor::green, 2);
    //          //          }
    //        }
    //      }

    //    }

    //m_tracker->initClick(I);
    m_state = init_tracking;


  }
  if (m_state == init_tracking) {
    //vpDisplay::displayText(I, 40,10, "state: init tracking", vpColor::red);
    try {
      m_tracker->resetTracker();

      //m_tracker->initFromPoints(I, m_corners_detected, true);
      m_tracker->initClick(I);
      m_tracker->track(I);
      m_tracker->display(I, vpColor::green);
      m_zone_ref = m_tracker->getZoneRef();
      m_area_m_zone_ref = m_zone_ref.getArea();
      p = m_tracker->getp();
      m_warp.warpZone(m_zone_ref, p, zone_cur);
      m_area_zone_prev = m_area_zone_cur = zone_cur.getArea();
      m_corners_tracked = getTemplateTrackerCorners(zone_cur);
      m_corners_tracked_index = computedTemplateTrackerCornersIndexes(m_corners_detected, m_corners_tracked);
      m_corners_tracked = orderPointsFromIndexes(m_corners_tracked_index, m_corners_tracked);

      computePose(m_P, m_corners_tracked, m_cam, true, m_cMo);
      vpDisplay::displayFrame(I, m_cMo, m_cam, 0.04, vpColor::none, 3);

      m_state = tracking;
      m_target_found = true;
    }
    catch(...) {
      std::cout << "Exception init tracking" << std::endl;
      m_state = detection;
      m_target_found = false;
    }
  }
  else if (m_state == tracking) {
    try {
      //vpDisplay::displayText(I, 40,10, "state: tracking", vpColor::red);
      m_tracker->track(I);

      //m_tracker->display(I, vpColor::blue);

      // Instantiate and get the reference zone
      p = m_tracker->getp();
      m_warp.warpZone(m_zone_ref, p, zone_cur);
      m_area_zone_cur = zone_cur.getArea();

      double size_percent = 0.95;
      double max_target_size = I.getSize()/4;
      if (m_area_zone_cur/m_area_zone_prev < size_percent || m_area_zone_cur/m_area_zone_prev > (1+size_percent)) {
        //          std::cout << "reinit caused by size" << std::endl;
        m_state = detection;
        m_target_found = false;
      }
      else if(zone_cur.getBoundingBox().getSize() > max_target_size) {
        //          std::cout << "reinit caused by size area" << std::endl;
        m_state = detection;
        m_target_found = false;
      }
      else {
        m_corners_tracked = getTemplateTrackerCorners(zone_cur);
        m_corners_tracked = orderPointsFromIndexes(m_corners_tracked_index, m_corners_tracked);

        computePose(m_P, m_corners_tracked, m_cam, false, m_cMo);

        //            vpDisplay::displayFrame(I, m_cMo, m_cam, 0.04, vpColor::none, 3);
        //            for(unsigned int j=0; j < m_corners_tracked.size(); j++) {
        //              std::ostringstream s;
        //              s << m_corners_tracked_index[j];
        //              vpDisplay::displayText(I, m_corners_tracked[j]+vpImagePoint(-10,-10), s.str(), vpColor::blue);
        //              vpDisplay::displayCross(I, m_corners_tracked[j], 15, vpColor::green, 2);
        //            }

        m_target_found = true;
      }

      m_area_zone_prev = m_area_zone_cur;

    }
    catch(...) {
      std::cout << "Exception tracking" << std::endl;
      m_state = detection;
      m_target_found = false;
    }
  }
  return m_target_found;
}

std::vector<vpImagePoint> vpTemplateLocatization::getTemplateTrackerCorners(const vpTemplateTrackerZone &zone)
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

std::vector<int> vpTemplateLocatization::computedTemplateTrackerCornersIndexes(const std::vector<vpImagePoint> &corners_detected, const std::vector<vpImagePoint> &corners_tracked)
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

std::vector<vpImagePoint> vpTemplateLocatization::orderPointsFromIndexes(const std::vector<int> &indexes, const std::vector<vpImagePoint> &corners)
{
  std::vector<vpImagePoint> corners_ordered(corners.size());
  for(unsigned int i=0; i < corners.size(); i++) {
    corners_ordered[indexes[i]] = corners[i];
  }
  return corners_ordered;
}

void vpTemplateLocatization::computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
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

  if (init) {
    vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
    pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo_dementhon);
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo_lagrange);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);
    if (residual_dementhon < residual_lagrange)
      cMo = cMo_dementhon;
    else
      cMo = cMo_lagrange;
  }

  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
}
