
#include <vpTemplateLocatization.h>


vpTemplateLocatization::vpTemplateLocatization(const std::string &model, const std::string &configuration_file_folder, const vpCameraParameters &cam)
  : m_warp(), m_tracker(NULL), m_state(detection), m_target_found(false), m_P(4), m_message("romeo_left_arm"), m_tracker_det(NULL),
    m_keypoint_learning(NULL), m_keypoint_detection (NULL), m_init_detection (false),m_num_iteration_detection(6), m_counter_detection(0),
    m_manual_detection (0), m_checkValiditycMo(NULL), m_only_detection(false), m_status_single_detection(false), verbose (true), m_corners_detected()
{

  //Detection *****************************************

  m_model = model;
  m_cam = cam;

  //Initiaze tracker
  m_tracker_det = new vpMbEdgeKltTracker;

  if(vpIoTools::checkFilename(m_model + ".xml")) {
    m_tracker_det->loadConfigFile(m_model + ".xml");
  }
  m_tracker_det->setCameraParameters(m_cam);
  m_tracker_det->setOgreVisibilityTest(false);
  //m_tracker->setScanLineVisibilityTest(true);

  if(vpIoTools::checkFilename(m_model + ".cao"))
    m_tracker_det->loadModel(m_model + ".cao");
  else if(vpIoTools::checkFilename(m_model + ".wrl"))
    m_tracker_det->loadModel(m_model + ".wrl");
  std::cout<< "2***************************************** " << std::endl;

  m_tracker_det->setDisplayFeatures(true);



  //Initalize Detection
  m_configuration_file = configuration_file_folder + "detection-config.xml";
#if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
  m_configuration_file = configuration_file_folder + "detection-config-SIFT.xml";
#endif

  std::cout<< "Using configuration file: " <<  m_configuration_file << std::endl;

  // Learning initialization
  m_keypoint_learning  = new vpKeyPoint;
  m_keypoint_learning->loadConfigFile(m_configuration_file);

  // Detection initialization
  m_keypoint_detection = new vpKeyPoint;
  m_keypoint_detection->loadConfigFile(m_configuration_file);

  // Template tracker *****************************************
  m_tracker = new vpTemplateTrackerSSDInverseCompositional(&m_warp);
  m_tracker->setSampling(2,2);
  m_tracker->setLambda(0.001);
  m_tracker->setIterationMax(5);
  m_tracker->setPyramidal(2, 1);

  setTemplateSize(0.10,0.10);
}

vpTemplateLocatization::~vpTemplateLocatization()
{
  if (m_tracker_det != NULL)
    delete m_tracker_det;
  if (m_tracker != NULL)
    delete m_tracker;
  if (m_keypoint_learning != NULL)
    delete m_keypoint_learning;
  if (m_keypoint_detection != NULL)
    delete m_keypoint_detection;

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

  if (m_state == detection) {
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

    if (verbose)
      std::cout << "DETECTION PHASE"<< std::endl ;
    //vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);
    if (!m_init_detection)
    {
      std::cout << "ERROR: You need to call before vpMbLocalization::initDetection(const std::string &name_file_learning_data).";
      return false;
    }

    if (!m_manual_detection)
    {

      if(m_counter_detection < m_num_iteration_detection)
      {
        std::cout<< "m_counter_detection: " << std::endl << m_counter_detection <<"-"<< m_num_iteration_detection<<std::endl;

        double error, elapsedTime;
        vpHomogeneousMatrix cMo_temp;

        //Matching and pose estimation
        if(m_keypoint_detection->matchPoint(I, m_cam, cMo_temp, error, elapsedTime))
        {
          if (verbose)
            std::cout <<"elaspedtime: " << elapsedTime << std::endl;
          if (!isIdentity(cMo_temp) )//&& m_checkValiditycMo(cMo_temp))
          {

            //Tracker set pose
            //m_tracker->setPose(I, cMo_temp);
            m_tracker_det->initFromPose(I, cMo_temp);
            if (verbose)
            {
              std::cout << "Detection ok" << std::endl;
              std::cout << "Pose: " <<std::endl << cMo_temp << std::endl;
            }

            //Display
            m_tracker_det->display(I, cMo_temp, m_cam, vpColor::cyan, 1);
            vpDisplay::displayFrame(I, cMo_temp, m_cam, 0.025, vpColor::none, 3);

            vpPoseVector cPo;
            cPo.buildFrom(cMo_temp);
            m_stack_cMo_detection.stackMatrices(cPo.t());
//            if (m_only_detection)
//            {
//              unsigned int nbMatch = m_keypoint_detection->matchPoint(I);
//              vpImagePoint iPref, iPcur, cog(0, 0);
//              for (unsigned int i = 0; i < nbMatch; i++)
//              {
//                m_keypoint_detection->getMatchedPoints(i, iPref, iPcur);
//                cog +=iPcur;
//              }

//              m_cog = cog/nbMatch;
//              m_status_single_detection = true;

//            }
//            else
              m_counter_detection ++;

          }
        }

        else
          std::cout << "Detection failed" << std::endl;
      }

      else

      {
        vpPoseVector cMo_;
        for (unsigned int i = 0; i<6; i++)
          cMo_[i] = vpColVector::median( m_stack_cMo_detection.getCol(i));

        std::cout<< "Median: " << std::endl << cMo_<<std::endl;
        unsigned int pose_ok_counter = 0;
        vpColVector translation_median = cMo_.getCol(0,0,3);

        //std::cout<< "translation_median " << translation_median <<std::endl;


        for (unsigned int i = 0; i < m_stack_cMo_detection.getRows()-1 ;i++ )
        {

          double distance_from_median = sqrt((translation_median.t() - m_stack_cMo_detection.getRow(i,0,3)).sumSquare());
          //std::cout<< "Distance " << i << ":" << std::endl << distance_from_median<<std::endl;

          if ( distance_from_median < 0.5) //0.005 )//&& (theta_error_grasp < vpMath::rad(3)) )
          {
            pose_ok_counter++;
            //std::cout<< "Ok for pose number: " << i <<std::endl;
          }
        }
        std::cout<< "Pose counter ok: " << std::endl << pose_ok_counter<<std::endl;


        if (pose_ok_counter >= int (0.75 * double (m_num_iteration_detection)) )
        {

          std::cout<< "Ok starting track. Reached: " <<  int (0.75 * double (m_num_iteration_detection)) <<std::endl;
          vpHomogeneousMatrix cMo;
          cMo.buildFrom(cMo_);

          //m_tracker->setPose(I,cMo);
          m_tracker_det->initFromPose(I,cMo);
          m_counter_detection = 0;
          m_stack_cMo_detection.init();

          std::vector<vpPolygon> polygons;
          std::vector<std::vector<vpPoint> > roisPt;
          std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = m_tracker_det->getPolygonFaces(false);
          polygons = pair.first;
          roisPt = pair.second;


          if (polygons.size() == 1)
          {
            m_corners_detected = polygons[0].getCorners();

            for(size_t j=0; j < m_corners_detected.size(); j++) {
              std::ostringstream s;
              s << j;
              vpDisplay::displayText(I, m_corners_detected[j]+vpImagePoint(-20,-20), s.str(), vpColor::green);
              vpDisplay::displayCross(I, m_corners_detected[j], 25, vpColor::green, 2);
//              vpDisplay::flush(I);
//              vpDisplay::getClick(I,true);


            }

            m_state = init_tracking;
          }
          else
          {
            std::cout << "ERROR: The model has to have only one face." << std::endl;

          }


        }

        else
          m_counter_detection = 0;

      }


    }
    else
    {
      m_tracker_det->initClick(I, m_model + ".init", true);
      m_state = init_tracking;
    }



  }
  if (m_state == init_tracking) {
    //vpDisplay::displayText(I, 40,10, "state: init tracking", vpColor::red);
    try {
      m_tracker->resetTracker();

      m_tracker->initFromPoints(I, m_corners_detected, true);
     // m_tracker->initClick(I,true);
      m_tracker->track(I);
      m_tracker->display(I, vpColor::green);
      m_zone_ref = m_tracker->getZoneRef();
      m_area_m_zone_ref = m_zone_ref.getArea();
      p = m_tracker->getp();
      m_warp.warpZone(m_zone_ref, p, zone_cur);
      m_area_zone_prev = m_area_zone_cur = zone_cur.getArea();
      m_corners_tracked = getTemplateTrackerCorners(zone_cur);
      m_corners_tracked_index = computedTemplateTrackerCornersIndexes(m_corners_detected, m_corners_tracked);
      //std::cout << "Size:" << m_corners_tracked.size() <<std::endl;
      m_corners_tracked = orderPointsFromIndexes(m_corners_tracked_index, m_corners_tracked);

      computePose(m_P, m_corners_tracked, m_cam, true, m_cMo);
      //vpDisplay::displayFrame(I, m_cMo, m_cam, 0.04, vpColor::none, 3);

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
      //      else if(zone_cur.getBoundingBox().getSize() > max_target_size) {
      //        //          std::cout << "reinit caused by size area" << std::endl;
      //        m_state = detection;
      //        m_target_found = false;
      //      }
      else {
        m_corners_tracked = getTemplateTrackerCorners(zone_cur);
        m_corners_tracked = orderPointsFromIndexes(m_corners_tracked_index, m_corners_tracked);
        computePose(m_P, m_corners_tracked, m_cam, false, m_cMo);

        //                   vpDisplay::displayFrame(I, m_cMo, m_cam, 0.04, vpColor::none, 3);
        //                    for(unsigned int j=0; j < m_corners_tracked.size(); j++) {
        //                      std::ostringstream s;
        //                      //s << m_corners_tracked_index[j];
        //                      s <<j;
        //                      vpDisplay::displayText(I, m_corners_tracked[j]+vpImagePoint(-10,-10), s.str(), vpColor::blue);
        //                      vpDisplay::displayCross(I, m_corners_tracked[j], 15, vpColor::green, 2);
        //                    }

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
          //if (corners[m] == corners_tracked[n]) {
          if (vpImagePoint::distance(corners[m], corners_tracked[n])<10) {
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


/*!
  This function check if the matrix A is an identity matrix or not
  * \param A Homogeneus matrix to check
 */
bool vpTemplateLocatization::isIdentity(const vpHomogeneousMatrix & A) const
{
  bool flag = 1;
  for (unsigned int i = 0; i < A.getRows(); i++)
  {
    for (unsigned int j = 0; j < A.getCols(); j++)
    {
      if((A[i][i] != 1) || (( i != j) && (A[i][j] != 0)))
      {
        flag = 0;
        break;
      }
    }
  }

  return flag;

}

/*!
  Init the detection loading the learning data.
 */
void vpTemplateLocatization::initDetection(const std::string &name_file_learning_data)
{
  m_keypoint_detection->loadLearningData(name_file_learning_data, true);
  m_init_detection = true;
}
