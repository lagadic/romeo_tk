
#include <vpFaceTracker.h>
#include <visp/vpImageConvert.h>

vpFaceTracker::vpFaceTracker() : m_warp(), m_tracker(NULL), m_faces(), m_state(detection),
  m_face_cascade(), m_frame_gray(), m_zone_ref(), m_zone_cur(),
  m_area_zone_ref(0), m_area_zone_cur(0), m_area_zone_prev(0), m_p()
{
  m_tracker = new vpTemplateTrackerSSDInverseCompositional(&m_warp);
  m_tracker->setSampling(2,2);
  m_tracker->setLambda(0.001);
  m_tracker->setIterationMax(5);
  m_tracker->setPyramidal(2, 1);
}

vpFaceTracker::~vpFaceTracker()
{
  if (m_tracker != NULL)
    delete m_tracker;
}

void vpFaceTracker::setFaceCascade(const std::string &filename)
{
  if( ! m_face_cascade.load( filename ) ) {
    throw vpException(vpException::ioError, "Cannot read haar file: %s", filename.c_str());
  }
}

bool vpFaceTracker::track(const vpImage<unsigned char> &I)
{
  vpImageConvert::convert(I, m_frame_gray);


  //std::cout << "state: " << m_state << std::endl;
  //-- Detect faces
  bool target_found = false;
  size_t larger_face_index = 0;

  if (1) {//state == detection) {
    m_faces.clear();
    m_face_cascade.detectMultiScale( m_frame_gray, m_faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
    //std::cout << "Detect " << m_faces.size() << " faces" << std::endl;
    if (m_faces.size()) {

      m_state = init_tracking;

      // Display all the detected faces
      int face_max_area = 0;
      for( size_t i = 0; i < m_faces.size(); i++ ) {
        if (m_faces[i].area() > face_max_area) {
          face_max_area = m_faces[i].area();
          larger_face_index = i;
        }
      }
      target_found = true;
      // Display the larger face
      size_t i=larger_face_index;
      m_target.set(m_faces[i].tl().x, m_faces[i].tl().y, m_faces[i].size().width, m_faces[i].size().height);
      //                vpDisplay::displayRectangle(I, target, vpColor::green, false, 4);
    }
  }
  //-- Track the face
  if (m_state == init_tracking) {
    //vpDisplay::displayText(I, 10,10, "state: detection", vpColor::red);
    size_t i=larger_face_index;
    double scale = 0.05; // reduction factor
    int x = m_faces[i].tl().x;
    int y = m_faces[i].tl().y;
    int width  = m_faces[i].size().width;
    int height = m_faces[i].size().height;
    std::vector<vpImagePoint> corners;
    corners.push_back( vpImagePoint(y+scale*height    , x+scale*width) );
    corners.push_back( vpImagePoint(y+scale*height    , x+(1-scale)*width) );
    corners.push_back( vpImagePoint(y+(1-scale)*height, x+(1-scale)*width) );
    corners.push_back( vpImagePoint(y+(1-scale)*height, x+scale*width) );
    try {
      m_tracker->resetTracker();
      m_tracker->initFromPoints(I, corners, true);
      m_tracker->track(I);
      //m_tracker->display(I, vpColor::green);
      m_zone_ref = m_tracker->getZoneRef();
      m_area_zone_ref = m_zone_ref.getArea();
      m_p = m_tracker->getp();
      m_warp.warpZone(m_zone_ref, m_p, m_zone_cur);
      m_area_zone_prev = m_area_zone_cur = m_zone_cur.getArea();
      m_state = tracking;
    }
    catch(...) {
      std::cout << "Exception init tracking" << std::endl;
      m_state = detection;
    }
  }
  else if (m_state == tracking) {
    try {
      //vpDisplay::displayText(I, 10,10, "state: tracking", vpColor::red);
      m_tracker->track(I);

      //m_tracker->display(I, vpColor::blue);
      {
        // Instantiate and get the reference zone
        m_p = m_tracker->getp();
        m_warp.warpZone(m_zone_ref, m_p, m_zone_cur);
        m_area_zone_cur = m_zone_cur.getArea();

        //          std::cout << "Area ref: " << m_area_zone_ref << std::endl;
        //std::cout << "Area tracked: " << m_area_zone_cur << std::endl;

        double size_percent = 0.6;
        if (m_area_zone_cur/m_area_zone_prev < size_percent || m_area_zone_cur/m_area_zone_prev > (1+size_percent)) {
          //std::cout << "reinit caused by size" << std::endl;
          m_state = detection;
        }
        else {
          m_target = m_zone_cur.getBoundingBox();
          target_found = true;
        }

        m_area_zone_prev = m_area_zone_cur;
      }
    }
    catch(...) {
      std::cout << "Exception tracking" << std::endl;
      m_state = detection;
    }
  }

  return target_found;
}


