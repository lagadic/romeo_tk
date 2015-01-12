#ifndef __vpFaceTracker_h__
#define __vpFaceTracker_h__

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpSRT.h>


class vpFaceTracker
{
protected:
  typedef enum {
    detection,
    init_tracking,
    tracking,
    none
  } state_t;
  vpTemplateTrackerWarpSRT m_warp;
  vpTemplateTrackerSSDInverseCompositional *m_tracker;
  std::vector<cv::Rect> m_faces;
  state_t m_state;
  cv::CascadeClassifier m_face_cascade;
  cv::Mat m_frame_gray;
  vpTemplateTrackerZone m_zone_ref, m_zone_cur;
  double m_area_zone_ref, m_area_zone_cur, m_area_zone_prev;
  vpColVector m_p;
  vpRect m_target;


public:
  vpFaceTracker();
  ~vpFaceTracker();

  vpRect getFace() const { return m_target;};
  void setFaceCascade(const std::string &filename);
  bool track(const vpImage<unsigned char> &I);
};

#endif

