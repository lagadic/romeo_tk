#ifndef __vpQRCodeTracker_h__
#define __vpQRCodeTracker_h__

#include <visp/vpCameraParameters.h>
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>
#include <visp/vpPose.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpPixelMeterConversion.h>

#ifndef VISP_HAVE_ZBAR
#  error "Cannot build the project, libzbar is missing. Install libzbar using apt-get install libzbar-dev and rebuild ViSP."
#endif

class vpQRCodeTracker
{
public:
  typedef enum {
    detection,
    init_tracking,
    tracking,
    none
  } state_t;

protected:
  vpDetectorBase *m_detector;
  vpTemplateTrackerWarpHomography m_warp;
  vpTemplateTrackerSSDInverseCompositional *m_tracker;
  vpTemplateTrackerZone m_zone_ref, zone_cur;
  double m_area_m_zone_ref, m_area_zone_cur, m_area_zone_prev;

  state_t m_state;
  std::vector<vpImagePoint> m_corners_detected;
  std::vector<vpImagePoint> m_corners_tracked;
  std::vector<int> m_corners_tracked_index;
  bool m_target_found;
  vpRect m_target_bbox; // BBox of the tracked qrcode
  std::vector<vpPoint> m_P; // Points of the qrcode model
  vpCameraParameters m_cam;
  vpHomogeneousMatrix m_cMo;
  bool m_force_detection;
  std::string m_message;

public:

  /*!
   Default QRcode size is set to 0.06 meter.
   * \param barcode
   */
  vpQRCodeTracker(int barcode=0);

  virtual ~vpQRCodeTracker();

  vpHomogeneousMatrix get_cMo() const {return m_cMo;}

  /*!
    Return the center of gravity location of the tracked bar code.
    */
  vpImagePoint getCog();
  std::vector<vpImagePoint> getCorners() const {return m_corners_tracked;}

  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }

  void setForceDetection(bool force_detection) {
    m_force_detection = force_detection;
  }

  void setMessage(const std::string &message) {
    m_message = message;
  }

  void setQRCodeSize(double qrcode_size);

  bool track(const vpImage<unsigned char> &I);
  bool track(const vpImage<unsigned char> &I, vpDetectorBase *&detector );

private:
  std::vector<vpImagePoint> getTemplateTrackerCorners(const vpTemplateTrackerZone &zone);

  std::vector<int> computedTemplateTrackerCornersIndexes(const std::vector<vpImagePoint> &corners_detected,
                                                         const std::vector<vpImagePoint> &corners_tracked);

  std::vector<vpImagePoint> orderPointsFromIndexes(const std::vector<int> &indexes,
                                                   const std::vector<vpImagePoint> &corners);

  void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
                   const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);
};

#endif
