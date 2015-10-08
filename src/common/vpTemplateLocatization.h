#ifndef __vpTemplateLocatization_h__
#define __vpTemplateLocatization_h__


#include <visp/vpKeyPoint.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>
#include <visp/vpPose.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpPixelMeterConversion.h>


class vpTemplateLocatization
{
public:
  typedef enum {
    detection,
    init_tracking,
    tracking,
    none
  } state_t;

protected:


  //template tracker
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

  // Detection
  std::string m_configuration_file;
  vpMbEdgeKltTracker * m_tracker_det;
  std::string m_model;
  vpKeyPoint * m_keypoint_learning;
  vpKeyPoint * m_keypoint_detection;
  vpImagePoint m_cog;
  bool m_init_detection;
  bool m_manual_detection;
  bool m_only_detection;
  bool m_status_single_detection;
  unsigned int m_counter_detection;
  unsigned int m_num_iteration_detection;
  vpMatrix m_stack_cMo_detection;
  bool (*m_checkValiditycMo)(vpHomogeneousMatrix);
  bool verbose;

public:

  /*!
   Default QRcode size is set to 0.06 meter.
   * \param barcode
   */
  vpTemplateLocatization(const std::string &model, const std::string &configuration_file_folder, const vpCameraParameters &cam);

  virtual ~vpTemplateLocatization();

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

  void setTemplateSize(double x, double y );

  bool track(const vpImage<unsigned char> &I);
  bool track(const vpImage<unsigned char> &I, vpDetectorBase *&detector );

  bool isIdentity (const vpHomogeneousMatrix &A) const;

  void setForceDetection() {m_state = detection; }
  void setManualDetection(){m_manual_detection = true;}
  void setOnlyDetection(const bool only_detection){m_only_detection = only_detection;}
  void setNumberDetectionIteration (unsigned int &num) { m_num_iteration_detection = num;}
  void setValiditycMoFunction (bool (*funct)(vpHomogeneousMatrix)) { m_checkValiditycMo = funct;}
  void initDetection(const std::string & name_file_learning_data);


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
