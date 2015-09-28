#ifndef __vpQRBlobsTarget_h__
#define __vpQRBlobsTarget_h__

#include <visp/vpCameraParameters.h>
#include <visp/vpPose.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>


#include <vpColorDetection.h>
class vpBlobsTargetTracker
{
public:
  typedef enum {
    detection,
    init_tracking,
    tracking,
    none
  } state_t;

protected:
  vpColorDetection m_colBlob;


  state_t m_state;
  //  std::vector<vpImagePoint> m_corners_detected;
  //  std::vector<vpImagePoint> m_corners_tracked;
  //  std::vector<int> m_corners_tracked_index;
  bool m_target_found;
  std::vector<vpPoint> m_P; // Points of the target
  vpCameraParameters m_cam;
  vpHomogeneousMatrix m_cMo;
  bool m_force_detection;
  std::string m_name;
  std::list<vpDot2> m_blob_list; // blob_list contains the list of the blobs that are detected in the image
  vpImagePoint m_cog;
  bool m_initPose;
  unsigned int m_numBlobs;
  bool m_manual_blob_init;
  bool m_left_hand_target;
  unsigned int m_grayLevelMaxBlob;
  unsigned int m_grayLevelMinBlob;
  bool m_full_manual;

public:

  /*!
   * \param barcode
   */
  vpBlobsTargetTracker();

  virtual ~vpBlobsTargetTracker();

  vpHomogeneousMatrix get_cMo() const {return m_cMo;}

  /*!
    Return the center of gravity location of the tracked bar code.
    */
  vpImagePoint getCog();

  unsigned int getGrayLevelMinBlob() const {return m_grayLevelMinBlob;}
  unsigned int getGrayLevelMaxBlob() const {return m_grayLevelMaxBlob;}

  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }

  void setForceDetection(const bool &force_detection) {
    m_force_detection = force_detection;
  }

  void setManualBlobInit(const bool &init) {
    m_manual_blob_init = init;
  }

  void setFullManualBlobInit(const bool &init) {
    m_manual_blob_init = init;
  }

  void setFullManual(const bool &value) {
    m_full_manual = value;
  }


  void setNumBlobs(const unsigned int &num) {
    m_numBlobs = num;
  }

  void setName(const std::string &name) {
    m_name = name;
    m_colBlob.setName(name);
  }

  void setLeftHandTarget(const bool &left_target) {
    m_left_hand_target = left_target;
  }


  bool loadHSV(const std::string name_file)
  {
    return  m_colBlob.loadHSV(name_file);
  }

  void setMaxAndMinObjectAreaColor(const double &area_min, const double &area_max)
  {
    m_colBlob.setMaxAndMinObjectArea(area_min, area_max);
  }

  void setGrayLevelMinBlob(const unsigned int & valueMin)  { m_grayLevelMinBlob = valueMin; }
  void setGrayLevelMaxBlob(const unsigned int & valueMax)  { m_grayLevelMaxBlob = valueMax; }


  void setPoints(const std::vector<vpPoint> &points)
  {
    m_P = points;
  }

  bool track(const cv::Mat &cvI, const vpImage<unsigned char> &I );

private:


  void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
                   const vpCameraParameters &cam, bool &init, vpHomogeneousMatrix &cMo);
};

#endif
