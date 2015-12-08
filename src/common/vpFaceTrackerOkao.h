#ifndef __vpFaceTrackerOkao_h__
#define __vpFaceTrackerOkao_h__

#include <visp3/core/vpConfig.h>

#include <alproxies/almemoryproxy.h>
#include <alproxies/alfacedetectionproxy.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp3/detection/vpDetectorBase.h>

class VISP_EXPORT vpFaceTrackerOkao : public vpDetectorBase
{
protected:

  AL::ALFaceDetectionProxy m_proxy;
  AL::ALMemoryProxy m_mem_proxy;
  std::vector<cv::Rect> m_faces;  //!< Bounding box of each detected face.
  std::vector<float> m_scores;
  const int m_image_height;
  const int m_image_width;
  vpImagePoint m_previuos_cog;



public:
  /*!
    Default destructor.
   */
  vpFaceTrackerOkao(std::string ip, int port);
  ~vpFaceTrackerOkao();

  bool clearDatabase();
  bool detect(const vpImage <unsigned char> &I);
  bool detect();
  bool forgetPerson(const std::string& name);


  float getScore(unsigned int i) const;

};

#endif

