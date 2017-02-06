#ifndef __vpPepperFollowPeople_h__
#define __vpPepperFollowPeople_h__

#include <alproxies/almemoryproxy.h>
#include <alproxies/alfacedetectionproxy.h>
#include <alproxies/alpeopleperceptionproxy.h>
#include <alproxies/alspeechrecognitionproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/altexttospeechproxy.h>


#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorBase.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPlot.h>


//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>


#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpFaceTrackerOkao.h>


class vpPepperFollowPeople
{
protected:

  // Robot
  vpCameraParameters m_cam;
  vpHomogeneousMatrix m_eMc;
  vpNaoqiRobot * m_robot;
  std::vector<std::string> m_jointNames_head;
  std::vector<std::string> m_vocabulary;
  std::string m_phrase;
  AL::ALValue m_limit_yaw;

  // Proxies and tracker
  AL::ALMemoryProxy m_mem_proxy;
  AL::ALPeoplePerceptionProxy m_people_proxy;
  AL::ALSpeechRecognitionProxy m_asr_proxy;
  AL::ALLedsProxy m_led_proxy;
  vpFaceTrackerOkao m_face_tracker;
  AL::ALTextToSpeechProxy m_tts_proxy;

  // Servo
  vpServo m_task;
  vpAdaptiveGain m_lambda_base;
  vpAdaptiveGain m_lambda_nobase;
  AL::ALFaceDetectionProxy m_proxy;
  std::vector<cv::Rect> m_faces;  //!< Bounding box of each detected face.
  std::vector<float> m_scores;
  const int m_image_height;
  const int m_image_width;
  vpImagePoint m_previuos_cog;
  double m_Zd;
  double m_Z;
  double m_servo_time_init;
  vpColVector m_q_dot;
  double m_t_prev;
  vpImagePoint m_head_cog_cur;
  vpImagePoint m_ip;
  vpFeatureDepth m_s_Z;
  vpFeatureDepth m_s_Zd;
  vpFeaturePoint m_s;
  vpFeaturePoint m_sd;



  vpMatrix tJe;
  vpMatrix eJe;

  //Bool
  bool m_stop_vxy;
  bool m_move_base;
  bool m_move_base_prev;
  bool m_reinit_servo;
  bool m_person_found;
  bool m_reverse;


public:
  /*!
    Default destructor.
   */
  vpPepperFollowPeople(std::string ip, int port, vpNaoqiRobot &robot);
  ~vpPepperFollowPeople();

  bool computeAndApplyServo();
  void stopTranslationBase();
  void activateTranslationBase();
  void stop();
  void exit();

  double getActualDistance() const;
  void setDesiredDistance(double dist);
  void setReverse(bool flag);

};

#endif

