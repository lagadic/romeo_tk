/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://team.inria.fr/lagadic/visp for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://team.inria.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * This class allows to learn, detect an object using the color information (HSV).
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/
#ifndef __vpColorDetection_h__
#define __vpColorDetection_h__

#include <iostream>


//OpenCV Include

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// ViSP includes
#include <visp/vpConfig.h>
#include <visp/vpDetectorBase.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>


/*!
  This class allows to learn, detect an object with a specific color.

  The following example shows how to use this class.
  \code
   //To do
  \endcode
 */

class VISP_EXPORT vpColorDetection : public vpDetectorBase
{
protected:
  int m_H_min; //!< Minumum H value
  int m_H_max; //!< Maximum H value
  int m_S_min; //!< Minumum S value
  int m_S_max; //!< Maximum S value
  int m_V_min; //!< Minumum V value
  int m_V_max; //!< Maximum V value

  bool m_init_learning; //!< One when the learning phase starts
  bool m_learning_phase; //!< If we are in learning phase equal to one
  double m_max_obj_area; //!< Maximum Area for an object to be detected
  double m_min_obj_area; //!< Minimum Area for an object to be detected
  unsigned int m_max_objs_num; //!<  Max number of objects to detect
  std::string m_name; //!< Name of the kind of object
  std::vector<cv::Rect> m_objects;  //!< Bounding box of the detected objects.

  const std::string m_trackbarWindowName; //!< Name of the trackbar window in opencv
  cv::Mat m_T; //!< OpenCV image used as input for the object detection.

  void createTrackbars();
  void drawObject(int &x, int &y, cv::Mat &frame);
  void morphOps(cv::Mat &T);
  bool trackFilteredObject(cv::Mat threshold);
  std::string intToString(int number);


public:
  vpColorDetection();
  virtual ~vpColorDetection() {}
  bool detect(const vpImage<unsigned char> &I) { std::cout << "Not implemented" << std::endl;}
  bool detect(const cv::Mat &I);


  std::vector<int> getValueHSV();
  std::string getName(){return m_name;}

  bool learningColor(const cv::Mat &I);
  bool loadHSV(const std::string &filename);
  bool saveHSV(const std::string &filename);
  void setMinObjectArea(const double &area_min){m_min_obj_area = area_min; }
  void setMaxObjectArea(const double &area_max){m_max_obj_area = area_max; }
  void setMaxAndMinObjectArea(const double &area_min, const double &area_max );
  void setName(const std::string &name) {m_name = name;}
  void setValuesHSV(const int H_min, const int S_min, const int V_min,
                    const int H_max, const int S_max, const int V_max);
  void setValuesHSV(const std::vector<int> values);

};









#endif
