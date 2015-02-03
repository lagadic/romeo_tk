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
 * This class allows to learn, detect and track an object. We use keypoints to detect and estimate the pose of a known object
 * using his cad model. The first step consists in detecting and learning keypoints located on the faces of an object, while
 * the second step makes the matching between the detected keypoints in the query image with those previously learned.
 * The pair of matches are then used to estimate the pose of the object with the knowledge of the correspondences between the
 * 2D and 3D coordinates.
 *
 * Authors:
 * Giovanni Claudio
 *
 *****************************************************************************/
#ifndef __vpMbLocalization_h__
#define __vpMbLocalization_h__

#include <iostream>


// ViSP includes
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpVideoReader.h>
#include <visp/vpKeyPoint.h>
#include <visp/vpImage.h>


/*!
  This class allows to learn, detect and track an object. We use keypoints to detect and estimate the pose of a known object
  using his cad model. The first step consists in detecting and learning keypoints located on the faces of an object, while
  the second step makes the matching between the detected keypoints in the query image with those previously learned.
  The pair of matches are then used to estimate the pose of the object with the knowledge of the correspondences between the
  2D and 3D coordinates.

  The following example shows how to use this class.
  \code
   //To do
  \endcode
 */

class vpMbLocalization
{
public:
  typedef enum {
    detection,
    tracking,
    none
  } state_t;


protected:
  std::string m_configuration_file;
  vpMbEdgeKltTracker * m_tracker;
  //vpMbEdgeTracker * m_tracker;
  std::string m_model;
  state_t m_state;
  vpCameraParameters m_cam;
  vpHomogeneousMatrix m_cMo;
  vpKeyPoint * m_keypoint_learning;

  //Detection:
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


public:
  vpMbLocalization(const std::string &model, const std::string &configuration_file_folder, const vpCameraParameters &cam);
  virtual ~vpMbLocalization();

  bool detectObject(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo);

  vpHomogeneousMatrix get_cMo() const {return m_cMo;}
  vpMbEdgeTracker * getTracker() const {return m_tracker;}
  vpImagePoint get_cog() const {return m_cog;}
  bool getDetectionStatus() const {return m_status_single_detection;}
  void initDetection(const std::string & name_file_learning_data);
  bool isIdentity (const vpHomogeneousMatrix &A) const;
  void learnObject(vpImage<unsigned char> &I);
  void saveLearningData(const std::string & name_new_file_learning_data);
  void setForceDetection() {m_state = detection; }
  void setCameraParameters(const vpCameraParameters &cam) { m_cam = cam; }
  void setManualDetection(){m_manual_detection = true;}
  void setOnlyDetection(const bool only_detection){m_only_detection = only_detection;}
  void setNumberDetectionIteration (unsigned int &num) { m_num_iteration_detection = num;}
  void setValiditycMoFunction (bool (*funct)(vpHomogeneousMatrix)) { m_checkValiditycMo = funct;}
  bool track(const vpImage<unsigned char> &I);

  //bool detection(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, const unsigned int num_detections, bool (*checkcMo)(vpHomogeneousMatrix));


};













#endif
