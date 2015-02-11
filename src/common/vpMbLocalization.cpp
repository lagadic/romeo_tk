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

# include <vpMbLocalization.h>


/*!
  Default constructor that set the default parameters as:
  - number iteration detection: 5
  - automatic detection (m_manual_detection = false)
  */
vpMbLocalization::vpMbLocalization(const std::string &model, const std::string &configuration_file_folder, const vpCameraParameters &cam)
  : m_tracker(NULL), m_keypoint_learning(NULL), m_keypoint_detection (NULL), m_init_detection (false),m_state(detection),
    m_num_iteration_detection(6), m_counter_detection(0), m_manual_detection (0), m_checkValiditycMo(NULL), m_only_detection(false), m_status_single_detection(false)

{
  m_model = model;
  m_cam = cam;

  //Initiaze tracker
  m_tracker = new vpMbEdgeKltTracker;
  //m_tracker = new vpMbEdgeTracker;

  if(vpIoTools::checkFilename(m_model + ".xml")) {
    m_tracker->loadConfigFile(m_model + ".xml");
  }

  m_tracker->setCameraParameters(m_cam);
  m_tracker->setOgreVisibilityTest(false);
  if(vpIoTools::checkFilename(m_model + ".cao"))
    m_tracker->loadModel(m_model + ".cao");
  else if(vpIoTools::checkFilename(m_model + ".wrl"))
    m_tracker->loadModel(m_model + ".wrl");
  //m_tracker->setDisplayFeatures(true);

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
}

/*!
  Learning the characteristics of the considered object by extracting the keypoints detected on the different faces.
  The learning data will be saved in a binary format.
  * \param I : image to process.
 */
void vpMbLocalization::learnObject(vpImage<unsigned char> &I)
{

  //Keypoint declaration and initialization
  //  static bool firstTime = false;
  //  if (firstTime == false) {
  m_tracker->initClick(I, m_model + ".init", true);
  //    firstTime = true;
  //  }
  //  else {
  //    m_tracker->resetTracker();
  //    m_tracker->initClick(I, m_model + ".init", true);
  //  }
  // m_tracker->track(I);

  //Keypoints reference detection
  double elapsedTime;
  std::vector<cv::KeyPoint> trainKeyPoints;
  m_keypoint_learning->detect(I, trainKeyPoints, elapsedTime);

  //Keypoints selection on faces
  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = m_tracker->getPolygonFaces(false);
  polygons = pair.first;
  roisPt = pair.second;

  vpHomogeneousMatrix cMo;
  std::vector<cv::Point3f> points3f;
  m_tracker->getPose(cMo);
  vpKeyPoint::compute3DForPointsInPolygons(cMo, m_cam, trainKeyPoints, polygons, roisPt, points3f);

  //Keypoints build reference
  m_keypoint_learning->buildReference(I, trainKeyPoints, points3f, true);


  //Display reference keypoints
  for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(I, it->pt.y, it->pt.x, 4, vpColor::red);
  }

}

/*!
  Save in a file .bin the learning data.
  * \param name_file
 */

void vpMbLocalization::saveLearningData(const std::string &name_new_file_learning_data)
{
  const std::string name = name_new_file_learning_data;
  m_keypoint_learning->saveLearningData( name, true);
}


/*!
  This function check if the matrix A is an identity matrix or not
  * \param A Homogeneus matrix to check
 */
bool vpMbLocalization::isIdentity(const vpHomogeneousMatrix & A) const
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
  This function will detect and track an object. If the tracking fails the algorithm will try to detect again the box.
  \param I : Image to process.
 */
bool vpMbLocalization::track(const vpImage<unsigned char> &I)
{

  bool status_tracking = false;
  m_status_single_detection = false;
  bool verbose = 0;

  if (m_state == detection ) {

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

        double error, elapsedTime;
        vpHomogeneousMatrix cMo_temp;

        if (m_only_detection)
        {
          unsigned int nbMatch = m_keypoint_detection->matchPoint(I);

          vpImagePoint iPref, iPcur, cog(0, 0);
          for (unsigned int i = 0; i < nbMatch; i++)
          {
            m_keypoint_detection->getMatchedPoints(i, iPref, iPcur);
            cog +=iPcur;
          }

          m_keypoint_detection->display(I);

          m_cog = cog/nbMatch;
          m_status_single_detection = true;

        }

        //Matching and pose estimation
        else if(m_keypoint_detection->matchPoint(I, m_cam, cMo_temp, error, elapsedTime))
        {
          if (verbose)
            std::cout <<"elaspedtime: " << elapsedTime << std::endl;
          if (!isIdentity(cMo_temp) && m_checkValiditycMo(cMo_temp))
          {

            //Tracker set pose
            m_tracker->setPose(I, cMo_temp);
            if (verbose)
            {
              std::cout << "Detection ok" << std::endl;
              std::cout << "Pose: " <<std::endl << cMo_temp << std::endl;
            }

            //Display
            m_tracker->display(I, cMo_temp, m_cam, vpColor::cyan, 1);
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
            //else
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

        std::cout<< "translation_median " << translation_median <<std::endl;
        //vpDisplay::getClick(I);

        for (unsigned int i = 0; i < m_stack_cMo_detection.getRows()-1 ;i++ )
        {

          double distance_from_median = sqrt((translation_median.t() - m_stack_cMo_detection.getRow(i,0,3)).sumSquare());
          std::cout<< "Distance " << i << ":" << std::endl << distance_from_median<<std::endl;

          if ( distance_from_median < 0.5) //0.005 )//&& (theta_error_grasp < vpMath::rad(3)) )
          {
            pose_ok_counter++;
            std::cout<< "Ok for pose number: " << i <<std::endl;
          }
        }
        std::cout<< "Pose counter ok: " << std::endl << pose_ok_counter<<std::endl;


        if (pose_ok_counter >= int (0.75 * double (m_num_iteration_detection)) )
        {

          //std::cout<< "Ok starting track. Reached: " <<  int (0.75 * double (m_num_iteration_detection)) <<std::endl;
          vpHomogeneousMatrix cMo;
          cMo.buildFrom(cMo_);

          m_tracker->setPose(I,cMo);
          m_counter_detection = 0;
          m_stack_cMo_detection.init();

          m_state = tracking;

        }

        else
          m_counter_detection = 0;

      }


    }
    else
    {
      m_tracker->initClick(I, m_model + ".init", true);
      m_state = tracking;
    }
  } // End State Detection


  if (m_state == tracking ) {

    //    vpDisplay::displayText(I, 12, 10, "Tracking...", vpColor::red);

    try
    {
      m_tracker->track(I);
      m_tracker->getPose(m_cMo);
      //printPose("cMo teabox: ", cMo_teabox);
      if (!m_checkValiditycMo(m_cMo))
        std::cout << "OK";
      // m_tracker->display(I, m_cMo, m_cam, vpColor::red, 2);
      //vpDisplay::displayFrame(I, m_cMo, m_cam, 0.025, vpColor::none, 3);
      status_tracking = true;

    }
    catch(...)
    {
      std::cout << "Exception tracking" << std::endl;
      m_state = detection;
      //m_tracker->resetTracker();
      //m_tracker->reInitModel(I,m_model,m_cMo);

      status_tracking = false;
    }
  } // End State Tracking

  return status_tracking;
}


/*!
  Detect the object in a image and compute pose. TO DELETE
  * \param I
  * \param cMo
 */
bool vpMbLocalization::detectObject(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo)
{
  double error, elapsedTime;
  return m_keypoint_detection->matchPoint(I, m_cam, cMo, error, elapsedTime);
}
/*!
  Init the detection loading the learning data.
 */
void vpMbLocalization::initDetection(const std::string &name_file_learning_data)
{
  m_keypoint_detection->loadLearningData(name_file_learning_data, true);
  m_init_detection = true;
}


vpMbLocalization::~vpMbLocalization()
{
  delete m_tracker;
  delete m_keypoint_learning;
  delete m_keypoint_detection;

}
