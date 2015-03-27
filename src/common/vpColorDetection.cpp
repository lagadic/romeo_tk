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

#include <vpColorDetection.h>
#include <fstream>


/*!
  Default constructor.
 */
vpColorDetection::vpColorDetection() :
  m_init_learning(0), m_learning_phase(0) ,m_min_obj_area(400), m_max_obj_area(100000),
  m_max_objs_num(10), m_name("object"), m_objects(), m_trackbarWindowName("Trackbars"),
  m_T()

{
  m_H_min = 0;
  m_H_max = 255;
  m_S_min = 0;
  m_S_max = 255;
  m_V_min = 0;
  m_V_max = 255;
}


void vpColorDetection::setValuesHSV(const int H_min, const int S_min, const int V_min,
                                    const int H_max, const int S_max, const int V_max)
{
  m_H_min = H_min;
  m_H_max = H_max;
  m_S_min = S_min;
  m_S_max = S_max;
  m_V_min = V_min;
  m_V_max = V_max;
}

void vpColorDetection::setValuesHSV(const std::vector<int> values)
{
  setValuesHSV(values[0], values[1], values[2], values[3], values[4], values[5]);
}


std::vector<int> vpColorDetection::getValueHSV()
{
  std::vector<int> values(6);

  values[0]= m_H_min;
  values[1]= m_H_max;
  values[2]= m_S_min;
  values[3]= m_S_max;
  values[4]= m_V_min;
  values[5]= m_V_max;

  return values;
}


bool vpSortLargestObject(cv::Rect rect1, cv::Rect rect2)
{
  return (rect1.area() > rect2.area());
}


bool vpColorDetection::learningColor(const cv::Mat &I)
{
  if (!m_init_learning)
  {
    createTrackbars();
    m_init_learning = 1;
    m_learning_phase = 1;
  }

  bool result = detect(I);
  cv::waitKey(1) ;
  return result;

}

/*!
  Create window for trackbars: set HSV values
*/
void vpColorDetection::createTrackbars(){

  std::cout << "Created TrackBars" << std::endl;
  cv::namedWindow(m_trackbarWindowName,0);
  cv::createTrackbar( "H_MIN", m_trackbarWindowName, &m_H_min, m_H_max );
  cv::createTrackbar( "H_MAX", m_trackbarWindowName, &m_H_max, m_H_max );
  cv::createTrackbar( "S_MIN", m_trackbarWindowName, &m_S_min, m_S_max );
  cv::createTrackbar( "S_MAX", m_trackbarWindowName, &m_S_max, m_S_max );
  cv::createTrackbar( "V_MIN", m_trackbarWindowName, &m_V_min, m_V_max );
  cv::createTrackbar( "V_MAX", m_trackbarWindowName, &m_V_max, m_V_max );

}

/*!
  Create structuring element that will be used to "dilate" and "erode" image.
  \param T : Image openCV to process.
*/
void vpColorDetection::morphOps(cv::Mat &T){

  cv::Mat erodeElement = getStructuringElement( cv::MORPH_RECT,cv::Size(3,3)); //3
  //dilate with larger element so make sure object is nicely visible
  cv::Mat dilateElement = getStructuringElement( cv::MORPH_RECT,cv::Size(8,8)); //8

  cv::erode(T,T,erodeElement);
  cv::erode(T,T,erodeElement);

  cv::dilate(T,T,dilateElement);
  cv::dilate(T,T,dilateElement);

}

/*!
   Allows to detect an object with defined color range in the image. When more than one object is detected, objects are sorted from largest to smallest.

   \param I : Input image to process.
   \return true if one or more object are found, false otherwise.

   The number of detected objects is returned using getNbObjects().
   If a object is found the functions getBBox(), getCog() return some information about the location of the object.

   The largest object is always available using getBBox(0) or getCog(0).
 */

bool vpColorDetection::detect(const cv::Mat &I)

{
  bool detected = false;

  cv::Mat HSV;
  cv::Mat T; //Treshold
  cv::cvtColor(I,HSV,cv::COLOR_BGR2HSV);
  cv::inRange(HSV,cv::Scalar(m_H_min,m_S_min,m_V_min),cv::Scalar(m_H_max,m_S_max,m_V_max),T);
  morphOps(T);
  detected = trackFilteredObject(T);
  if (m_learning_phase)
  {
   //cv::imshow("Original",I);
    cv::imshow("HSV Image",HSV);
    cv::imshow("Treshold",T);
  }
  return detected;
}


/*!
   Find contours, the centroid and the boundary box of the objects
   \param T : Treshold image to process.
   \return true if one or more object are found, false otherwise.
 */

bool vpColorDetection::trackFilteredObject(cv::Mat threshold)
{
  m_nb_objects = 0;
  m_polygon.clear();
  m_objects.clear();

  //cv::Mat drawing = cv::Mat::zeros( I.size(), CV_8UC3 );
  //cv::RNG rng(12345);

  int x = 0;
  int y = 0;
  cv::Mat temp;
  threshold.copyTo(temp);
  //these two vectors needed for output of findContours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  //find contours of filtered image using openCV findContours function
  cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  //use moments method to find our filtered object
  bool objectFound = false;
  if (hierarchy.size() > 0)
  {
    int numObjects = hierarchy.size();
    //if number of objects greater than m_max_objs_num we have a noisy filter
    if(numObjects < m_max_objs_num)
    {

      for (int index = 0; index >= 0; index = hierarchy[index][0])
      {
        cv::Moments moment = cv::moments((cv::Mat)contours[index]);
        double area = moment.m00;

        //if the area is less than m_min_obj_area then it is probably just noise
        //if the area bigger than m_max_obj_area, probably just a bad filter
        //we only want the object with the largest area so we safe a reference area each
        //iteration and compare it to the area in the next iteration.
        if(area >= m_min_obj_area && area <= m_max_obj_area)
        {
          x = moment.m10/area;
          y = moment.m01/area;
          //drawObject(x,y,I);
          m_nb_objects++;

          //cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          //cv::drawContours( I, contours, index, color, 2, 8, hierarchy, 0, cv::Point() );


          cv::Rect rect = cv::boundingRect(contours[index]);

          //cv::rectangle(I, cv::Point(rect.x, rect.y),cv::Point(rect.x+rect.width, rect.y+rect.height),cv::Scalar(0, 0, 255, 0),2, 8, 0);

          m_objects.push_back(rect);

        }

      }

    }

  }

  if (m_nb_objects > 0)
  {
    objectFound = true;
    std::sort(m_objects.begin(), m_objects.end(), vpSortLargestObject);

    for( size_t i = 0; i < m_objects.size(); i++ )
    {
      std::ostringstream message;
      message << m_name << " " << int(i);
      m_message.push_back( message.str() );

      std::vector<vpImagePoint> polygon;
      double x = m_objects[i].tl().x;
      double y = m_objects[i].tl().y;
      double w = m_objects[i].size().width;
      double h = m_objects[i].size().height;

      polygon.push_back(vpImagePoint(y  , x  ));
      polygon.push_back(vpImagePoint(y+h, x  ));
      polygon.push_back(vpImagePoint(y+h, x+w));
      polygon.push_back(vpImagePoint(y  , x+w));

      m_polygon.push_back(polygon);
    }

  }

  //  cv::imshow("Cont",drawing);
  return objectFound;

}


void vpColorDetection::drawObject(int &x, int &y, cv::Mat &frame){

  cv::Size s = frame.size();

  cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,255,0),1);
  if(y-10>0)
    cv::line(frame,cv::Point(x,y),cv::Point(x,y-10),cv::Scalar(0,255,0),1);
  else cv::line(frame,cv::Point(x,y),cv::Point(x,0),cv::Scalar(0,255,0),1);
  if(y+10<s.height)
    cv::line(frame,cv::Point(x,y),cv::Point(x,y+10),cv::Scalar(0,255,0),1);
  else cv::line(frame,cv::Point(x,y),cv::Point(x,s.height),cv::Scalar(0,255,0),1);
  if(x-10>0)
    cv::line(frame,cv::Point(x,y),cv::Point(x-10,y),cv::Scalar(0,255,0),1);
  else cv::line(frame,cv::Point(x,y),cv::Point(0,y),cv::Scalar(0,255,0),1);
  if(x+10<s.width)
    cv::line(frame,cv::Point(x,y),cv::Point(x+10,y),cv::Scalar(0,255,0),1);
  else cv::line(frame,cv::Point(x,y),cv::Point(s.width,y),cv::Scalar(0,255,0),1);

  cv::putText(frame, intToString(x)+","+intToString(y),cv::Point(x,y+30),1,1,cv::Scalar(0,255,0),1);

}

/*!
   Convert int to string
   \param number : int to convert
   \return string
 */

std::string vpColorDetection::intToString(int number){

  std::stringstream ss;
  ss << number;
  return ss.str();
}


void vpColorDetection::setMaxAndMinObjectArea(const double &area_min, const double &area_max)
{
  setMinObjectArea(area_min);
  setMaxObjectArea(area_max);
}

/*!
   Save in a file the current value of HSV
   \param filename : name of the file to create.
   \return true if the process has success.
 */
bool vpColorDetection::saveHSV(const std::string &filename)
{

  std::ofstream fileout(filename.c_str(), std::ios_base::out);
  if (!fileout)
  {
    std::cerr << "ERROR: cannot create the file " << filename << std::endl;
    return false;
  }

  fileout << m_name <<" "<< m_H_min <<" "<<  m_H_max <<" "<< m_S_min <<" "<< m_S_max <<" "<< m_V_min <<" "<< m_V_max << std::endl;

  fileout.close();

  return true;

}

/*!
   Load from a file the current value of HSV and save them in the class.
   \param filename : name of the file to read.
   \return true if the process has success.
 */
bool vpColorDetection::loadHSV(const std::string &filename)
{
  std::ifstream filein(filename.c_str(),std::ios_base::in);

  if (!filein)
  {
    std::cerr << "ERROR: cannot open the file " << filename << std::endl;
    return false;
  }

  filein >> m_name >> m_H_min >> m_H_max >> m_S_min >> m_S_max >> m_V_min >> m_V_max;

  std::cout << "Loaded the following value from the file " << filename << ":" << std::endl;
  std::cout << "H_min :" << m_H_min << ":" << std::endl;
  std::cout << "H_max :" << m_H_max << ":" << std::endl;
  std::cout << "S_min :" << m_S_min << ":" << std::endl;
  std::cout << "S_max :" << m_S_max << ":" << std::endl;
  std::cout << "V_min :" << m_V_min << ":" << std::endl;
  std::cout << "V_max :" << m_V_max << ":" << std::endl;

  filein.close();
  return true;

}





