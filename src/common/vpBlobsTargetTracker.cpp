
#include <vpBlobsTargetTracker.h>
#include <visp/vpDisplay.h>


vpBlobsTargetTracker::vpBlobsTargetTracker()
    : m_colBlob(),  m_state(detection), m_target_found(false), m_P(), m_force_detection(false), m_name("target_blob"),
      m_blob_list(), m_cog(0,0), m_initPose(true), m_numBlobs(4), m_manual_blob_init(false), m_left_hand_target(true)
{

    //m_colBlob = new vpColorDetection;
    m_colBlob.setMaxAndMinObjectArea(150.0,4000.0);
    m_colBlob.setLevelMorphOps(false);

}

vpBlobsTargetTracker::~vpBlobsTargetTracker()
{
}


/*!
    Return the center of gravity location of the tracked bar code.
    */
vpImagePoint vpBlobsTargetTracker::getCog()
{
    return m_cog;
}



bool vpBlobsTargetTracker::track(const cv::Mat &cvI, const vpImage<unsigned char> &I )
{

    if (m_state == detection || m_force_detection) {
        std::cout << "STATE: DETECTION "<< std::endl;

        bool obj_found = m_colBlob.detect(cvI);
        // Delete previuos list of blobs
        m_blob_list.clear();
        m_initPose = true;
        m_target_found = false;

        if (obj_found || m_manual_blob_init) {

            try{

                // std::cout << "TARGET FOUND" << std::endl;

                vpDot2 blob;
                blob.setGraphics(true);
                blob.setGraphicsThickness(1);
                blob.setEllipsoidShapePrecision(0.9);
                if (m_manual_blob_init)
                {
                    vpDisplay::flush(I);
                    blob.initTracking(I);
                }
                else
                {
                    vpImagePoint cog = m_colBlob.getCog(0);
                    vpDisplay::displayCross(I,cog,10, vpColor::red,2 );
                    blob.initTracking(I,cog);
                }
                blob.track(I);

                printf("Dot characteristics: \n");
                printf("  width : %lf\n", blob.getWidth());
                printf("  height: %lf\n", blob.getHeight());
                printf("  area: %lf\n", blob.getArea());
                printf("  gray level min: %d\n", blob.getGrayLevelMin());
                printf("  gray level max: %d\n", blob.getGrayLevelMax());
                printf("  grayLevelPrecision: %lf\n", blob.getGrayLevelPrecision());
                printf("  sizePrecision: %lf\n", blob.getSizePrecision());
                printf("  ellipsoidShapePrecision: %lf\n", blob.getEllipsoidShapePrecision());


                vpDot2 black_blob = blob;
                black_blob.setGrayLevelMax(40);
                black_blob.setGrayLevelMin(0);

                int i,j,aj,ai;

                if(m_left_hand_target)
                {
                    i = blob.getCog().get_i()-blob.getHeight()*2.3;
                    j = blob.getCog().get_j()-blob.getWidth()*3.3;
                    ai = blob.getHeight()*5;
                    aj = blob.getWidth()*5;
                }
                else
                {
                    i = blob.getCog().get_i()-blob.getHeight();
                    j = blob.getCog().get_j()-blob.getWidth()*2.0;
                    ai = blob.getHeight()*4.5;
                    aj = blob.getWidth()*4;

                }




                    //search similar blobs in the image and store them in blob_list
                    //black_blob.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), m_blob_list);
                    //vpDisplay::displayRectangle(I, i, j, ai, aj, vpColor::red, false, 1);
                    black_blob.searchDotsInArea(I, j, i, ai, aj, m_blob_list);

                //        vpDisplay::flush(I);
                //        vpDisplay::getClick(I,true);

                m_blob_list.insert(m_blob_list.begin(),blob);
                std::cout << "SIZE: " << m_blob_list.size() << std::endl;

                if(m_blob_list.size() == m_numBlobs)
                {
                    for(std::list<vpDot2>::iterator it = m_blob_list.begin(); it != m_blob_list.end(); ++it)
                        it->setEllipsoidShapePrecision(0.9);

                    m_state = tracking;
                    m_force_detection = false;
                }
                else
                    std::cout << "Number blobs found is "<< m_blob_list.size() << ". Expected number: " << m_numBlobs << std::endl;
            }
            catch(...) {
                std::cout << "Exception tracking" << std::endl;
                m_target_found = false;
            }

        }
    }
    else if (m_state == tracking) {
        // std::cout << "STATE: TRACKING "<< std::endl;
        try {

            m_cog.set_uv(0.0,0.0);
            for(std::list<vpDot2>::iterator it = m_blob_list.begin(); it != m_blob_list.end(); ++it)
            {
                it->track(I);
                m_cog += it->getCog();
            }


            m_cog /= m_blob_list.size();

            // Display the ACTUAL center of gravity of the object
            //vpDisplay::displayCross(I,cog_tot,10, vpColor::blue,2 );

            //      std::vector<vpImagePoint> corners(m_blob_list.begin(),m_blob_list.end());



            // Now we create a map of VpPoint in order to order the points

            std::map< double,vpImagePoint> poly_verteces;

            double theta;
            for(std::list<vpDot2>::iterator it=m_blob_list.begin(); it != m_blob_list.end(); ++it)
            {
                // Get the cog of the blob
                vpImagePoint cog = it->getCog();

                theta = atan2(cog.get_v() - m_cog.get_v(), cog.get_u() - m_cog.get_u());
                // Insert the vertexes in the map (ordered)
                poly_verteces.insert ( std::pair<double,vpImagePoint>(theta,cog) );

            }

            // Now we create a Vector containing the ordered vertexes

            std::vector<vpImagePoint> poly_vert;
            int index_first= 0;
            unsigned int count = 0;

            for( std::map<double,vpImagePoint>::iterator it = poly_verteces.begin();  it!=poly_verteces.end(); ++it )
            {

                poly_vert.push_back( it->second );
                if (m_blob_list.front().getCog() == it->second )
                    index_first = count;
                count++;

            }

            std::rotate(poly_vert.begin(), poly_vert.begin() + index_first, poly_vert.end());
            //std::cout << "---------------------------------------" << std::endl;
            for(unsigned int j = 0; j<poly_vert.size();j++)
            {
                std::ostringstream s;
                s << j;
                vpDisplay::displayText(I, poly_vert[j], s.str(), vpColor::green);
                //std::cout << "Cog blob " << j << " :" << poly_vert[j] << std::endl;
            }
            computePose(m_P, poly_vert, m_cam, m_initPose, m_cMo);




            bool duplicate = false;
            for(unsigned int i = 0; i < poly_vert.size()-1; i++)
            {
                for(unsigned int j = i+1; j < poly_vert.size(); j++)
                {
                    //std::cout << "Distance " << i <<"-" << j << " :" << vpImagePoint::sqrDistance(poly_vert[i],poly_vert[j] )<< std::endl;
                    if (vpImagePoint::sqrDistance(poly_vert[i],poly_vert[j]) < 5.0)
                        duplicate = true;
                }
            }

            //std::cout << "---------------------------------------" << std::endl;

            // std::cout << "Number blobs found is "<< poly_vert.size() << ". Expected number: " << m_numBlobs << std::endl;

            //      int i = m_blob_list.front().getCog().get_i()-m_blob_list.front().getHeight()*2.3;
            //      int j = m_blob_list.front().getCog().get_j()-m_blob_list.front().getWidth()*3.3;
            //      unsigned int ai = m_blob_list.front().getHeight()*5;
            //      unsigned int aj =m_blob_list.front().getWidth()*5;
            //      vpDisplay::displayRectangle(I,i, j, ai,aj, vpColor::red,false,1);


            if (duplicate)
            {
                m_target_found = false;
                m_state = detection;
                std::cout << "PROBLEM: tracking failed " << m_numBlobs << std::endl;

            }
            else if (poly_vert.size() != m_numBlobs)
            {
                m_target_found = false;
                m_state = detection;
                std::cout << "PROBLEM: Expected number: " << m_numBlobs << std::endl;
            }

            else
                m_target_found = true;

        }
        catch(...) {
            std::cout << "Exception tracking" << std::endl;
            m_state = detection;
            m_target_found = false;
        }
    }
    return m_target_found;
}

//std::vector<vpImagePoint> vpBlobsTargetTracker::getTemplateTrackerCorners(const vpTemplateTrackerZone &zone)
//{
//  std::vector<vpImagePoint> corners_tracked;

//  // Parse all the triangles that describe the zone
//  for (int i=0; i < zone.getNbTriangle(); i++) {
//    vpTemplateTrackerTriangle triangle;
//    // Get a triangle
//    zone.getTriangle(i, triangle);
//    std::vector<vpImagePoint> corners;
//    // Get the 3 triangle corners
//    triangle.getCorners( corners );
//    if (i==0)
//      corners_tracked = corners;
//    else {
//      for(unsigned int m=0; m < corners.size(); m++) { // corners of the 2nd triangle
//        bool already_exists = false;
//        for(unsigned int n=0; n < corners_tracked.size(); n++) { // already registered corners from the 1st triangle
//          if (corners[m] == corners_tracked[n]) {
//            already_exists = true;
//            break;
//          }
//        }
//        if (! already_exists)
//          corners_tracked.push_back(corners[m]);
//      }

//    }
//  }
//  return corners_tracked;
//}

//std::vector<int> vpBlobsTargetTracker::computedTemplateTrackerCornersIndexes(const std::vector<vpImagePoint> &corners_detected, const std::vector<vpImagePoint> &corners_tracked)
//{
//  std::vector<int> corners_tracked_index;
//  for(unsigned int i=0; i < corners_tracked.size(); i++) {
//    for(unsigned int j=0; j < corners_detected.size(); j++) {
//      if (vpImagePoint::distance(corners_tracked[i], corners_detected[j]) < 4) {
//        corners_tracked_index.push_back(j);
//        break;
//      }
//    }
//  }
//  return corners_tracked_index;
//}

//std::vector<vpImagePoint> vpBlobsTargetTracker::orderPointsFromIndexes(const std::vector<int> &indexes, const std::vector<vpImagePoint> &corners)
//{
//  std::vector<vpImagePoint> corners_ordered(corners.size());
//  for(unsigned int i=0; i < corners.size(); i++) {
//    corners_ordered[indexes[i]] = corners[i];
//  }
//  return corners_ordered;
//}

void vpBlobsTargetTracker::computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
                                       const vpCameraParameters &cam, bool &init, vpHomogeneousMatrix &cMo)
{
    vpPose pose;
    double x=0, y=0;
    for (unsigned int i=0; i < point.size(); i ++) {
        vpPixelMeterConversion::convertPoint(cam, corners[i], x, y);
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]);
    }

    if (init) {
        vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
        pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo_dementhon);
        double residual_dementhon = pose.computeResidual(cMo_dementhon);
        pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo_lagrange);
        double residual_lagrange = pose.computeResidual(cMo_lagrange);
        if (residual_dementhon < residual_lagrange)
            cMo = cMo_dementhon;
        else
            cMo = cMo_lagrange;
    }

    pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
    init = false;
}
