/****************************************************************************
 *
 * $Id: calibrate3dGrid-Lagadic.cpp,v 1.7 2007/05/02 13:29:40 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Camera calibration with a 3d calibration grid.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/


/*!
  \example calibrate3dGrid-Lagadic.cpp
  \brief example of camera calibration.
  This example is an implementation of a camera calibration with a non linear
  method based on virtual visual servoing. It uses several images of a unique
  calibration pattern. The calibration pattern used here is available in :
  ViSP-images/calibration/testPattern3D.pgm
  this is a 64*64*64 dots calibration pattern where dots centers are spaced by 0.06 meter.
  You can obviously use another pattern changing its name and its parameters
  in the program.
  Then you have to grab some images of this pattern, save them as PGM files
  and precise their names.
*/
#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpCalibration.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpMouseButton.h>

#include <visp/vpPose.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpXmlParserCamera.h>

#include <visp/vpXmlParserCamera.h>
// List of allowed command line options
#define GETOPTARGS  "di:p:m:hf:g:n:s:c:w:x:z:"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param gridname : calibration grid name.
  \param gray : gray level precision.
  \param first : First image.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.
  \param lambda : Gain of the virtual visual servoing.

 */
void usage(const char *name, const char *badparam, std::string ipath,
           std::string &ppath, std::string &xml_ipath, std::string &camera_name,
	   std::string &gridname,
           double gray, double shapePrecision, double sizePrecision, unsigned first, unsigned nimages, unsigned step, double lambda)
{
  fprintf(stdout, "\n\
  Read images of a calibration pattern from the disk and \n\
  calibrate the camera used for grabbing it.\n\
  Each image corresponds to a PGM file.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-x <xml calibration file name in input>] [-c <camera name>]\n\
     [-m <calibration grid name>] \n\
     [-g <gray level precision>] [-z <size precision>] [-w <shape precision>] [-f <first image>] \n\
     [-n <number of images>] [-s <step>] [-l lambda] \n\
     [-c] [-d] [-h] \n\
 ", name);

 fprintf(stdout, "\n\
 OPTIONS:                                               Default\n\
  -i <test image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/calibration/pattern-36-%%02d.pgm\"\n\
     images. These images come from ViSP-images-x.y.z.tar.gz\n\
     available on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/calibration/grid36-%%02d.pgm\"\n\
     %%02d is for the image numbering.\n\
 \n\
  -x <xml calibration file name in input>              %s\n\
     If set we use the parameters from the camera with\n\
     name set with \"-c <camera name>\" option.\n\
\n\
  -c <camera name>                                     %s\n\
     If this camera is found in the xml file set with\n\
     \"-x <xml calibration file name in input>\" option,\n\
     the parameters are used as initialisation.\n\
\n\
  -m <calibration grid name>                           %s\n\
     Specify the calibration grid to process.\n\
 \n\
  -g <gray level precision>                            %f\n\
     Specify a gray level precision to detect dots.\n\
     A number between 0 and 1.\n\
     precision of the gray level of the dot. \n\
     It is a double precision float witch \n\
     value is in ]0,1]. 1 means full precision, \n\
     whereas values close to 0 show a very bad \n\
     precision.\n\
 \n\
  -w <shape precision>                                 %f\n\
     Specify an ellipsoid shape precision to detect dots.\n\
     A number between 0 and 1.\n\
     \n\
 \n\
  -z <size precision>                                  %f\n\
     Specify a size precision to detect dots.\n\
     A number between 0 and 1.\n\
     \n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -n <number of images>                                %u\n\
     Number of images used to compute calibration.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
 \n\
  -l <lambda>                                          %f\n\
     Gain of the virtual visual servoing.\n\
 \n\
  -d                                             \n\
     Disable the image display. This can be usefull \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
\n\
  -h\n\
     Print the help.\n\n",
	 ipath.c_str(),ppath.c_str(), xml_ipath.c_str(), 
	 camera_name.c_str(), gridname.c_str(), gray, shapePrecision, sizePrecision, first, nimages, step, lambda);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param gridname : calibration grid name.
  \param gray : gray level precision.
  \param first : First image.
  \param nimages : Number of images to display.
  \param step : Step between two images.
  \param lambda : Gain of the virtual visual servoing.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be usefull for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, 
                std::string &xml_ipath, std::string &camera_name,
                std::string &gridname, double &gray, double &shapePrecision, double &sizePrecision, unsigned &first,
                unsigned &nimages, unsigned &step,
                double &lambda, bool &display)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'p': ppath = optarg; break;
    case 'x': xml_ipath = optarg; break;
    case 'c': camera_name = optarg; break;
    case 'm': gridname = optarg; break;
    case 'g': gray = atof(optarg);break;
    case 'w': shapePrecision = atof(optarg);break;
    case 'z': sizePrecision = atof(optarg);break;
    case 'f': first = (unsigned) atoi(optarg); break;
    case 'n': nimages = (unsigned) atoi(optarg); break;
    case 's': step = (unsigned) atoi(optarg); break;
    case 'l': lambda = atof(optarg); break;
    case 'h': usage(argv[0], NULL, ipath, ppath, xml_ipath, camera_name,
                    gridname, gray, shapePrecision, sizePrecision, first, nimages, step, lambda);
      return false; break;

    default:
      usage(argv[0], optarg, ipath, ppath, xml_ipath, camera_name,
            gridname, gray, shapePrecision, sizePrecision, first, nimages, step, lambda);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath, xml_ipath, camera_name,
          gridname, gray, shapePrecision, sizePrecision, first, nimages, step, lambda);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}
/*!
  Find a string in a file.

  \param pFic : Pointer to a FILE object that identifies the stream where
  characters are read from.
  \param source : Pointer to an array of chars where the string to compare is stored.

  \return true if the string is found.

  Sample code :
  Suppose we have a file were "mystring" is followed by a float mydata
  on the next line.
  this function retrieve "mystring" and place the stream pointer just
  the line after.
  \code
  #include <visp/vpIoTools.h>
  #include <visp/vpDebug.h>
  int main()
  {
  FILE * pFile;
  char* mystring = "mystring";
  float mydata;
  pFile = fopen ("myfile.txt" , "r");
  if (pFile == NULL)
  vpERROR_TRACE("Error opening file");
  else {
  if (!vpIoTools::FSearchString (pFile, mystring))
  vpERROR_TRACE("%s has not been found",mystring);
  std::fscanf(%f,&mydata);
  std::fclose (pFile);

  std::printf("%f",mydata);
  }
  return 0;
  }
  \endcode
*/
bool
    FSearchString(FILE* pFic, const char *source)
{

  char   string[FILENAME_MAX];
  char*  str_ret = NULL;
  while( str_ret == NULL ) /* if str_ret != NULL, the line is found */
  {
    /* A line is read */
    str_ret = std::fgets(string, FILENAME_MAX, pFic);
    if( str_ret == NULL ){
      return(false);  /* End of file or read issue */
    }
    str_ret = (char *)strstr(string, source);
  }
  return(true);
}

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

typedef enum {
  valid_pose, // pose valid, we continue
  dont_care, // bad dots, we don't care of this pose
  retry,
  modif_points,
  add_point,
  remove_point
}t_menu_state;
t_menu_state menu_validation_dots(vpImage<unsigned char> &I,
                                  bool *valid, unsigned int nbpt,
                                  vpDot2 *md, vpPoint* mP,
                                  vpCameraParameters &camTmp,
                                  vpHomogeneousMatrix &cMoTmp,
                                  double gray_level, double shape_precision, double size_precision) ;
t_menu_state menu_add_remove(vpImage<unsigned char> &I,
                             bool *valid, unsigned int nbpt,
                             vpDot2 *md, vpCameraParameters &camTmp,
                             vpHomogeneousMatrix &cMoTmp,
                             double gray_level, double shape_precision, double size_precision);
t_menu_state menu_modif_retry(vpImage<unsigned char> &I,
                              bool *valid, unsigned int nbpt,
                              vpDot2 *md, vpPoint* mP,
                              vpCameraParameters &camTmp,
                              vpHomogeneousMatrix &cMoTmp,
                              double gray_level, double shape_precision, double size_precision);

void display_point(vpImage<unsigned char> &I,
                   bool *valid, unsigned int nbpt,
                   vpDot2 *md, vpPoint* mP,
                   vpCameraParameters &camTmp,
                   vpHomogeneousMatrix &cMoTmp)
{
  vpImagePoint valid_cog, projected_cog;

  vpDisplay::display(I);
  vpColVector _cP, _p ;
  for (unsigned int i=0 ; i < nbpt ; i++) {
    if (valid[i]) {
      valid_cog = md[i].getCog();
      vpDisplay::displayCross(I, valid_cog, 10, vpColor::blue, 3);
      mP[i].changeFrame(cMoTmp, _cP) ;
      mP[i].projection(_cP,_p) ;
      vpMeterPixelConversion::convertPoint(camTmp,_p[0],_p[1], projected_cog);
      //       std::cout << "[" << i << "] 2D detecte: " << valid_cog
      // 		<< " projete: " << projected_cog << std::endl;
      vpDisplay::displayCross(I, projected_cog, 10, vpColor::green, 3);
    }
  }
	

  vpDisplay::flush(I);
}
void add_remove_point(vpImage<unsigned char> &I, 
                      t_menu_state state,
                      const vpImagePoint &cog,
                      bool *valid, unsigned int nbpt,
                      vpDot2 *md, vpPoint* mP, vpCameraParameters &camTmp,
                      vpHomogeneousMatrix &cMoTmp,
                      double gray_level, double shape_precision, double size_precision)

{
  vpDot2 new_dot;

  try {
    new_dot.setGraphics(true);
    new_dot.setGrayLevelPrecision(gray_level);
    new_dot.setSizePrecision(size_precision);
    new_dot.setEllipsoidShapePrecision(shape_precision);
    new_dot.initTracking(I, cog);
    vpDisplay::flush(I);
    vpImagePoint new_cog = new_dot.getCog();
    if (state == remove_point) {
      for (unsigned int i=0 ; i < nbpt ; i++) {
        if (valid[i]) {
          vpImagePoint valid_cog = md[i].getCog();
          if ( (fabs(valid_cog.get_u() - new_cog.get_u()) < 2) &&
               (fabs(valid_cog.get_v() - new_cog.get_v()) < 2) ) {
            std::cout << "We remove point [" << i
                << "] with coordinates 2D " << new_cog
                << " and 3D ("
                << mP[i].get_oX() << ", "
                << mP[i].get_oY() << ", "
                << mP[i].get_oZ() << ")" << std::endl;
            valid[i] = false;
          }
        }
      }
    }
    else if (state == add_point) {
      vpColVector _cP, _p ;
      vpImagePoint projected_cog;
      double min_l2norm = 100000., l2norm;
      int index = -1;
      vpRect area(10, 10, I.getWidth()-10, I.getHeight()-10);

      //     std::cout << "Cog to add with germ " << new_cog << std::endl;
      for (unsigned int i=0 ; i < nbpt ; i++) {
        mP[i].changeFrame(cMoTmp, _cP) ;
        mP[i].projection(_cP,_p) ;
        vpMeterPixelConversion::convertPoint(camTmp,_p[0],_p[1], projected_cog);
        vpDot2 dot;
        //dot.setGraphics(true);
        if (projected_cog.inRectangle(area)) {
          try {
            // 	  std::cout << "[" << i << "] Try to found a cog at germ "
            // 		    << projected_cog ;

            dot.setGrayLevelPrecision(gray_level);
            dot.setSizePrecision(size_precision);
            dot.setEllipsoidShapePrecision(shape_precision);
            dot.initTracking(I, projected_cog);
            //vpDisplay::flush(I);
            projected_cog = dot.getCog();
            //std::cout << "Cog to compare with " << projected_cog << std::endl;
            l2norm = (projected_cog.get_u() - new_cog.get_u())*(projected_cog.get_u() - new_cog.get_u()) + (projected_cog.get_v() - new_cog.get_v())*(projected_cog.get_v() - new_cog.get_v());

            // 	  std::cout << "norm: " << l2norm << std::endl;
            if (l2norm < min_l2norm) {
              min_l2norm = l2norm;
              index = i;
            }
          }
          catch(...) {
            std::cout << "cannot found the corresponding dot a " << projected_cog
                << " projected location" << std::endl;
          }
        }
      }
      if (index == -1) {
        std::cout << "Cannot match 2D point " << new_cog << " with 3D point" << std::endl;
      }
      else {
        std::cout << "We add point [" << index
            << "] with coordinates 2D " << new_cog
            << " and 3D ("
            << mP[index].get_oX() << ", "
            << mP[index].get_oY() << ", "
            << mP[index].get_oZ() << ")" << std::endl;

        md[index] = new_dot;
        valid[index] = true;
      }
    }

    display_point(I, valid, nbpt, md, mP, camTmp, cMoTmp);
  }
  catch(...) {
    std::cout << "Add or remove fails" << std::endl;
  }

}


// menu level 3
t_menu_state menu_add_remove(vpImage<unsigned char> &I,
                             bool *valid, unsigned int nbpt,
                             vpDot2 *md, vpPoint* mP,
                             vpCameraParameters &camTmp,
                             vpHomogeneousMatrix &cMoTmp,
                             double gray_level, double shape_precision, double size_precision)
{
  std::cout << "\nA left click in a dot to add the selected point." << std::endl;
  std::cout << "A middle click in a dot to remove the selected point." << std::endl;
  std::cout << "A right click to continue." << std::endl;
  vpMouseButton::vpMouseButtonType button;
  vpImagePoint cog;
  t_menu_state state;
  do {
    vpDisplay::getClick(I, cog, button) ;
    switch(button){
    case 1 : //left
      state = add_point;
      //std::cout << "add point" << std::endl;
      add_remove_point(I, state, cog, valid, nbpt, md, mP, camTmp, cMoTmp, gray_level, shape_precision, size_precision);
      break;
    case 2 : //middle
      state = remove_point;
      //std::cout << "remove point" << std::endl;
      add_remove_point(I, state, cog, valid, nbpt, md, mP, camTmp, cMoTmp, gray_level, shape_precision, size_precision);
      break;
    }
  } while(button != 3);
  state = menu_validation_dots(I, valid, nbpt, md, mP, camTmp, cMoTmp, gray_level, shape_precision, size_precision);
  return state;
}

// menu level 2
t_menu_state menu_modif_retry(vpImage<unsigned char> &I,
                              bool *valid, unsigned int nbpt,
                              vpDot2 *md, vpPoint* mP,
                              vpCameraParameters &camTmp,
                              vpHomogeneousMatrix &cMoTmp,
                              double gray_level, double shape_precision, double size_precision)
{
  std::cout << "\nA left click to add/remove points." << std::endl;
  std::cout << "A right click to retry." << std::endl;
  vpMouseButton::vpMouseButtonType button;
  vpImagePoint cog;
  vpDisplay::getClick(I, cog, button) ;
  t_menu_state state;
  switch(button){
  case 1 : //left
    state = modif_points;
    state = menu_add_remove(I, valid, nbpt, md, mP, camTmp, cMoTmp, gray_level, shape_precision, size_precision);
    break;
  case 3 : //right
    state = retry;
    break;
  }
  return state;
}

// menu level 1
t_menu_state menu_validation_dots(vpImage<unsigned char> &I,
                                  bool *valid, unsigned int nbpt,
                                  vpDot2 *md, vpPoint* mP,
                                  vpCameraParameters &camTmp,
                                  vpHomogeneousMatrix &cMoTmp,
                                  double gray_level, double shape_precision, double size_precision)
{
  std::cout << "\nA left click to validate this pose." << std::endl;
  std::cout << "A middle click to don't care of this pose." << std::endl;
  std::cout << "A right click to remove/add point or retry." << std::endl;
  vpMouseButton::vpMouseButtonType button;
  vpImagePoint cog;
  vpDisplay::getClick(I, cog, button) ;
  t_menu_state state;
  switch(button){
  case 1 : //left
    state = valid_pose;
    break;
  case 2 : //middle = don't care
    state = dont_care;
    break;
  case 3 : //right
    // add/remove or retry
    state = menu_modif_retry(I, valid, nbpt, md, mP, camTmp, cMoTmp, gray_level, shape_precision, size_precision);
    break;
  }
  return state;
}

int main(int argc, const char ** argv)
{
  ///////////////////////////////////////////
  //---------PARAMETERS--------------------

  // set the calibration method
  vpCalibration::vpCalibrationMethodType calibMethod = vpCalibration::CALIB_VIRTUAL_VS_DIST ;

  // set the camera intrinsic parameters
  // see more details about the model in vpCameraParameters
  double px = 600 ;
  double py = 600 ;
  double u0 = 0;
  double v0 = 0;
  vpCameraParameters cam_tmp;
  vpCameraParameters cam; // without distortion
  vpCameraParameters cam_dist; // with distortion
  //set tracking dots parameters
  double opt_sizePrecision = 0.5 ;
  double opt_shapePrecision = 0.65 ;
  //  double ellipsoidShapePrecision = 1 ;
  //set the 3D coordinates of points used to compute the initial pose
  const unsigned int nptPose = 5;
  vpPoint P[nptPose];
  double L = 0.06;
  //plan xOy
  //    P[0].setWorldCoordinates(L,L, 0) ; // (X,Y,Z)
  //    P[1].setWorldCoordinates(L,4*L, 0 ) ;
  //    P[2].setWorldCoordinates(4*L,4*L, 0 ) ;
  //    P[3].setWorldCoordinates(4*L,L,0 ) ;
  //plan xOy
  //     P[0].setWorldCoordinates(L,L, 0 ) ; // dot to small
  P[0].setWorldCoordinates(3*L,3*L, 0 ) ;
  //plan xOz
  P[1].setWorldCoordinates(L,0, -L ) ;
  P[2].setWorldCoordinates(3*L,0, -3*L ) ;
  //     //plan yOz
  P[3].setWorldCoordinates(0,L, -L ) ;
  P[4].setWorldCoordinates(0,3*L, -3*L);

  std::list<double> LoX,LoY,LoZ; //3D coordinates of the calibration dots
  unsigned int nbpt; //number of points in the calibration pattern


  //---------------------------------------------------
  ///////////////////////////////////////////////////////
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string opt_ppath;
  std::string opt_xml_ipath; // existing xml file name containing a previous calibration used to init the intrinsic camera parameters
  std::string opt_camera_name; // camera name set in the xml file in input (opt_xml_ipath)
  std::string dirname;
  std::string filename;
  std::string filename_out;
  std::string opt_gridname = "/tmp/mire3p.dat";
  double opt_gray = 0.8;
  unsigned opt_first = 1;
  unsigned opt_nimages = 4;
  unsigned opt_step = 1;
  double opt_lambda = 0.5;
  bool opt_display = true;
  double dotSize;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_ppath, 
                 opt_xml_ipath, opt_camera_name,
                 opt_gridname, opt_gray, opt_shapePrecision, opt_sizePrecision,
                 opt_first, opt_nimages, opt_step, opt_lambda, opt_display) == false) {
    return (-1);
  }

  if(vpCalibration::readGrid(opt_gridname.c_str(),nbpt,LoX,LoY,LoZ)!=0){
    vpCERROR << "Can't read " << opt_gridname << std::endl;
    return(-1);
  }

  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (opt_ipath.empty() && opt_ppath.empty()) {
    if (ipath != env_ipath) {
      std::cout << std::endl
          << "WARNING: " << std::endl;
      std::cout << "  Since -i <visp image path=" << ipath << "> "
          << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
          << "  we skip the environment variable." << std::endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty() && opt_ppath.empty() ){
    usage(argv[0], NULL, ipath, opt_ppath, opt_xml_ipath, opt_camera_name,
          opt_gridname,opt_gray, opt_shapePrecision, opt_sizePrecision, opt_first,
          opt_nimages, opt_step, opt_lambda);
    std::cerr << std::endl
	      << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	      << std::endl
	      << "  environment variable to specify the location of the " << std::endl
	      << "  image path where test images are located." << std::endl
	      << "  Use -p <personal image path> option if you want to "<<std::endl
	      << "  use personal images." << std::endl
	      << std::endl;

    return(-1);
  }

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  unsigned iter = opt_first;
  std::ostringstream s;
  char cfilename[FILENAME_MAX];

  if (opt_ppath.empty()){


    // Warning :
    // the image sequence is not provided with the ViSP package
    // therefore the program will return you an error :
    //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file
    //        ViSP-images/cube/image.0001.pgm
    //  !!    vpDotExample.cpp: main(#95) :Error while reading the image
    //  terminate called after throwing an instance of 'vpImageException'
    //
    //  The sequence is available on the visp www site
    //  http://www.irisa.fr/lagadic/visp/visp.html
    //  in the download section. It is named "ViSP-images.tar.gz"

    // Set the path location of the image sequence
    dirname = ipath + vpIoTools::path("/ViSP-images/calibration/");

    // Build the name of the image file

    s.setf(std::ios::right, std::ios::adjustfield);
    s << "grid36-" << std::setw(2) << std::setfill('0') << iter << ".pgm";
    filename = dirname + s.str();
  }
  else {

    sprintf(cfilename,opt_ppath.c_str(), iter) ;
    filename = cfilename;
  }
  // Read the PGM image named "filename" on the disk, and put the
  // bitmap into the image structure I.  I is initialized to the
  // correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpImageIo::read(I, filename) ;
  }
  catch(...)
  {
    // an exception is throwned if an exception from readPGM has been catched
    // here this will result in the end of the program
    // Note that another error message has been printed from readPGM
    // to give more information about the error
    std::cerr << std::endl
        << "ERROR:" << std::endl;
    std::cerr << "  Cannot read " << filename << std::endl;
    std::cerr << "  Check your -i " << ipath << " option, " << std::endl
        << "  or your -p " << opt_ppath << " option " <<std::endl
        << "  or VISP_INPUT_IMAGE_PATH environment variable"
        << std::endl;
    return(-1);
  }


  // We determine and store the calibration parameters for each image.
  vpCalibration* table_cal;
  table_cal = new vpCalibration[opt_nimages];
  bool* table_use;
  table_use = new bool[opt_nimages];
  unsigned int niter = 0;

  for(unsigned int i = 0;i<opt_nimages;i++) table_use[i] = true;
  while (iter < opt_first + opt_nimages*opt_step) {
    try {
      // set the new image name

      if (opt_ppath.empty()){
        s.str("");
        s << "grid36-" << std::setw(2) << std::setfill('0') << iter << ".pgm";
        filename = dirname + s.str();
      }
      else {
        sprintf(cfilename, opt_ppath.c_str(), iter) ;
        filename = cfilename;
      }
      filename_out = filename + ".txt";

      std::cout << "read : " << filename << std::endl;
      // read the image
      vpImageIo::read(I, filename);
      u0 = I.getWidth()/2;
      v0 = I.getHeight()/2;

      if (opt_xml_ipath.empty() || opt_camera_name.empty()) {
        cam_tmp.initPersProjWithoutDistortion(px,py,u0,v0);
        std::cout << "Initialise default camera parameters: " << cam_tmp << std::endl;

      }
      else {
        // try to init from an existing calib from xml
        vpXmlParserCamera p; // Create a XML parser
        vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
        // Use a perspective projection model without distorsion
        projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
        // Parse the xml file "myXmlFile.xml" to find the intrinsic camera
        // parameters of the camera named "myCamera" for the image sizes 640x480,
        // for the projection model projModel. The size of the image is optional
        // if camera parameters are given only for one image size.
        if (p.parse(cam_tmp, opt_xml_ipath.c_str(), opt_camera_name.c_str(),
                    projModel, I.getWidth(), I.getHeight()) == vpXmlParserCamera::SEQUENCE_OK) {

          std::cout << "Use camera parameters from " << opt_camera_name.c_str()
              << " camera name. " << std::endl
              << "These parameters are read from "
              << opt_xml_ipath.c_str() << " file" << std::endl;
          std::cout << "Initialise with camera parameters: " << cam_tmp << std::endl;
        }
        else {
          std::cout << "Cannot find camera parameters from " << opt_camera_name.c_str()
              << " camera name. " << std::endl
              << "These parameters are read from "
              << opt_xml_ipath.c_str() << " file" << std::endl;
          cam_tmp.initPersProjWithoutDistortion(px,py,u0,v0);
          std::cout << "Initialise default camera parameters: " << cam_tmp << std::endl;

        }

      }
      
#if defined VISP_HAVE_GDI
      vpDisplayGDI display;
      vpDisplayGDI display_model;
#elif defined VISP_HAVE_GTK
      vpDisplayGTK display;
      vpDisplayGTK display_model;
#elif defined VISP_HAVE_X11
      vpDisplayX display;
      vpDisplayX display_model;
#elif defined VISP_HAVE_D3D9
      vpDisplayD3D display;
      vpDisplayD3D display_model;
#endif
      vpImagePoint cog, cogd;
      // Display on the model
      vpImage<unsigned char> Imodel(300,300,255);
      display_model.init(Imodel);
      vpRxyzVector rxyz(-M_PI/4,0,M_PI/4);
      vpRotationMatrix R_model(rxyz);
      vpHomogeneousMatrix cMo_model(0.0,0.0,2.0,0.0,0.0,0.0);
      cMo_model.insert(R_model);
      vpCalibration calib_model;
      calib_model.cMo = cMo_model;
      calib_model.cMo_dist = cMo_model;
      calib_model.cam_dist.initPersProjWithoutDistortion(600,600,150,150);
      calib_model.cam.initPersProjWithoutDistortion(600,600,150,150);
      calib_model.cam_dist = cam_tmp;
      calib_model.cam = cam_tmp;

      std::cout << "1 calib_model.cam_dist: " << calib_model.cam_dist << std::endl;
      std::cout << "1 calib_model.cam: " << calib_model.cam << std::endl;

      std::list<double>::const_iterator it_LoX=LoX.begin();
      std::list<double>::const_iterator it_LoY=LoY.begin();
      std::list<double>::const_iterator it_LoZ=LoZ.begin();
      for (unsigned int i=0;i<nbpt;i++){
        cog.set_u( 0 );
        cog.set_v( 0 );
        calib_model.addPoint(*it_LoX, *it_LoY, *it_LoZ, cog);
        ++it_LoX;
        ++it_LoY;
        ++it_LoZ;
      }

      vpDisplay::display(Imodel);
      vpPose::display(Imodel,cMo_model,calib_model.cam,0.05,vpColor::red);
      calib_model.displayGrid(Imodel,vpColor::black);

      vpPoint Xaxis;
      Xaxis.setWorldCoordinates(0.06,0.0,0.0);
      Xaxis.project(cMo_model);
      double x_model=0,y_model=0 ;
      vpMeterPixelConversion::convertPoint ( calib_model.cam,Xaxis.get_x(),Xaxis.get_y(),x_model,y_model) ;
      vpDisplay::displayCharString(Imodel,(unsigned int)y_model,(unsigned int)x_model,"x",vpColor::red);
      vpDisplay::flush(Imodel);
      // End display on the model


      if (opt_display) {
        // Display the image

        try{
          char displayname[FILENAME_MAX];
          sprintf(displayname,"Pose %d",iter);

          // Display size is automatically defined by the image (I) size
          display.init(I, 100, 100,displayname) ;
          // Display the image
          // The image class has a member that specify a pointer toward
          // the display that has been initialized in the display declaration
          // therefore is is no longuer necessary to make a reference to the
          // display variable.
          vpDisplay::display(I) ;
          vpDisplay::flush(I) ;
        }
        catch(...){
          vpERROR_TRACE("Error while displaying the image") ;
          delete [] table_cal;
          delete [] table_use;
          return(-1);
        }
      }


      // here we track dots on the calibration grid
      vpDot2 d[nptPose] ;
      vpImagePoint click[nptPose];
      unsigned int gmin = 255; //global gray level minimum
      unsigned int gmax = 0;   //global gray level maximum

      double meanSurface = 0;
      try{
        for(unsigned int i=0;i<nptPose;i++) {
          // by using setGraphics, we request to see the edges of the dot
          // in red on the screen.
          // It uses the overlay image plane.
          // The default of this setting is that it is time consumming

          d[i].setGraphics(true) ;
          d[i].setGrayLevelPrecision(opt_gray);
          d[i].setEllipsoidShapePrecision(opt_shapePrecision);
          d[i].setSizePrecision(opt_sizePrecision);
          //	  d[i].setEllipsoidShapePrecision(ellipsoidShapePrecision);

          // tracking is initalized
          // if no other parameters are given to the iniTracking(..) method
          // a right mouse click on the dot is expected
          // dot location can also be specified explicitely in the initTracking
          // method  : d.initTracking(I,u,v)  where u is the column index and v is
          // the row index

          std::printf("Click in the dot of coordinates\nx=%f y=%f z=%f \n",
                      P[i].get_oX(),P[i].get_oY(),P[i].get_oZ());
          if(i==0) std::cout << "This dot is in the plane containing the small dot." << std::endl;
          // Display on the model
          P[i].display(Imodel,cMo_model,calib_model.cam,vpColor::blue);
          vpPoint Number;
          Number.setWorldCoordinates(P[i].get_oX(),P[i].get_oY(),P[i].get_oZ());
          Number.project(cMo_model);
          double numberx=0,numbery=0 ;
          vpMeterPixelConversion::convertPoint ( calib_model.cam,Number.get_x(),Number.get_y(),numberx,numbery) ;
          char strnumber[10];
          std::sprintf(strnumber,"%d",i+1);
          vpDisplay::displayCharString(Imodel,(unsigned int)numbery,(unsigned int)numberx+2,strnumber,vpColor::blue);
          vpDisplay::flush(Imodel);
          //End display on the model

          try{
            d[i].initTracking(I);
            d[i].track(I);
            meanSurface += d[i].getArea();
          }
          catch(...){
          }

          // an expcetion is thrown by the track method if
          //  - dot is lost
          //  - the number of pixel is too small
          //  - too many pixels are detected (this is usual when a "big" specularity
          //    occurs. The threshold can be modified using the
          //    setNbMaxPoint(int) method
          if (opt_display) {
            cog = d[i].getCog();
            vpDisplay::displayCross(I, cog, 10, vpColor::red) ;
            // flush the display buffer
            vpDisplay::flush(I) ;
          }
          if ( d[i].getGrayLevelMin() < gmin )  gmin = d[i].getGrayLevelMin();
          if ( d[i].getGrayLevelMax() > gmax )  gmax = d[i].getGrayLevelMax();
        }

        // Compute the mean surface of the dots used to compute the initial pose
        meanSurface /= nptPose;
        std::cout << "Dot mean surface: " << meanSurface << std::endl;
      }
      catch(vpException e){
        vpERROR_TRACE("Error while tracking dots") ;
        vpCTRACE << e;
        delete [] table_cal;
        delete table_use;
        return(-1);
      }

      // --------------------------------------------------------
      // Now will compute the pose
      //
      // The pose will be contained in an homogeneous matrix cMo
      vpHomogeneousMatrix cMo ;

      // We need a structure that content both the 3D coordinates of the point
      // in the object frame and the 2D coordinates of the point expressed in meter
      // the vpPoint class is ok for that

      //The vpCalibration class mainly contents a list of points (X,Y,Z,u,v)
      vpCalibration calib;
      calib.clearPoint();

      // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
      vpPose pose ;
      //  the list of point is cleared (if that's not done before)
      pose.clearPoint() ;
      // we set the 3D points coordinates (in meter !) in the object/world frame



      // pixel-> meter conversion
      for (unsigned int i=0 ; i < nptPose ; i++){
        // conversion in meter is achieved using
        // x = (u-u0)/px
        // y = (v-v0)/py
        // where px, py, u0, v0 are the intrinsic camera parameters
        double x=0,y=0 ;
        cog = d[i].getCog();
        vpPixelMeterConversion::convertPoint(cam_tmp, cog, x,y)  ;
        P[i].set_x(x) ;
        P[i].set_y(y) ;
      }

      // The pose structure is build, we put in the point list the set of point
      // here both 2D and 3D world coordinates are known
      for (unsigned int i=0 ; i < nptPose ; i++){
        pose.addPoint(P[i]) ; // and added to the pose computation point list

        cog = d[i].getCog();
        //and added to the local calibration points list
        calib.addPoint(P[i].get_oX(),P[i].get_oY(),P[i].get_oZ(), cog);

      }
      // compute the initial pose using Lagrange method followed by a non linear
      // minimisation method
      // Pose by Dementhon it provides an initialization of the pose
      pose.computePose(vpPose::DEMENTHON, cMo) ;

      // the pose is now refined using the virtual visual servoing approach
      // Warning: cMo needs to be initialized otherwise it may diverge
      pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
      //pose.display(I,cMo,cam_tmp, 0.05, vpColor::blue) ;
      vpHomogeneousMatrix cMoTmp = cMo;
      vpCameraParameters camTmp = cam_tmp;
      //compute local calibration to match the calibration grid with the image
      try{
        std::cout << "2 camTmp: " << camTmp << std::endl;
        calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS,cMoTmp,camTmp,false);
        std::cout << "3 camTmp: " << camTmp << std::endl;
      }
      catch(...){

        std::cout << "\nPose computation failed." << std::endl;
        std::cout << "A right click to define other dots." << std::endl;
        std::cout << "A middle click to don't care of this pose." << std::endl;
        vpMouseButton::vpMouseButtonType button;
        vpDisplay::getClick(I, cog, button) ;
        switch(button){
        case 1 :
        case 3 :
          continue;
        case 2 :
          table_use[niter] = false;
          iter += opt_step ;
          niter++;
          continue;
        }
      }
      if (opt_display) {
        // display the compute pose
        pose.display(I,cMoTmp,camTmp, 0.05, vpColor::red) ;
        vpPoint x;
        x.setWorldCoordinates ( 0.1,0.0,0.0 ) ;
        x.track ( cMoTmp ) ;
        double x1=0,y1=0 ;

        vpMeterPixelConversion::convertPoint ( camTmp,x.p[0],x.p[1],x1,y1) ;
        vpDisplay::displayCharString(I,(unsigned int)y1,(unsigned int)x1,"x",vpColor::red);

        vpDisplay::flush(I) ;

        std::cout << "\nA a left click to display grid." << std::endl;
        std::cout << "A right click to define other dots." << std::endl;
        vpMouseButton::vpMouseButtonType button;
        vpDisplay::getClick(I, cog, button) ;
        switch(button){
        case 1 :
          break;
        case 2 :
        case 3 :
          continue;
        }

        vpDisplay::display(I) ;
        vpDisplay::flush(I) ;
      }
      dotSize = 0;
      for(unsigned i =0 ; i<nptPose ;i++){
        dotSize += d[i].getWidth()+d[i].getHeight();
      }
      dotSize /= nptPose;
      //now we detect all dots of the grid
      vpDot2* md = new vpDot2[nbpt];
      for(unsigned int i=0;i<nbpt;i++){

        // by using setGraphics, we request to see the contour of the dot
        // in red on the screen.
        md[i].setGraphics(false);
        md[i].setEllipsoidShapePrecision(opt_shapePrecision);
        md[i].setSizePrecision(opt_sizePrecision);
        md[i].setGrayLevelPrecision(opt_gray);
        //	md[i].setEllipsoidShapePrecision(ellipsoidShapePrecision);
      }

      // --------------------------------------------------------
      // Now we will compute the calibration
      //

      // We need a structure that content both the 3D coordinates of the point
      // in the object frame and the 2D coordinates of the point expressed in meter
      // the vpPoint class is ok for that
      vpPoint* mP=new vpPoint[nbpt]  ;

      // The vpCalibration class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
      //  the list of point is cleared (if that's not done before)
      table_cal[niter].clearPoint() ;

      // we set the 3D points coordinates (in meter !) in the object/world frame
      //xOy plan
      it_LoX=LoX.begin();
      it_LoY=LoY.begin();
      it_LoZ=LoZ.begin();
      for (unsigned int i=0;i<nbpt;i++){
        mP[i].setWorldCoordinates(*it_LoX, *it_LoY, *it_LoZ); // (X,Y,Z)
        ++it_LoX;
        ++it_LoY;
        ++it_LoZ;
      }

      // pixel-> meter conversion
      bool* valid = new bool[nbpt];
      for (unsigned int i=0 ; i < nbpt ; i++){
        vpColVector _cP, _p ;
        valid[i] = true;
        mP[i].changeFrame(cMoTmp,_cP) ;
        mP[i].projection(_cP,_p) ;
        vpMeterPixelConversion::convertPoint(camTmp,_p[0],_p[1], cog);
        if(10<cog.get_u() && cog.get_u()<I.getWidth()-10 && 10<cog.get_v() && cog.get_v()<I.getHeight()-10){
          try {
            md[i].initTracking(I, cog,(unsigned int)dotSize);
            md[i].track(I);
            vpRect bbox = md[i].getBBox();
            cogd = md[i].getCog();
            if(bbox.getLeft()<10 || bbox.getRight()>(double)I.getWidth()-10 ||
               bbox.getTop()<10 || bbox.getBottom()>(double)I.getHeight()-10||
               vpMath::abs(cog.get_u() - cogd.get_u())>20 ||
               vpMath::abs(cog.get_v() - cogd.get_v())>20 ||
               // Test on the dot surface: we accept 80% to 120% variation from
               // the mean surface
               md[i].getArea() < 0.5*meanSurface ||
               md[i].getArea() > 3*meanSurface ||
               // Test on the shape
               md[i].getWidth() > 1.5*(md[i].getWidth() + md[i].getHeight())/2 ||
               md[i].getWidth() < 0.5*(md[i].getWidth() + md[i].getHeight())/2 ||
               md[i].getHeight() > 1.5*(md[i].getWidth() + md[i].getHeight())/2 ||
               md[i].getHeight() < 0.5*(md[i].getWidth() + md[i].getHeight())/2) {

              valid[i] = false;
            }
            // u[i]. v[i] are expressed in pixel
            // conversion in meter
            double x=0,y=0 ;
            vpPixelMeterConversion::convertPoint(camTmp, cogd, x,y)  ;
            mP[i].set_x(x) ;
            mP[i].set_y(y) ;
            if (opt_display) {
              if(valid[i]){
                md[i].display(I,vpColor::red);
                mP[i].display(I,cMoTmp,camTmp) ;
              }
            }
          }
          catch(...){
            valid[i] = false;
          }
        }
        else {valid[i] = false;}
      }
      vpDisplay::flush(I) ;
      // The calibration structure is build, we put in the point list the set of point
      // here both 2D and 3D world coordinates are known
      // and added to the calibration computation point list.

      t_menu_state state = menu_validation_dots(I, valid, nbpt, md, mP, 
                                                camTmp, cMoTmp, opt_gray, opt_shapePrecision, opt_sizePrecision);

      switch(state){
      case valid_pose :
        break;
      case dont_care : 
        table_use[niter] = false;
        for (unsigned int i=0 ; i < nbpt ; i++)
          valid[i]=false;
        break;
      case retry : 
        continue;
      }
      
      //Add valid points in the calibration structure
      for (unsigned int i=0 ; i < nbpt ; i++){
        if(valid[i]){
          cog = md[i].getCog();
          table_cal[niter].addPoint(mP[i].get_oX(),mP[i].get_oY(),mP[i].get_oZ(), cog) ;
        }
      }

      //we free the memory
      delete [] mP;
      delete [] md;
      delete [] valid;

      niter++ ;
    }
    catch(...) {
      return(-1) ;
    }
    iter += opt_step ;
  }

  vpCalibration::setLambda(opt_lambda);
  // Calibrate by a non linear method based on virtual visual servoing
  std::cout << "4 cam_tmp before multi: " << cam_tmp << std::endl;
  vpCalibration::computeCalibrationMulti(calibMethod,opt_nimages,table_cal,cam_tmp, true) ;
  std::cout << "5 cam_tmp after multi: " << cam_tmp << std::endl;

  cam = table_cal[0].cam ;
  cam_dist = table_cal[0].cam_dist ;
  std::cout << "7 cam after multi: " << cam << std::endl;
  std::cout << "8 cam_dist after multi: " << cam_dist << std::endl;


  //CALIB_VIRTUAL_VS 1
  //CALIB_VIRTUAL_VS_DIST  2

  // Compute Tsai calibration for extrinsic parameters estimation

  iter = opt_first;
  niter = 0;
  //Print calibration results for each image
  while (iter < opt_first + opt_nimages*opt_step) {
    try {
      if(table_use[niter] == true){
        // set the new image name

        if (opt_ppath.empty()){
          s.str("");
          s << "grid36-" << std::setw(2) << std::setfill('0') << iter << ".pgm";
          filename = dirname + s.str();
        }
        else {
          sprintf(cfilename, opt_ppath.c_str(), iter) ;
          filename = cfilename;
        }

        std::cout << "read : " << filename << std::endl;
        // read the image
        vpImageIo::read(I, filename);
        std::cout << "\nCompute standard deviation for pose " << niter <<std::endl;
        double deviation, deviation_dist ;
        table_cal[niter].computeStdDeviation(deviation,deviation_dist);
        std::cout << "deviation for model without distortion : "
            << deviation << std::endl;
        std::cout << "deviation for model with distortion : "
            << deviation_dist << std::endl;
        //Display results

#if defined VISP_HAVE_GDI
        vpDisplayGDI display;
#elif defined VISP_HAVE_GTK
        vpDisplayGTK display;
#elif defined VISP_HAVE_X11
        vpDisplayX display;

#elif defined VISP_HAVE_D3D9
        vpDisplayD3D display;
#endif

        if (opt_display) {
          // Display the image

          try{
            char displayname[FILENAME_MAX];
            sprintf(displayname,"Pose %d",iter);
            // Display size is automatically defined by the image (I) size
            display.init(I, 100, 100,displayname) ;
            // Display the image
            // The image class has a member that specify a pointer toward
            // the display that has been initialized in the display declaration
            // therefore is is no longuer necessary to make a reference to the
            // display variable.
            vpDisplay::display(I) ;
          }
          catch(...){
            vpERROR_TRACE("Error while displaying the image") ;
            delete [] table_cal;
            return(-1);
          }
          //Display the data of the calibration (center of the dots)
          table_cal[niter].displayData(I) ;
          //Display grids : estimated center of dots using camera parameters
          table_cal[niter].displayGrid(I) ;
          vpPose pose;
          pose.display(I,table_cal[niter].cMo_dist,table_cal[niter].cam_dist,
                       0.05, vpColor::red) ;
          vpPoint x;
          x.setWorldCoordinates ( 0.1,0.0,0.0 ) ;
          x.track ( table_cal[niter].cMo_dist ) ;
          double x1=0,y1=0 ;

          vpMeterPixelConversion::convertPoint (
              table_cal[niter].cam_dist,x.p[0],x.p[1],x1,y1) ;
          vpDisplay::displayCharString(I,
                                       (unsigned int)y1,
                                       (unsigned int)x1,"x",vpColor::red);

          vpDisplay::flush(I) ;

          std::cout << "A click to continue..." << std::endl;
          vpDisplay::getClick(I);

        }
      }
      niter++;
    }
    catch(...) {
      delete [] table_use;
      delete [] table_cal;
      return(-1) ;
    }
    iter += opt_step ;
  }
  //////////////////////////////////////
  /* save in an xml file*/
#ifdef VISP_HAVE_XML2
  int save = 0;
  std::cout <<"Save intrinsic camera parameters in a xml file ? (yes=1) : "
      << std::endl;
  if (scanf("%d",&save) != 1)
    save = 0;
  if(save == 1){
    std::string filename_xml ;
    std::string cameraname;
    std::cout <<"Give the xml filename where to save results : "<< std::endl;
    std::cin >> filename_xml;
    std::cout <<"Give the camera name : "<< std::endl;
    std::cin >> cameraname;
    vpXmlParserCamera parser;
    parser.save(cam,filename_xml.c_str(),cameraname,I.getWidth(),I.getHeight());
    parser.save(cam_dist,filename_xml.c_str(),cameraname,I.getWidth(),I.getHeight());
  }
#endif

  delete [] table_use;
  delete [] table_cal;
  return(0);
}





#else // (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)...)

int
    main()
{
  vpTRACE("X11 or GTK or GDI or D3D functionnality is not available...") ;
  return 0;
}
#endif // (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_GDI)...)

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
