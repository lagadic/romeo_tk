##
# Copyright (C) 2010 Aldebaran Robotics

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(face_detection)
# this lets you find the qibuild cmake frameworl
find_package(qibuild)



find_package(VISP REQUIRED)
if(VISP_FOUND)
  #add_definitions(${VISP_DEFINITIONS})
  #include_directories(${VISP_INCLUDE_DIRS})
  #link_directories(${VISP_LIBRARY_DIRS})
  #link_libraries(${VISP_LIBRARIES})
endif(VISP_FOUND)

find_package(visp_naoqi REQUIRED)

# Here, we create an executable named "getimages" from the cpp file.
qi_create_bin(face_detection_visp face_detection_visp.cpp)
qi_create_bin(servo_face_detection_visp servo_face_detection_visp.cpp)
qi_create_bin(servo_blob_tracking_visp servo_blob_tracking_visp.cpp)
qi_create_bin(servo_blob_tracking_visp_all_head servo_blob_tracking_visp_all_head.cpp)

# Here we say that our executable depends on
# - ALCOMMON (main naoqi lib)
# - ALPROXIES (using specialized naoqi proxies)
# - ALVISION (for vision definitions)
# - OPENCV (face detection)
#
# It automatically links with the corresponding libraries and make their headers
# available.
qi_use_lib(face_detection_visp visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_face_detection_visp visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_blob_tracking_visp visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_blob_tracking_visp_all_head visp_naoqi ALCOMMON ALPROXIES ALVISION)
