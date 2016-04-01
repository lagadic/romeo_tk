set(source 
  test_transformations.cpp
  barcode-detector-live.cpp
  qrcode_detector_romeo.cpp
  motion_romeo.cpp
  speech_recognition.cpp
  dcm_test.cpp
  get_velocity.cpp
  color_detection_romeo.cpp
  red_ball_detection_romeo.cpp
  tracker_cylinder.cpp
  get_pose_plot.cpp
  )

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()

set(source1
  detection-object-mbt_class.cpp
  vpColoDetectionExample.cpp
  tracking_box_color.cpp
  vpBlobsTargetTrackerExample.cpp
  test_calibration.cpp
  vpBlobsTargetTracker_two_cameras.cpp
  #template_tracker_test.cpp
)

foreach(src ${source1})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


#qi_use_lib(  detection-object-mbt_class romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
#qi_use_lib(  vpColoDetectionExample romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
#qi_use_lib(  tracking_box_color romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)



subdirs(audio)
