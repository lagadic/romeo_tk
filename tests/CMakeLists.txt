set(source 
  test_transformations.cpp
  barcode-detector-live.cpp
  motion_romeo.cpp
  speech_recognition.cpp
  detection-object-mbt_class.cpp
  tracking_box_head.cpp
  tracking_box_head_detection.cpp
  detect_skin.cpp
  detect_color_track.cpp
  )

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


qi_use_lib(  detection-object-mbt_class romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(  tracking_box_head romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(  tracking_box_head_detection romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
