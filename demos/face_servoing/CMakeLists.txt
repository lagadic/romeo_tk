set(source 
  servo_face_detection_visp_head.cpp
  servo_face_detection_visp_head_joint_avoidance_basic.cpp

  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


