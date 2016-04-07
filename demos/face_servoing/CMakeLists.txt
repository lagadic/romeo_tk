set(source 
  servo_face_detection_visp_head.cpp
  servo_face_detection_visp_eyes.cpp
  servo_face_detection_okao_eyes.cpp
  servo_face_detection_okao_eyes_trunk.cpp
  servo_face_detection_okao_eyes_trunk_multithread.cpp
  servo_face_detection_okao_eyes_joint_limits.cpp
  servo_face_detection_visp_eyes_trunk_multithread.cpp
  )

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


