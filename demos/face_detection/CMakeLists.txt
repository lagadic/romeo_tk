set(source 
  face_detection_visp.cpp
  face_detection_okao.cpp
  face_detection_okao_class.cpp
  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi romeo_tk ALCOMMON ALPROXIES ALVISION)
endforeach()


