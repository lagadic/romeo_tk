set(source 
  test_transformations.cpp
  barcode-detector-live.cpp
  motion_romeo.cpp
  speech_recognition.cpp
  )

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


