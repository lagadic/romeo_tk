set(source 
  audio_test.cpp
  audio_compute_snr.cpp
  audio_test_base.cpp
  audio_test_romeo.cpp
  audio_test_romeo_neckYaw.cpp
  )

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi QI ALCOMMON ALPROXIES ALVISION)
endforeach()


