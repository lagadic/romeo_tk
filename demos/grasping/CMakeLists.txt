set(source 
  servo_PB_Grasp.cpp
  servo_PB_Grasp_Tea.cpp
  servo_PB_grasp_tracking_tea.cpp
  servo_PB_grasp_tracking_tea_move_head.cpp

  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()


