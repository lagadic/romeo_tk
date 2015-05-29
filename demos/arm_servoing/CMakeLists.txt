set(source 
    servo_arm_qrcode.cpp
    servo_arm_qrcode_joint_avoidance.cpp
    servo_arm_qrcode_joint_avoidance_2dFeatures.cpp
    servo_arm_qrcode_manipulability.cpp
    servo_2arms_qrcode.cpp
  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()

#qi_use_lib(servo_arm_qrcode romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
#qi_use_lib(servo_arm_qrcode_joint_avoidance romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
#qi_use_lib(servo_arm_qrcode_joint_avoidance_2dFeatures romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)


