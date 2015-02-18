set(source 
    servo_blob_tracking_visp_LArm_metapod.cpp
    servo_visp_LArm_6dof.cpp
    servo_PB_MB_tracker.cpp
    servo_arm_qrcode.cpp
    servo_arm_qrcode_joint_avoidance.cpp
    servo_arm_qrcode_joint_avoidance_2dFeatures.cpp
  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi ALCOMMON ALPROXIES ALVISION)
endforeach()

qi_use_lib(servo_arm_qrcode romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_arm_qrcode_joint_avoidance romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_arm_qrcode_joint_avoidance_2dFeatures romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION)


#qi_create_bin(grasp_box grasp_box.cpp)
#qi_use_lib(grasp_box romeo_tk visp_naoqi ALCOMMON ALPROXIES ALVISION ALMATH)
