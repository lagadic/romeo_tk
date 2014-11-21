
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(romeo_tk)
# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

find_package(visp_naoqi REQUIRED)


# Blob Tracking
qi_create_bin(servo_blob_tracking_visp_all_head demos/blob_tracking/servo_blob_tracking_visp_all_head.cpp)


# Face Tracking
qi_create_bin(face_detection_visp demos/face_tracking/face_detection_visp.cpp)


# Face Servoing
qi_create_bin(servo_face_detection_visp demos/face_servoing/servo_face_detection_visp.cpp)
qi_create_bin(servo_face_detection_visp_head demos/face_servoing/servo_face_detection_visp_head.cpp)
qi_create_bin(servo_face_detection_visp_head_joint_avoidance_basic demos/face_servoing/servo_face_detection_visp_head_joint_avoidance_basic.cpp)


# Arm Servoing
qi_create_bin(servo_blob_tracking_visp_LArm demos/arm_servoing/servo_blob_tracking_visp_LArm.cpp)
qi_create_bin(servo_blob_tracking_visp_LArm_metapod demos/arm_servoing/servo_blob_tracking_visp_LArm_metapod.cpp)
qi_create_bin(servo_visp_LArm_6dof demos/arm_servoing/servo_visp_LArm_6dof.cpp)
qi_create_bin(servo_PB_MB_tracker demos/arm_servoing/servo_PB_MB_tracker.cpp)

# Grasping
qi_create_bin(servo_PB_Grasp demos/grasping/servo_PB_Grasp.cpp)
qi_create_bin(servo_PB_Grasp_Tea demos/grasping/servo_PB_Grasp_Tea.cpp)
qi_create_bin(servo_PB_grasp_tracking_tea demos/grasping/servo_PB_grasp_tracking_tea.cpp)
qi_create_bin(servo_PB_grasp_tracking_tea_move_head demos/grasping/servo_PB_grasp_tracking_tea_move_head.cpp)

# Tests
qi_create_bin(test_transformations tests/test_transformations.cpp)


# General
qi_create_bin(learn_oMh learn_oMh.cpp)



## Create LIBRARY


# Here we say that our executable depends on
# - ALCOMMON (main naoqi lib)
# - ALPROXIES (using specialized naoqi proxies)
# - ALVISION (for vision definitions)
# - OPENCV (face detection)
#
# It automatically links with the corresponding libraries and make their headers
# available.


# Arm Servoing
qi_use_lib(servo_blob_tracking_visp_LArm visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_blob_tracking_visp_LArm_metapod visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_visp_LArm_6dof visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_PB_MB_tracker visp_naoqi ALCOMMON ALPROXIES ALVISION)


# Blob Tracking
qi_use_lib(servo_blob_tracking_visp_all_head visp_naoqi ALCOMMON ALPROXIES ALVISION)

# Face Tracking
qi_use_lib(face_detection_visp visp_naoqi ALCOMMON ALPROXIES ALVISION)


# Face Servoing
qi_use_lib(servo_face_detection_visp visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_face_detection_visp_head visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_face_detection_visp_head_joint_avoidance_basic visp_naoqi ALCOMMON ALPROXIES ALVISION)

# Grasping
qi_use_lib(servo_PB_Grasp visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_PB_Grasp_Tea visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_PB_grasp_tracking_tea visp_naoqi ALCOMMON ALPROXIES ALVISION)
qi_use_lib(servo_PB_grasp_tracking_tea_move_head visp_naoqi ALCOMMON ALPROXIES ALVISION)


# Tests
qi_use_lib(test_transformations visp_naoqi ALCOMMON ALPROXIES ALVISION)

# General
qi_use_lib(learn_oMh visp_naoqi ALCOMMON ALPROXIES ALVISION)

# Propagate in subdirs
subdirs(tools/calibration/3d-grid)

