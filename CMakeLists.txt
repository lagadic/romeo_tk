cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(romeo_tk)

# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

find_package(visp_naoqi REQUIRED)

message("VISP_NAOQI_LIBRARIES: ${VISP_NAOQI_LIBRARIES}")

set(ROMEOTK_DATA_FOLDER ${PROJECT_SOURCE_DIR}/data)

configure_file(${PROJECT_SOURCE_DIR}/src/common/vpRomeoTkConfig.h.cmake
  ${CMAKE_BINARY_DIR}/include/vpRomeoTkConfig.h
)

# Propagate in subdirs
subdirs(demos)
subdirs(tests)
subdirs(tools)

include_directories(${PROJECT_SOURCE_DIR}/src/common ${CMAKE_BINARY_DIR}/include)

qi_create_lib(romeo_tk
  SRC
    src/common/vpQRCodeTracker.h
    src/common/vpQRCodeTracker.cpp
    src/common/vpFaceTracker.h
    src/common/vpFaceTracker.cpp
    src/common/vpServoArm.h
    src/common/vpServoArm.cpp
    src/common/vpServoHead.h
    src/common/vpServoHead.cpp
    src/common/vpCartesianDisplacement.h
    src/common/vpCartesianDisplacement.cpp
    src/common/vpMbLocalization.h
    src/common/vpMbLocalization.cpp
    src/common/vpColorDetection.h
    src/common/vpColorDetection.cpp
    src/common/vpJointLimitAvoidance.h
    src/common/vpBlobsTargetTracker.h
    src/common/vpBlobsTargetTracker.cpp
    src/common/vpFaceTrackerOkao.h
    src/common/vpFaceTrackerOkao.cpp
    src/common/vpTemplateLocatization.h
    src/common/vpTemplateLocatization.cpp
)

qi_use_lib(romeo_tk visp_naoqi)

qi_stage_lib(romeo_tk)
