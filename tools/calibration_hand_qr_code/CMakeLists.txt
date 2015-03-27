project(romeo)
cmake_minimum_required(VERSION 2.8)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(pose_from_2_qrcode pose_from_2_qrcode.cpp)
target_link_libraries(pose_from_2_qrcode ${VISP_LIBRARIES})

