include_directories(${VISP_INCLUDE_DIRS})
add_executable(calibrate3dGrid-Lagadic calibrate3dGrid-Lagadic.cpp)
target_link_libraries(calibrate3dGrid-Lagadic ${VISP_LIBRARIES})

