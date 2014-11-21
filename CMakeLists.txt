
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(romeo_tk)
# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

find_package(visp_naoqi REQUIRED)


# Propagate in subdirs
subdirs(demos)
subdirs(tests)
subdirs(tools)


## Create LIBRARY
# Here we say that our executable depends on
# - ALCOMMON (main naoqi lib)
# - ALPROXIES (using specialized naoqi proxies)
# - ALVISION (for vision definitions)
# - OPENCV (face detection)
#
# It automatically links with the corresponding libraries and make their headers
# available.

