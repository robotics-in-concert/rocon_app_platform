###############################################################################
# CMake
###############################################################################

cmake_minimum_required(VERSION 2.4.6)

###############################################################################
# Ros
###############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

###############################################################################
# Version
###############################################################################

set(PROJECT_VERSION "0.10.2")
set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "10")
set(PROJECT_VERSION_PATCH "2")

###############################################################################
# Rocon
###############################################################################

rosbuild_include(rocon_build rocon_packaging)

###############################################################################
# Project
###############################################################################

add_subdirectory(src)
add_subdirectory(resources)
rocon_apps_roll_deb_pkg() # add rosdeps in here
rocon_upload_app_data_to_yujin_repo(rocon_app_example example)

install(FILES CMakeLists.txt Makefile manifest.xml DESTINATION ${PROJECT_NAME})

