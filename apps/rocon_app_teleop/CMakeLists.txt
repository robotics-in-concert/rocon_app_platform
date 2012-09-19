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

set(PROJECT_VERSION   "0.10.1")
set(PROJECT_VERSION_MAJOR  "0")
set(PROJECT_VERSION_MINOR "10")
set(PROJECT_VERSION_PATCH  "1")

###############################################################################
# Rocon
###############################################################################

rosbuild_include(rocon_build rocon_packaging)

###############################################################################
# Project
###############################################################################

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src)
add_subdirectory(resources)
rocon_apps_roll_deb_pkg()
rocon_upload_app_data_to_yujin_repo(rocon_app_teleop teleop)

install(FILES build.xml CMakeLists.txt dependencies.xml mainpage.dox Makefile manifest.xml readme.txt ros.properties DESTINATION ${PROJECT_NAME} )
install(DIRECTORY bin       DESTINATION ${PROJECT_NAME} PATTERN "*~" EXCLUDE PATTERN "*svn*" EXCLUDE )
install(DIRECTORY src       DESTINATION ${PROJECT_NAME} PATTERN "*~" EXCLUDE PATTERN "*svn*" EXCLUDE )
install(DIRECTORY resources DESTINATION ${PROJECT_NAME} PATTERN "*~" EXCLUDE PATTERN "*svn*" EXCLUDE )
