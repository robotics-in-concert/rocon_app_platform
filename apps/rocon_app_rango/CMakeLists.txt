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
rocon_apps_roll_deb_pkg(espeak smplayer)
rocon_upload_app_data_to_yujin_repo(rocon_app_rango rango)

install(FILES CMakeLists.txt Makefile manifest.xml readme.txt DESTINATION ${PROJECT_NAME} )

###############################################################################
# Move Downloads
###############################################################################

set(MOVIE_SERVER ${ROCON_YUJIN_REPO_HTTP}/movies)
set(RANGO_URL ${MOVIE_SERVER}/rango.avi)
set(BUCK_URL ${MOVIE_SERVER}/big_buck_bunny-light.avi)
set(RANGO_FILE resources/movies/rango.avi)
set(BUCK_FILE resources/movies/big_buck_bunny.avi)

rocon_download(${RANGO_URL} ${RANGO_FILE})
rocon_download(${BUCK_URL} ${BUCK_FILE})

