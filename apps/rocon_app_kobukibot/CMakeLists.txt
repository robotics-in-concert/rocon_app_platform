##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

# Verbosity is useful - helps eclipse find paths also!
set(CMAKE_VERBOSE_MAKEFILE true)

##############################################################################
# YCS-ROS
##############################################################################

#include($ENV{YCS_ROOT}/ycs_tools/ycs_build/ycsbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()


##############################################################################
# Build
##############################################################################

# You should keep this commented for svn as ros (ycs) build options are 
# set globally. It is however, useful when debugging/testing for a single 
# package. The default option if no global ros configuration is set, is 
# RelWithDebInfo.
# 
# Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo) 

set(PROJECT_VERSION "0.1.1")
set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "1")
set(PROJECT_VERSION_PATCH "1")

##############################################################################
# Init - the order of the preceding commands is important!
##############################################################################

#ycsbuild_init()
#rosbuild_init()
rosbuild_include(rocon_build rocon_packaging)

##############################################################################
# Messages/Services
##############################################################################

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

##############################################################################
# Actual project configuration
##############################################################################

# Including a source directory
#add_subdirectory(src)
add_subdirectory(resources)
rocon_apps_roll_deb_pkg()
rocon_upload_app_data_to_yujin_repo(rocon_app_kobukibot kobukibot)

install(FILES CMakeLists.txt Makefile manifest.xml DESTINATION ${PROJECT_NAME} )

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

