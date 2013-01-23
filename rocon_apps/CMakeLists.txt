##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_apps)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(
#    PROGRAMS 
#        scripts/app_manager.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

install(DIRECTORY apps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY sounds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
