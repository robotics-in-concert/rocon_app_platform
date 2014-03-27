##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_app_manager)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED roslint)
catkin_package()

catkin_python_setup()

file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/rapp_manager.py
        scripts/pairing_master.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY concert
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
