##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_app_manager)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslint)
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
endif()
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(
    PROGRAMS 
        scripts/concert_client.py
        scripts/standalone.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  
##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()