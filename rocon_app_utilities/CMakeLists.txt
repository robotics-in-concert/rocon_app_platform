##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_app_utilities)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslint)
catkin_python_setup()
catkin_package()
catkin_add_env_hooks(15.rocon_app SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


file(GLOB_RECURSE ${PROJECT_NAME}_MODULE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
file(GLOB_RECURSE ${PROJECT_NAME}_SCRIPT_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
roslint_python(${${PROJECT_NAME}_MODULE_SOURCES} ${${PROJECT_NAME}_SCRIPT_SOURCES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY shells DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
