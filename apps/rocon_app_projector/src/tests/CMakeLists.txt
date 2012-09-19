###############################################################################
# Files
###############################################################################

#file(GLOB SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)

install(FILES test_background_change.cpp CMakeLists.txt
	DESTINATION ${PROJECT_ROS_REL_ROOT}/src/tests
	)

install(FILES CMakeLists.txt DESTINATION ${PROJECT_NAME}/src/tests )
