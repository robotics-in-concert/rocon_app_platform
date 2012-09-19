###############################################################################
# Files
###############################################################################

#file(GLOB SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)

install(FILES main.cpp main_window.cpp qnode.cpp CMakeLists.txt
	DESTINATION ${PROJECT_NAME}/src
	)

install(FILES CMakeLists.txt DESTINATION ${PROJECT_NAME}/src )
