###############################################################################
# Files
###############################################################################

#file(GLOB SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)

install(FILES rango.py 
            DESTINATION ${PROJECT_ROS_REL_ROOT}/src
            PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ 
                        GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
            )

install(FILES CMakeLists.txt DESTINATION ${PROJECT_NAME}/src )
