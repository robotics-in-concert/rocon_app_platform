################################################################################
# Subdirectories
###############################################################################

#add_subdirectory(app)

###############################################################################
# Files
###############################################################################

#file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

set(SCRIPT_FILES
    example.py
)
set(DIR_FILES
    CMakeLists.txt
)

install(FILES ${DIR_FILES} DESTINATION ${PROJECT_NAME}/resources )
install(FILES ${SCRIPT_FILES} DESTINATION ${PROJECT_NAME}/resources PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
