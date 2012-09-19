###############################################################################
# Files
###############################################################################

set(MOVIES
    CMakeLists.txt
    rango.avi
#    big_buck_bunny.avi
)

install(FILES ${MOVIES} DESTINATION ${PROJECT_NAME}/resources/movies )
