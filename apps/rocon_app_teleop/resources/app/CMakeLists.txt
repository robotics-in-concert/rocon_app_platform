###############################################################################
# Files
###############################################################################

# <app>.yaml currently gets handled separately, though I put it in the debian so
# a repo script can extract it at some later date.

set(SCRIPT_FILES
    audio.sh
    video.sh
    teleop.sh
)
set(APP_FILES
    teleop.launch
    teleop.yaml
    teleop.app
    teleop.interface
    teleop.png
    CMakeLists.txt
)

install(FILES ${APP_FILES} DESTINATION ${PROJECT_NAME}/resources/app )
install(FILES ${SCRIPT_FILES} DESTINATION ${PROJECT_NAME}/resources/app PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
