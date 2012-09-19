###############################################################################
# Files
###############################################################################

# <app>.yaml currently gets handled separately, though I put it in the debian so
# a repo script can extract it at some later date.

set(APP_FILES
    rango.yaml
    CMakeLists.txt
    rango.app
    rango.interface
    rango.launch
    rango.png
)

install(FILES ${APP_FILES} DESTINATION ${PROJECT_NAME}/resources/app )