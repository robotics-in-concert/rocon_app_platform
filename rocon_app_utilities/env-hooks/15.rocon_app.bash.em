# generated from rocon_app_utilities/env-hooks/15.rocon_app_utilities.bash.em

@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/shells/env.bash"
@[else]@
. "@(CMAKE_INSTALL_PREFIX)/share/rocon_app_utilities/shells/env.bash"
@[end if]@
