#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/hydro-devel/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rosmaster
import sys

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rosmaster.rosmaster_main(argv=[a for a in sys.argv if not ':=' in a])