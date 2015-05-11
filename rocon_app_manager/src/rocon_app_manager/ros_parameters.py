#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy

###############################################################################
# Functions
###############################################################################


def setup_ros_parameters():
    '''
      Returns validated parameters for this module from the ros param server.
    '''
    param = {}
    param['robot_type'] = rospy.get_param('~robot_type', 'robot')  #@IgnorePep8
    param['robot_name'] = rospy.get_param('~robot_name', 'app_manager')  #@IgnorePep8
    # image filename
    param['robot_icon'] = rospy.get_param('~robot_icon', '')  #  #@IgnorePep8
    param['auto_start_rapp'] = rospy.get_param('~auto_start_rapp', None)  #@IgnorePep8
    param['rapp_package_whitelist'] = rospy.get_param('~rapp_package_whitelist', [])
    param['rapp_package_blacklist'] = rospy.get_param('~rapp_package_blacklist', [])
    # Todo fix these up with proper whitelist/blacklists
    param['remote_controller_whitelist'] = rospy.get_param('~remote_controller_whitelist', [])
    param['remote_controller_blacklist'] = rospy.get_param('~remote_controller_blacklist', [])
    # Useful for local machine/simulation tests (e.g. chatter_concert)
    param['local_remote_controllers_only'] = rospy.get_param('~local_remote_controllers_only', False)
    # Check if rocon is telling us to be verbose about starting apps (this comes from the
    # rocon_launch --screen option).
    rocon_screen = rospy.get_param('/rocon/screen', False)
    # Also check if a user has privately told this app manager to start with verbose output.
    app_manager_screen = rospy.get_param('~screen', False)
    param['app_output_to_screen'] = rocon_screen or app_manager_screen
    param['auto_rapp_installation'] = rospy.get_param('~auto_rapp_installation', False)

    # to delay services creation until gateway appears (services/topics names should not change after being published)
    param['use_gateway_uuids'] = rospy.get_param('~use_gateway_uuids',True)

    # Preferred rapp configuration
    param['preferred'] = rospy.get_param('~preferred',[])

    # Simulation
    param['simulation'] = rospy.get_param('~simulation', False)

    return param
