#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rosgraph
import rospy
import rocon_console.console as console

###############################################################################
# Functions
###############################################################################


class StandaloneParameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar robot_type: used for `rocon_uri`_ rapp compatibility checks *['robot']*
    :vartype robot_type: str
    :ivar robot_name: also used for `rocon_uri`_ rapp compatibility checks *['robot']*
    :vartype robot_name: str
    :ivar auto_start_rapp: indicates via a `resource name`_ (e.g. gopher_rapps/delivery) a rapp to launch on startup *[None]*
    :vartype auto_start_rapp: str
    :ivar rapp_package_whitelist: restrict rapp search (default is the whole workspace) to these packages *[[]]*
    :vartype rapp_package_whitelist: [ str ]
    :ivar rapp_package_blacklist: if no whitelist, blacklist these packages from the search *[[]]*
    :vartype rapp_package_blacklist: [ str ]
    :ivar screen: verbose rapp output to screen *[False]*
    :vartype screen: bool
    :ivar auto_rapp_installation: install dependencies on rapp start  *[False]*
    :vartype auto_rapp_installation: bool
    :ivar preferred: configure a dict of preferred child rapps when there are several choices *[{}]*
    :vartype preferred: {str}
    :ivar public_namespace: a hint for where rapps should lay down public connections *['/applications']*
    :vartype public_namespace: str

    Each element in the dict of preferred rapps should identify the preferred child rapp
    for each parent rapp specification. e.g. if the parent rapp is *rocon_apps/chirp* and there are
    child rapps *rocon_apps/moo*, *gopher_rapps/groot*, then *preferred = {'rocon_apps/chirp': 'gopher_rapps/groot'}*
    would ensure that groot is played each time the *rocon_apps/chirp* is requested. This is usually supplied to
    a ros launcher via yaml.

    The screen flag also checks for the ros parameter in */rocon/screen* to assist in providing verbose
    output when used in conjunction with `rocon_launch`_.

    .. _rocon_launch: http://wiki.ros.org/rocon_launch
    .. _rocon_uri: http://wiki.ros.org/rocon_uri
    .. _resource name: http://wiki.ros.org/Names#Package_Resource_Names
    """
    def __init__(self):
        # see sphinx docs above for more detailed explanations of each parameter
        self.robot_type = rospy.get_param('~robot_type', 'robot')
        self.robot_name = rospy.get_param('~robot_name', 'cybernetic_pirate')
        self.auto_start_rapp = rospy.get_param('~auto_start_rapp', None)
        self.rapp_package_whitelist = rospy.get_param('~rapp_package_whitelist', [])
        self.rapp_package_blacklist = rospy.get_param('~rapp_package_blacklist', [])
        rocon_screen = rospy.get_param('/rocon/screen', False)
        rapp_manager_screen = rospy.get_param('~screen', False)
        self.screen = rocon_screen or rapp_manager_screen
        self.auto_rapp_installation = rospy.get_param('~auto_rapp_installation', False)
        preferred = rospy.get_param('~preferred', [])
        self.application_namespace = rospy.get_param('~application_namespace', "/applications")
        # processing
        self.auto_start_rapp = self.auto_start_rapp if self.auto_start_rapp else None  # empty string -> None
        self.application_namespace = rosgraph.names.make_global_ns(self.application_namespace)
        # when pulled from yaml (see rocon_app_manager/param/preferred_defaults.yaml), it splices
        # the resource name for the key, e.g. 'preferred/rocon_apps/chirp'. As a result, preferred looks like:
        # {'rocon_apps': {'chirp': 'rocon_apps/moo_chirp', 'talker': 'rocon_apps/talker'}}
        self.preferred = {}
        for pkg, preferred_rapps_dict in preferred.iteritems():
            for parent_rapp_basename, preferred_rapp_resource_name in preferred_rapps_dict.iteritems():
                self.preferred[pkg + "/" + parent_rapp_basename] = preferred_rapp_resource_name

    def __str__(self):
        s = console.bold + "\nRapp Manager Standalone Parameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s


class ConcertParameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar concert_whitelist: used for `rocon_uri`_ rapp compatibility checks *['rapp_manager_script']*
    :vartype robot_type: str
    """
    def __init__(self):
        self.concert_whitelist = rospy.get_param('~concert_whitelist', [])
        # not yet implemented
        # self.local_concerts_only = rospy.get_param('~local_concerts_only', False)

    def __str__(self):
        s = console.bold + "\nRapp Manager Concert Parameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s
