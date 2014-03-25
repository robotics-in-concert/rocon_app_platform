#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
import yaml
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
from .utils import dict_to_KeyValue

##############################################################################
# Class
##############################################################################
class PairingClient(object):
    '''
      A pairing client runs an app which is one which will work in tandem with
      this robot (rocon) app. The client is usually a smart phone or tablet.
    '''
    __slots__ = ['client_type', 'manager_data', 'app_data']

    def __init__(self, client_type, manager_data, app_data):
        self.client_type = client_type
        self.manager_data = manager_data
        self.app_data = app_data

    def as_dict(self):
        return {'client_type': self.client_type, 'manager_data': self.manager_data, 'app_data': self.app_data}

    def __eq__(self, other):
        if not isinstance(other, PairingClient):
            return False
        return self.client_type == other.client_type and \
               self.manager_data == other.manager_data and \
               self.app_data == other.app_data

    def __repr__(self):
        return yaml.dump(self.as_dict())

    def to_msg(self):
        msg = rocon_app_manager_msgs.PairingClient()
        msg.client_type  = self.client_type
        msg.manager_data = dict_to_KeyValue(self.manager_data)
        msg.app_data     = dict_to_KeyValue(self.app_data)
        return msg
