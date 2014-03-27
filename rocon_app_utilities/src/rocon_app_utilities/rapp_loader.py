#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################
from .exceptions import InvalidRappException
import yaml

def load_rapp_yaml_from_file(filename):
    '''
      Load rapp specs yaml from the given file

      :param filename: absolute path to the rapp definition
      :type filename: str

      :returns: dict of loaded rapp
      :rtype: dict

      :raises: InvalidRappFieldException: Rapp includes invalid filed
    '''
    RAPP_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capability']

    with open(filename, 'r') as f:
        app_data = yaml.load(f.read())

        for d in app_data:
            if d not in RAPP_ATTRIBUTES:
                raise InvalidRappException('Invalid Field : [' + str(d) + '] Valid Fields : [' + str(RAPP_ATTRIBUTES) + ']')

    return app_data
