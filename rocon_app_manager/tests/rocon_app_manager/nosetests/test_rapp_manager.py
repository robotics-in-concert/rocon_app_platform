from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rocon_app_manager.rapp_manager import RappManager

def test_msg_build():
    # TODO
    raise

class TestRappManager(object):
    def setUp(self):
        self.rapp_managerInstance = RappManager()

    def tearDown(self):
        pass

    def test_msg_build(self):
        # TODO
        raise
