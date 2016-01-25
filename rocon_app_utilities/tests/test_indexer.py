#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises, assert_true
import os
import rocon_console.console as console

import rocon_uri
from rocon_uri.exceptions import RoconURIValueError
from rocon_app_utilities import *
from rocon_app_utilities import *
from rocon_app_utilities.exceptions import *

default_data = [('basic/child',             '/test_rapps/indexer/basic/child.rapp'),
                ('basic/parent',            '/test_rapps/indexer/basic/parent.rapp'),
                ('chained/child',           '/test_rapps/indexer/chained/child.rapp'),
                ('chained/parent',          '/test_rapps/indexer/chained/parent.rapp'),
                ('chained/ancestor',        '/test_rapps/indexer/chained/ancestor.rapp'),
                ('multi_children/child1',   '/test_rapps/indexer/multi_children/child1.rapp'),
                ('multi_children/child2',   '/test_rapps/indexer/multi_children/child2.rapp'),
                ('multi_children/parent',   '/test_rapps/indexer/multi_children/parent.rapp'),
                ('cyclic/child',            '/test_rapps/indexer/cyclic/child.rapp'),
                ('cyclic/parent',           '/test_rapps/indexer/cyclic/parent.rapp'),
                ('invalid/invalid_child',   '/test_rapps/indexer/invalid/invalid_child.rapp')
               ]

valid_data = [('basic/child',             '/test_rapps/indexer/basic/child.rapp'),
              ('basic/parent',            '/test_rapps/indexer/basic/parent.rapp'),
             ]

multi_children_data = [('multi_children/child1',   '/test_rapps/indexer/multi_children/child1.rapp'),
                       ('multi_children/child2',   '/test_rapps/indexer/multi_children/child2.rapp'),
                       ('multi_children/parent',   '/test_rapps/indexer/multi_children/parent.rapp')
                      ]

cyclic_data = [('cyclic/child',            '/test_rapps/indexer/cyclic/child.rapp'),
               ('cyclic/parent',           '/test_rapps/indexer/cyclic/parent.rapp')
              ]

compat_data = [('compat/turtlebot_teleop',  '/test_rapps/indexer/compat/turtlebot_teleop.rapp'),
               ('compat/kobuki_teleop',     '/test_rapps/indexer/compat/kobuki_teleop.rapp'),
               ('compat/teleop',            '/test_rapps/indexer/compat/teleop.rapp')
              ]


##############################################################################
# Tests
##############################################################################

class TestRappIndexer():

    @classmethod
    def setup_class(cls):
        load_data(default_data, verbose=True)

    def setup(self,data=default_data):
        self.data = load_data(data)
        self.indexer = RappIndexer(self.data)

    def teardown(self):
        del self.data

    def test_update_index(self):
        # How could this be tested...
        pass

    def test_get_raw_rapp(self):
        print_title('Test Get Raw Rapp')

        # call not exist rapp
        print(assert_raises(RappNotExistException,self.indexer.get_raw_rapp,'noexist/rapp'))

        # Correct call
        rapp = self.indexer.get_raw_rapp('basic/child')
        print(str(rapp))
        assert_true(rapp.raw_data['compatibility'] == 'rocon:/*')
        assert_true(rapp.raw_data['launch'] == os.path.join(os.getcwd(), "test_rapps", "indexer", "basic", 'child.launch'))
        assert_true(rapp.raw_data['parent_name'] == 'basic/parent')


    def test_get_rapp(self):
        def compare(a,b,field):
            return a.raw_data[field] == b.raw_data[field]
 
        print_title('Test Get Rapp')
        console.pretty_println('TODO')

        # Basic
        """
        These are not testable at the moment since test case only includes file pointers which does not exist...
 
        console.pretty_println('Basic', console.bold)
        inherited_rapp = self.indexer.get_rapp('basic/child')
        parent_rapp = self.indexer.get_raw_rapp('basic/parent')
        child_rapp = self.indexer.get_raw_rapp('basic/child')
        assert_true(inherited_rapp.type == 'Implementation Ancestor')
        assert_true(compare(inherited_rapp, parent_rapp, 'display'))
        assert_true(compare(inherited_rapp, parent_rapp, 'description'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_parameters'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_interface'))
        assert_true(compare(inherited_rapp, child_rapp,  'launch'))
 
        # Chained Child -> Parent -> Ancestor
        console.pretty_println('Chained', console.bold)
        inherited_rapp = self.indexer.get_rapp('chained/child')
        ancestor_rapp = self.indexer.get_raw_rapp('chained/ancestor')
        parent_rapp = self.indexer.get_raw_rapp('chained/parent')
        child_rapp = self.indexer.get_raw_rapp('chained/child')
        assert_true(inherited_rapp.type == 'Implementation Ancestor')
        assert_true(compare(inherited_rapp, child_rapp, 'launch'))
        assert_true(compare(inherited_rapp, parent_rapp, 'description'))
        assert_true(compare(inherited_rapp, parent_rapp, 'display'))
        assert_true(compare(inherited_rapp, ancestor_rapp, 'public_interface'))
        assert_true(compare(inherited_rapp, ancestor_rapp, 'public_parameters'))
 
        # Multiple Child
        console.pretty_println('Multiple Child', console.bold)
        inherited_rapp = self.indexer.get_rapp('multi_children/child1')
        child_rapp = self.indexer.get_raw_rapp('multi_children/child1')
        parent_rapp = self.indexer.get_raw_rapp('multi_children/parent')
        assert_true(inherited_rapp.type == 'Implementation Ancestor')
        assert_true(compare(inherited_rapp, parent_rapp, 'description'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_parameters'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_interface'))
        assert_true(compare(inherited_rapp, child_rapp, 'display'))
        assert_true(compare(inherited_rapp, child_rapp,  'launch'))
 
        inherited_rapp = self.indexer.get_rapp('multi_children/child2')
        child_rapp = self.indexer.get_raw_rapp('multi_children/child2')
        assert_true(inherited_rapp.type == 'Implementation Ancestor')
        assert_true(compare(inherited_rapp, parent_rapp, 'description'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_parameters'))
        assert_true(compare(inherited_rapp, parent_rapp, 'public_interface'))
        assert_true(compare(inherited_rapp, child_rapp, 'display'))
        assert_true(compare(inherited_rapp, child_rapp,  'launch'))
 
        # Cyclic
        console.pretty_println('Cyclic', console.bold)
        assert_raises(RappCyclicChainException, self.indexer.get_rapp, 'cyclic/child')
        """
 
    def test_get_compatible_rapps(self):
        print_title('Test Get Compatible Rapps')
 
        self.setup(compat_data)
        # string rocon uri test
        console.pretty_println('String Rocon URI Given')
        compat = 'rocon:/kobuki'
        compatible_rapps, incompatible_rapps, invalid_rapps = self.indexer.get_compatible_rapps(compat)
        print(str(compatible_rapps))
        print(str(incompatible_rapps))
        print(str(invalid_rapps))
 
        for r in compatible_rapps:
            print(r)
 
        for r in incompatible_rapps:
            print(r)
 
        for r in invalid_rapps:
            print(r + " : " + invalid_rapps[r])
        assert_true(len(compatible_rapps.keys()) == 1 and len(incompatible_rapps.keys()) == 1)
 
        incompat = [r for r in compatible_rapps.values() if not r.is_compatible(compat)]
        compat = [r for r in incompatible_rapps.values() if r.is_compatible(compat)]
        assert_true(len(incompat) == 0)
        assert_true(len(compat) == 0)
         
    def test_to_dot(self):
        print_title('Test To Dot')
         
        console.pretty_println('Not Implemented')
        assert_raises(NotImplementedError,self.indexer.to_dot)


def print_title(title):
    print(console.bold + "\n******************************************************" + console.reset)
    print(console.bold + "* " + str(title) + console.reset)
    print(console.bold + "******************************************************" + console.reset)


def load_data(data, verbose=False):
    if verbose:
        console.pretty_println('Loading Test Rapps..',console.bold)
    pwd = os.getcwd() 

    loaded = {}
    for name, path in data:
        loaded[name] = Rapp(name, filename=str(pwd + path))

    if verbose:
        for n in loaded:
            console.pretty_println(' - %s'% n)
    return loaded 
