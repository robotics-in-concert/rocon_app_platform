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

from rocon_app_utilities.rapp import *
from rocon_app_utilities.exceptions import *

##############################################################################
# Tests
##############################################################################

pwd = os.getcwd() 

def test_rapp_loading(): 
    print_title('Raising on Invalid Rapp Loading')

    console.pretty_println('Extra Field', console.bold)
    filename = pwd + '/test_rapps/rapp/invalid_loading/invalid_attribute.rapp'
    console.pretty_println(' - %s'% filename)
    assert_raises(InvalidRappException, load_yaml_from_file, filename)

    # console.pretty_println('Duplicated Field',console.bold)
    # TODO

def test_rapp_classification():
    print_title('Raising on Invalid Rapp Classification')

    console.pretty_println('Virtual Child', console.bold)
    verify_one(InvalidRappException,      classify_rapp_type, '/test_rapps/rapp/invalid_classification/virtual_child1.rapp')
    verify_one(InvalidRappException,      classify_rapp_type, '/test_rapps/rapp/invalid_classification/virtual_child2.rapp')

    console.pretty_println('Invalid Virtual Ancestor', console.bold)
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_virtual_ancestor1.rapp')

    console.pretty_println('Invalid Implementation Ancestor', console.bold)
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_implementation_ancestor1.rapp')
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_implementation_ancestor2.rapp')
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_implementation_ancestor3.rapp')
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_implementation_ancestor4.rapp')

    console.pretty_println('Invalid Implementation Child', console.bold)
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/invalid_implementation_child1.rapp')

    console.pretty_println('Field Conflict Rapp', console.bold)
    verify_one(InvalidRappFieldException, classify_rapp_type, '/test_rapps/rapp/invalid_classification/conflict.rapp')

def test_rapp_field_validation():
    print_title('Rapp Field Validation')

    console.pretty_println('Not Implemented', console.bold)
    verify_one(NotImplementedError, validate_rapp_field, '/test_rapps/rapp/valid/implementation_ancestor.rapp')

def test_rapp_inheritance():
    def inherit_pair(path):
        filename = pwd + path + '/child.rapp'
        child = Rapp('child',filename) 
        filename = pwd + path + '/parent.rapp'
        parent = Rapp('parent',filename) 

        child.inherit(parent)
        return child

    def validate(data, valid_data): 
        for f, d in valid_data:
            if not f in data: 
                return False
            if not data[f] == d:
                return False

        return True

    print_title('Rapp Inheritance')

    # full inheritance
    path = '/test_rapps/rapp/inherity/full'
    console.pretty_println(' - %s'%path) 
    child = inherit_pair(path)

    d = [('display',           'Talker'),
         ('description',       'Default ros style talker tutorial'),
         ('public_interface',  'rocon_apps/talker.interface'),
         ('public_parameters', 'rocon_apps/talker.parameters'),
         ('icon',              'rocon_apps/talker.png')]
    assert_true(validate(child.data, d))
    

    # icon and publics
    path = '/test_rapps/rapp/inherity/icon_and_publics'
    console.pretty_println(' - %s'%path) 
    child = inherit_pair(path)
    d = [('display',           'Child Talker in icon and publics'),
         ('description',       'Hola Child...........'),
         ('public_interface',  'rocon_apps/talker.interface'),
         ('public_parameters', 'rocon_apps/talker.parameters'),
         ('icon',              'rocon_apps/talker.png')]
    assert_true(validate(child.data, d))

    # publics
    path = '/test_rapps/rapp/inherity/publics'
    console.pretty_println(' - %s'%path) 
    child = inherit_pair(path)
    d = [('display',           'Child Talker in publics'),
         ('description',       'public public child'),
         ('public_interface',  'rocon_apps/talker.interface'),
         ('public_parameters', 'rocon_apps/talker.parameters')]
    assert_true(validate(child.data, d))

    # from meta
    path ='/test_rapps/rapp/inherity/from_meta'
    console.pretty_println(' - %s'%path) 
    child = inherit_pair(path)
    d = [('display',           'Talker in from meta'),
         ('description',       'from meta meta meta metaaaaaaaaa'),
         ('public_interface',  'rocon_apps/talker.interface'),
         ('public_parameters', 'rocon_apps/talker.parameters'),
         ('icon',              'rocon_apps/talker.png')]
    assert_true(validate(child.data, d))


    # from another child
    path = '/test_rapps/rapp/inherity/from_child'
    console.pretty_println(' - %s'%path) 
    child = inherit_pair(path)
    d = [('display',           'Talker'),
         ('description',       'Default ros style talker tutorial'),
         ('icon',              'rocon_apps/talker.png')]
    assert_true(validate(child.data, d))

def print_title(title):
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* " + str(title) + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)

def verify_one(Except, func, filename):
    f = pwd + filename
    data = load_yaml_from_file(f)
    console.pretty_println(' - %s'%f)
    assert_raises(Except, func, data)
