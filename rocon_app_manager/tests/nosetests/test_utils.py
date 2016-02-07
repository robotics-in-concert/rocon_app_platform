##############################################################################
# Imports
##############################################################################

from __future__ import absolute_import

import sys
import os

from nose.tools import assert_raises
from rocon_app_manager import utils
import rocon_console.console as console

##############################################################################
# Support Methods
##############################################################################

def default_launch_arg_mappings():
    launch_arg_mappings = utils.LaunchArgMappings()
    launch_arg_mappings.application_namespace = "/applications"
    launch_arg_mappings.robot_name = "dude"
    launch_arg_mappings.rocon_uri = "rocon:/"
    launch_arg_mappings.capability_nodelet_manager_name = None
    return launch_arg_mappings

##############################################################################
# Tests
##############################################################################


def test_prepare_launch_text_launchfile():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* No Bells and Whistles" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", [], {}, default_launch_arg_mappings())
    print launchtext
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher" ns="/">
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Application Namespace" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", ['application_namespace'], {}, default_launch_arg_mappings())
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher" ns="/">
    <arg name="application_namespace" value="/applications"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_public_parameters():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Public Parameters" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", [], {"public_param": "pubparam_value"}, default_launch_arg_mappings())
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher" ns="/">
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_rocon_uri():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Rocon Uri" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", ['rocon_uri'], {}, default_launch_arg_mappings())
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher" ns="/">
    <arg name="rocon_uri" value="rocon:/"/>
  </include>
</launch>
""".strip()

