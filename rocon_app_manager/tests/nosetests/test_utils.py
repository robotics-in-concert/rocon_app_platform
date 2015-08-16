from __future__ import absolute_import

import sys
import os

from nose.tools import assert_raises
from rocon_app_manager import utils
import rocon_console.console as console

def test_prepare_launch_text_launchfile():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* No Bells and Whistles" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", [], {}, "", "rocon:/", False)
    print launchtext
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher">
  </include>
</launch>
""".strip()
 
def test_prepare_launch_text_launchfile_appnamespace():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Application Namespace" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", ['application_namespace'], {}, "applications", "rocon:/", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher">
    <arg name="application_namespace" value="applications"/>
  </include>
</launch>
""".strip()
  
def test_prepare_launch_text_launchfile_public_parameters():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Public Parameters" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", [], {"public_param": "pubparam_value"}, "", "rocon:/", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher">
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
    launchtext = utils._prepare_launch_text("path_to_launcher", ['rocon_uri'], {}, "", "rocon:/", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher">
    <arg name="rocon_uri" value="rocon:/"/>
  </include>
</launch>
""".strip()
  
def test_prepare_launch_text_launchfile_simulation():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Simulation" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    launchtext = utils._prepare_launch_text("path_to_launcher", ['simulation'], {}, "", "rocon:/", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="path_to_launcher">
    <arg name="simulation" value="False"/>
  </include>
</launch>
""".strip()
  
