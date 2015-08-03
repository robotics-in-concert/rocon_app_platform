from __future__ import absolute_import

import sys
import os
import tempfile
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rocon_app_manager import utils

def test_prepare_launch_text_launchfile():
    launchtext = utils._prepare_launch_text("launch_file", [], {}, "", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace():
    launchtext = utils._prepare_launch_text("launch_file", [], {}, "application_namespace", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include ns="application_namespace" file="launch_file">
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace_pubparam():
    launchtext = utils._prepare_launch_text("launch_file", [], {"public_param": "pubparam_value"}, "", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace_pubparam_roconuri():
    launchtext = utils._prepare_launch_text("launch_file", ['rocon_uri'], {"public_param": "pubparam_value"}, "", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
    <arg name="rocon_uri" value="rocon_uri_string"/>
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace_pubparam_roconuri_gateway():
    launchtext = utils._prepare_launch_text("launch_file", ['rocon_uri', 'gateway_name'], {"public_param": "pubparam_value"}, "", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
    <arg name="rocon_uri" value="rocon_uri_string"/>
    <arg name="gateway_name" value="gateway_name_string"/>
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace_pubparam_roconuri_gateway_simulation():
    launchtext = utils._prepare_launch_text("launch_file", ['rocon_uri', 'gateway_name', 'simulation'], {"public_param": "pubparam_value"}, "", "gateway_name_string", "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
    <arg name="rocon_uri" value="rocon_uri_string"/>
    <arg name="gateway_name" value="gateway_name_string"/>
    <arg name="simulation" value="False"/>
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()

def test_prepare_launch_text_launchfile_appnamespace_pubparam_roconuri_NOgateway_simulation():
    launchtext = utils._prepare_launch_text("launch_file", ['rocon_uri', 'gateway_name', 'simulation'], {"public_param": "pubparam_value"}, "", None, "rocon_uri_string", False)
    print "'" + launchtext + "'"
    assert launchtext.strip() == """
<launch>
  <include file="launch_file">
    <arg name="rocon_uri" value="rocon_uri_string"/>
    <arg name="gateway_name" value=""/>
    <arg name="simulation" value="False"/>
    <arg name="public_param" value="pubparam_value"/>
  </include>
</launch>
""".strip()


    # TODO : unit tests for prepare_launcher

    # TODO : unit tests for apply_requested_public_parameters

#TODO : this should be a ros test because we depend on param server
def test_apply_remapping_rules_from_start_app_request():
    data={}
    data['launch'] = "launch_file"
    data['launch_args'] = ['rocon_uri', 'gateway_name', 'simulation']

    temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
    force_screen = True
    simulation = False
    application_namespace = "app_namespace"

    self_launch = utils.prepare_launcher(data, {}, "app_namespace", "gateway_name", "rocon_uri_string", None, force_screen, simulation, temp)

    pubif_data = {'public_interface': {
        'services': [],
        'publishers': [
            {'type': 'std_msgs/String', 'name': 'homebase'},
            {'type': 'geometry_msgs/PoseStamped', 'name': 'pose'},
            {'type': 'gopher_std_msgs/Status', 'name': 'robot_status'}
        ], 'action_clients': [],
        'subscribers': [],
        'action_servers': [
            {'type': 'gopher_std_msgs/DeliveryTask', 'name': 'delivery'}
        ]}
    }

    remappings=[]

    connections, published_interfaces = utils.apply_remapping_rules_from_start_app_request(self_launch, pubif_data, remappings, application_namespace)

    print connections
    print published_interfaces