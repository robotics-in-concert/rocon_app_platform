#!/usr/bin/env python

""" Testing the find_node function """

##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import rocon_app_manager
import rospy
import rostest
import unittest
import tempfile

##############################################################################
# Test Class
##############################################################################


class TestRemappings(unittest.TestCase):
    def test_apply_remapping_rules(self):
        print("")
        print(console.bold + "\n****************************************************************************************" + console.reset)
        print(console.bold + "* Remappings" + console.reset)
        print(console.bold + "****************************************************************************************" + console.reset)
        print("")
        data={}

        launcher_to_include = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        launcher_text = """
<launch>
  <arg name="application_namespace" default="applications"/>
  <arg name="rocon_uri" default="rocon:/"/>
</launch>
""".strip()
        launcher_to_include.write(launcher_text)
        launcher_to_include.close()

        data['launch'] = launcher_to_include.name
        data['launch_args'] = ['rocon_uri', 'application_namespace']

        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        application_namespace = "applications"
        rocon_uri = "rocon:/"
        force_screen = True

        self_launch = rocon_app_manager.utils.prepare_launcher(data, {}, force_screen, rocon_app_manager.utils.LaunchArgMappings(), temp)

        pubif_data = {'public_interface': {
            'services': [],
            'publishers': [
                {'type': 'std_msgs/String', 'name': 'homebase'},
                {'type': 'geometry_msgs/PoseStamped', 'name': 'pose'},
                {'type': 'gopher_std_msgs/Status', 'name': 'robot_status'}
            ],
            'action_clients': [],
            'subscribers': [],
            'action_servers': [
                {'type': 'gopher_std_msgs/DeliveryTask', 'name': 'delivery'}
            ]}
        }

        remappings=[]

        connections, published_interfaces = rocon_app_manager.utils.apply_remapping_rules_from_start_app_request(self_launch, pubif_data, remappings, application_namespace)

        print("Connections:")
        for key, value in connections.iteritems():
            print("  %s: %s" % (key, value))
        print("Published Interfaces")
        for interface in published_interfaces:  # [rocon_app_manager_msgs.PublishedInterfaces]
            print("  - name: %s" % interface.interface.name)
            print("    connection_type: %s" % interface.interface.connection_type)
            print("    data_type: %s" % interface.interface.data_type)
            print("    remapped name: %s" % interface.name)
        assert(len(published_interfaces) == 4)
        for topic in ['homebase', 'pose', 'robot_status']:
            assert('/' + application_namespace + '/' + topic in connections['publishers'])

if __name__ == '__main__':
    rospy.init_node("test_remappigns")
    rostest.rosrun('rocon_app_manager',
                   'test_remappings',
                   TestRemappings)
