#!/bin/sh
# $1 (provided by roslaunch) must be __name:=<node_name>

rosrun rosjava_bootstrap run.py rocon_app_teleop org.rocon.apps.teleop.servers.AudioPublisher $@
