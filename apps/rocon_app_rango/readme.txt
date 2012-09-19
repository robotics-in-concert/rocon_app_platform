##############################################################################
# Overview
##############################################################################

Simple app for the rocon guide demo.

It needs robosem (2011)'s manipulation under the hood to work.

##############################################################################
# App Comms
##############################################################################
 
== Global Publishers ==

 - finished [std_msgs/Empty] : emitted when the lesson ends.
   
== Global Subscribers ==

 - start_lesson [std_msgs/Empty] : used to start a lesson
  
== Local Publishers ==

 - enable [std_msgs/String]
 - disable [std_msgs/String]
 
== Local Subscribers ==
  
 - ~touch_sensors [sensor_integration_board_comms/OnOffDevice]
 
== Local Service Calls ==

 - ~ace_task [robosem_comms/AceTask]

== Local Action Clients ==

 = ~joint_trajectory [manipulation_comms/JointTrajectoryAction]

 