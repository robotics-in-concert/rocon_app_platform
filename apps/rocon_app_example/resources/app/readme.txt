
Concert : roslaunch concert_master concert.launch
Robot   : roslaunch museum app_manager.launch
Concert : rosservice call /ros_robot_0/start_app guide_demo/example
Concert : rostopic echo /ros_robot_0/application/result
Concert : rostopic pub /ros_robot_0/application/float std_msgs/Float32 1.3 -1

You can also publish to the local topic too if you feel like it (ros_robot_0/application/int).

===== .app File =====

Required keys

 - platform: 
 - launch: launcher for the application
 - interface: global publishers and subscribers
 
Optional keys:

 - display: textual display name
 - description: long textual description
 - icon: resource name for an icon file (e.g. resources/app/example.png)

===== Errata =====

 - 'resource name' : package/resource pair separated by the '/', e.g. std_msgs/String
 