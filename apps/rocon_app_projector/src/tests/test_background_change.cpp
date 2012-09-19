/*
 * test_background_change.cpp
 *
 *  Created on: Nov 16, 2011
 *      Author: marcus
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rocon_projector_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int8>("/rocon_projector/change_background", 10);
  std_msgs::Int8 trigger_msg;
  int trigger = 0;

  while (ros::ok())
  {
    switch (trigger)
    {
      case 0:
        trigger_msg.data = -1;
        ROS_INFO_STREAM("rocon_projector_test_node: sending request for changing to last background.");
        break;
      case 1:
        trigger_msg.data = -1;
        ROS_INFO_STREAM("rocon_projector_test_node: sending request for changing to last background.");
        break;
      case 2:
        trigger_msg.data = 1;
        ROS_INFO_STREAM("rocon_projector_test_node: sending request for changing to next background.");
        break;
      case 3:
        trigger_msg.data = 1;
        trigger = 0;
        ROS_INFO_STREAM("rocon_projector_test_node: sending request for changing to next background.");
        break;
      default:
        trigger = 0;
    }
    pub.publish(trigger_msg);
    trigger++;
    ros::Duration(3.0).sleep();
  }
  return 0;
}


