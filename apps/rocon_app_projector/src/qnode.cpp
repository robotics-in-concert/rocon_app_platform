/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <sstream>
#include "../include/rocon_projector/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_projector {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"rocon_projector");
  if ( ! ros::master::check() )
    return false;
  ros::start(); // explicitly needed since our node handle is going out of scope.
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Add your ros communications here.
  sub_ = nh.subscribe("change_background", 10, &QNode::subscriberCallback, this);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"rocon_projector");
  if ( ! ros::master::check() )
    return false;
  ros::start(); // explicitly needed since our node handle is going out of scope.
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Add your ros communications here.
  sub_ = nh.subscribe("change_background", 10, &QNode::subscriberCallback, this);
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::subscriberCallback(const std_msgs::Int8::ConstPtr& input)
{
  if ( input->data == 1 )
  {
    Q_EMIT changeBackgroundTrigger(input->data);
    ROS_INFO_STREAM("Subscriber: background change to next background triggered.");
  }
  else if ( input->data == -1 )
  {
    Q_EMIT changeBackgroundTrigger(input->data);
    ROS_INFO_STREAM("Subscriber: background change to last background triggered.");
  }
  else
    ROS_WARN_STREAM("Subscriber: invalid messages sent.");
}


}  // namespace rocon_projector
