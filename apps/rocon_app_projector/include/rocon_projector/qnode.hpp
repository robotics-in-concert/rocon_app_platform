/**
 * @file /include/rocon_projector/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rocon_projector_QNODE_HPP_
#define rocon_projector_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <std_msgs/Int8.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_projector {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel
	{
    Debug,
    Info,
    Warn,
    Error,
    Fatal
	 };

Q_SIGNALS:
  void rosShutdown();
  void changeBackgroundTrigger(int trigger);

private:
	int init_argc;
	char** init_argv;
	std::string name_; // name of the ros node

	ros::Subscriber sub_;
  /*
   * Subscriber callback function for reading topic messages and setting the trigger accordingly
   */
  void subscriberCallback(const std_msgs::Int8::ConstPtr& input);
};

}  // namespace rocon_projector

#endif /* rocon_projector_QNODE_HPP_ */
