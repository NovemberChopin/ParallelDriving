/**
 * @file /include/ros_gui_pub/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_gui_pub_QNODE_HPP_
#define ros_gui_pub_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include "yhs_can_msgs/ctrl_cmd.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_gui_pub {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	QStringListModel* loggingModel_sub() { return &logging_model_sub; }
	void log(const LogLevel &level, const std::string &msg);
	void log_sub(const LogLevel &level_sub, const std::string &msg_sub);

	void Callback(const std_msgs::StringConstPtr& submsg);

Q_SIGNALS:
	void loggingUpdated();
	void loggingUpdated_sub();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Subscriber chatter_subscriber;
    QStringListModel logging_model;
	QStringListModel logging_model_sub;

	ros::Publisher pub_ctrl_cmd;
	ros::Subscriber sub_ctrl_cmd;
	yhs_can_msgs::ctrl_cmd ctrl_msg_;
};

}  // namespace ros_gui_pub

#endif /* ros_gui_pub_QNODE_HPP_ */
