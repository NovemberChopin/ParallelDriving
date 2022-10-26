
#ifndef parallel_driving_QNODE_HPP_
#define parallel_driving_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include "sensor_msgs/image_encodings.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"

#include <QThread>
#include <QDebug>
#include <string>
#include <thread>
#include <opencv2/dnn.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>


namespace parallel_driving {


struct ConfigInfo {
	QString rosMasterUri;
	QString localhost;
	std::vector<QString> imageTopics;
};


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	void setConfigInfo(ConfigInfo *config);
	void ImgCallback(const sensor_msgs::ImageConstPtr &msg, int cam_index);
	void ctrlCallback(const yhs_can_msgs::ctrl_fb& msg);
	
	
	void sendCtrlCmd();


Q_SIGNALS:
    void rosShutdown();

	void getImage_0(cv::Mat img);
	void getImage_1(cv::Mat img);
	void getImage_2(cv::Mat img);
	void getImage_3(cv::Mat img);

private:
	int init_argc;
	char** init_argv;
	
	ConfigInfo *configInfo;

	image_transport::Subscriber image_sub1;
	image_transport::Subscriber image_sub2;
	image_transport::Subscriber image_sub3;
	image_transport::Subscriber image_sub4;

	yhs_can_msgs::ctrl_cmd ctrl_msg_;

	ros::Subscriber sub_ctrl_fb;

public:
	ros::Publisher pub_ctrl_cmd;
	ros::Publisher pub_io_cmd;
	float velo_fb_ = 0;
	float steer_fb_ = 0;
};

}  // namespace parallel_driving

#endif /* parallel_driving_QNODE_HPP_ */
