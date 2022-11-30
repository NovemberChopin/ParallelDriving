
#ifndef parallel_driving_QNODE_HPP_
#define parallel_driving_QNODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_control/GetTopics.h"

#include <QThread>
#include <QDebug>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>


namespace parallel_driving {


struct ConfigInfo {
	QString rosMasterUri;	// master 节点地址
	QString localhost;		// 本机ip
	QString nodename;		// 本机节点名称
	std::vector<QString> imageTopics;
};


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	void setConfigInfo(ConfigInfo *config);
	void setImageTopic(QStringList *list, bool hasCompress);
	void setIsCompressImage(bool isCompress);
	void ImgCallback(const sensor_msgs::ImageConstPtr &msg, int cam_index);
	void ctrlCallback(const yhs_can_msgs::ctrl_fb& msg);

	void shutdownPubTopic();					// 停止发布话题
	void restorePubTopic(std::string prefix);	// 重新发布话题

	void shutdownSubTopic();					// 停止订阅话题
	void restoreSubTopic(std::string prefix);	// 重新订阅话题

	void shutdownService();						// 停止服务/客户端
	void restoreService(std::string prefix);	// 恢复服务/客户端

Q_SIGNALS:
    void rosShutdown();

	void getImage_0(cv::Mat img);
	void getImage_1(cv::Mat img);
	void getImage_2(cv::Mat img);
	void getImage_3(cv::Mat img);
	void getImage_4(cv::Mat img);

public Q_SLOTS:
	// 订阅压缩图像话题回调函数
	void imgCallback_C0(const sensor_msgs::CompressedImageConstPtr &msg);
	void imgCallback_C1(const sensor_msgs::CompressedImageConstPtr &msg);
	void imgCallback_C2(const sensor_msgs::CompressedImageConstPtr &msg);
	void imgCallback_C3(const sensor_msgs::CompressedImageConstPtr &msg);
	void imgCallback_C4(const sensor_msgs::CompressedImageConstPtr &msg);

	// 订阅正常图像话题回调函数
	void imgCallback_0(const sensor_msgs::ImageConstPtr &msg);
	void imgCallback_1(const sensor_msgs::ImageConstPtr &msg);
	void imgCallback_2(const sensor_msgs::ImageConstPtr &msg);
	void imgCallback_3(const sensor_msgs::ImageConstPtr &msg);
	void imgCallback_4(const sensor_msgs::ImageConstPtr &msg);


private:
	int init_argc;
	char** init_argv;
	
	ConfigInfo *configInfo;
	bool isCompressImage = true;			// 订阅的是否为压缩图像

	ros::Subscriber image_sub_c0;			// 订阅压缩图像话题
	ros::Subscriber image_sub_c1;
	ros::Subscriber image_sub_c2;
	ros::Subscriber image_sub_c3;
	ros::Subscriber image_sub_c4;

	image_transport::Subscriber image_sub0;	// 订阅非压缩图像话题
	image_transport::Subscriber image_sub1;
	image_transport::Subscriber image_sub2;
	image_transport::Subscriber image_sub3;
	image_transport::Subscriber image_sub4;
	

	yhs_can_msgs::ctrl_cmd ctrl_msg_;

	ros::Subscriber sub_ctrl_fb;

signals:
	void updateCtrlMsg(int gear, float velo, float steer);

public:
	ros::ServiceClient topic_client;
	ros::Publisher pub_ctrl_cmd;
	ros::Publisher pub_io_cmd;
	float velo_fb_ = 0;
	float steer_fb_ = 0;
	u_int8_t gear_fb_ = 1;	
};

}  // namespace parallel_driving

#endif /* parallel_driving_QNODE_HPP_ */
