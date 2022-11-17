
#include "../include/parallel_driving/qnode.hpp"

using namespace cv;
using namespace dnn;

namespace parallel_driving {


QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
	u_int8_t gear = 4;
	u_int8_t brake = 0;
	float velocity = 2.0;
	float steer = 20.0;
	this->ctrl_msg_.ctrl_cmd_gear = gear;
	this->ctrl_msg_.ctrl_cmd_velocity = velocity;
	this->ctrl_msg_.ctrl_cmd_steering = steer;
	this->ctrl_msg_.ctrl_cmd_Brake = brake;
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


bool QNode::init() {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = this->configInfo->rosMasterUri.toStdString();
	remappings["__hostname"] = this->configInfo->localhost.toStdString();
	ros::init(remappings, this->configInfo->nodename.toStdString());
	if (!ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	this->start();				// 启动线程
	return true;
}


void QNode::shutdownPubTopic() {
	this->pub_ctrl_cmd.shutdown();
	this->pub_io_cmd.shutdown();
	std::cout << "shutdown pub topic" << std::endl;
}

void QNode::restorePubTopic(std::string prefix) {
	ros::NodeHandle node;
	this->pub_ctrl_cmd = node.advertise<yhs_can_msgs::ctrl_cmd>(prefix+"ctrl_cmd", 5);
	this->pub_io_cmd = node.advertise<yhs_can_msgs::io_cmd>(prefix+"io_cmd", 5);
	std::cout << "restore pub topic: " << std::endl;
	ros::V_string topics;
	ros::this_node::getAdvertisedTopics(topics);
	for (auto t: topics) {
		std::cout << t << std::endl;
	}
}


void QNode::shutdownSubTopic() {
	this->sub_ctrl_fb.shutdown();
	if (isCompressImage) {
		this->image_sub_c0.shutdown();
		this->image_sub_c1.shutdown();
		this->image_sub_c2.shutdown();
		this->image_sub_c3.shutdown();
		this->image_sub_c3.shutdown();
	} else {
		this->image_sub0.shutdown();
		this->image_sub1.shutdown();
		this->image_sub2.shutdown();
		this->image_sub3.shutdown();
		this->image_sub4.shutdown();
	}
	std::cout << "shutdown sub topic" << std::endl;
}


void QNode::restoreSubTopic(std::string prefix) {
	ros::NodeHandle node;
	image_transport::ImageTransport it(node);
	this->sub_ctrl_fb = node.subscribe(prefix+"ctrl_fb", 5, &QNode::ctrlCallback, this);
	std::cout << "------- 要订阅的话题 -------" << std::endl;
	for (auto item: configInfo->imageTopics) {
		std::cout << item.toStdString() << std::endl;
	}
	if (isCompressImage) {		// 订阅压缩图像
		image_sub_c0 = node.subscribe(configInfo->imageTopics[0].toStdString(), 1, &QNode::imgCallback_C0, this);
		image_sub_c1 = node.subscribe(configInfo->imageTopics[1].toStdString(), 1, &QNode::imgCallback_C1, this);
		image_sub_c2 = node.subscribe(configInfo->imageTopics[2].toStdString(), 1, &QNode::imgCallback_C2, this);
		image_sub_c3 = node.subscribe(configInfo->imageTopics[3].toStdString(), 1, &QNode::imgCallback_C3, this);
		image_sub_c4 = node.subscribe(configInfo->imageTopics[4].toStdString(), 1, &QNode::imgCallback_C4, this);
	} else {
		image_sub0 = it.subscribe(configInfo->imageTopics[0].toStdString(), 1, &QNode::imgCallback_0, this);
		image_sub1 = it.subscribe(configInfo->imageTopics[1].toStdString(), 1, &QNode::imgCallback_1, this);
		image_sub2 = it.subscribe(configInfo->imageTopics[2].toStdString(), 1, &QNode::imgCallback_2, this);
		image_sub3 = it.subscribe(configInfo->imageTopics[3].toStdString(), 1, &QNode::imgCallback_3, this);
		image_sub4 = it.subscribe(configInfo->imageTopics[4].toStdString(), 1, &QNode::imgCallback_4, this);
	}
	std::cout << "restore sub topic: " << std::endl;
	ros::V_string sub_topics;
	ros::this_node::getSubscribedTopics(sub_topics);
	for (auto t: sub_topics) {
		std::cout << t << std::endl;
	}
}


void QNode::shutdownService() {
	this->topic_client.shutdown();
}


void QNode::restoreService(std::string prefix) {
	ros::NodeHandle node;
	this->topic_client = node.serviceClient<yhs_can_control::GetTopics>(prefix+"get_topics");
}


void QNode::setIsCompressImage(bool isCompress) {
	this->isCompressImage = isCompress;
}


void QNode::setConfigInfo(ConfigInfo *config) {
	qDebug() << "set config info";
	this->configInfo = config;
}

void QNode::setImageTopic(QStringList *list, bool hasCompress) {
	this->configInfo->imageTopics.clear();		// 先清空话题
	this->isCompressImage = hasCompress;		// 当前话题是否是压缩图像
	for (int i=0; i<list->size(); i++) {
		this->configInfo->imageTopics.push_back(list->at(i));
	}
	// qDebug() << this->configInfo->imageTopics;
}


/**
 * @brief 订阅小车的速度控制反馈话题回调函数
 * 
 * @param msg 
 */
void QNode::ctrlCallback(const yhs_can_msgs::ctrl_fb& msg) {
	// std::cout << "velocity fb is " << msg.ctrl_fb_velocity << " " << msg.ctrl_fb_steering << std::endl;
	this->velo_fb_ = msg.ctrl_fb_velocity;
	this->steer_fb_ = msg.ctrl_fb_steering;
	this->gear_fb_ = msg.ctrl_fb_gear;
	// 在这里更新速度信息
	int gear = msg.ctrl_fb_gear;
	Q_EMIT updateCtrlMsg(gear, this->velo_fb_, this->steer_fb_);
}


void QNode::run() {

	ros::spin();
	
	
	// std::string topic;
	// ros::NodeHandle n_a;
	// ros::CallbackQueue callback_queue_a;
	// n_a.setCallbackQueue(&callback_queue_a);
	// image_transport::ImageTransport it(n_a);
	// // 0 表示 Callback 的第二个参数 cam_index
	// topic = this->configInfo->imageTopics.at(0).toStdString();
	// image_sub1 = it.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 0));
	// std::thread spinner_thread_a([&callback_queue_a](){
	// 	ros::SingleThreadedSpinner spinner_a;
	// 	spinner_a.spin(&callback_queue_a);
	// });
	
	// ros::NodeHandle n_b;
	// ros::CallbackQueue callback_queue_b;
	// n_b.setCallbackQueue(&callback_queue_b);
	// image_transport::ImageTransport it_b(n_b);
	// topic = this->configInfo->imageTopics.at(1).toStdString();
	// image_sub2 = it_b.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 1));
	// std::thread spinner_thread_b([&callback_queue_b](){
	// 	ros::SingleThreadedSpinner spinner_b;
	// 	spinner_b.spin(&callback_queue_b);
	// });

	// ros::NodeHandle n_c;
	// ros::CallbackQueue callback_queue_c;
	// n_c.setCallbackQueue(&callback_queue_c);
	// image_transport::ImageTransport it_c(n_c);
	// topic = this->configInfo->imageTopics.at(2).toStdString();
	// image_sub3 = it_c.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 2));
	// std::thread spinner_thread_c([&callback_queue_c](){
	// 	ros::SingleThreadedSpinner spinner_c;
	// 	spinner_c.spin(&callback_queue_c);
	// });
	
	// ros::NodeHandle n_d;
	// ros::CallbackQueue callback_queue_d;
	// n_d.setCallbackQueue(&callback_queue_d);
	// image_transport::ImageTransport it_d(n_d);
	// topic = this->configInfo->imageTopics.at(3).toStdString();
	// image_sub4 = it_d.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 3));
	// std::thread spinner_thread_d([&callback_queue_d](){
	// 	ros::SingleThreadedSpinner spinner_d;
	// 	spinner_d.spin(&callback_queue_d);
	// });

	// ros::NodeHandle n_e;
	// ros::CallbackQueue callback_queue_e;
	// n_e.setCallbackQueue(&callback_queue_e);
	// image_transport::ImageTransport it_e(n_e);
	// topic = this->configInfo->imageTopics.at(4).toStdString();
	// image_sub5 = it_e.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 4));
	// std::thread spinner_thread_e([&callback_queue_e](){
	// 	ros::SingleThreadedSpinner spinner_e;
	// 	spinner_e.spin(&callback_queue_e);
	// });
	// ros::spin();
	// spinner_thread_a.join();
	// spinner_thread_b.join();
	// spinner_thread_c.join();
	// spinner_thread_d.join();
	// spinner_thread_e.join();
}

void QNode::ImgCallback(const sensor_msgs::ImageConstPtr &msg, int cam_index) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img = cv_ptr->image;
		if (cam_index == 0) {
			Q_EMIT getImage_0(img);
		} else if (cam_index == 1) {
			Q_EMIT getImage_1(img);
		} else if (cam_index == 2) {
			Q_EMIT getImage_2(img);
		} else if (cam_index == 3) {
			Q_EMIT getImage_3(img);
		} else {
			Q_EMIT getImage_4(img);
		}	
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

/*********************** 压缩图像回调函数 ***************************/
void QNode::imgCallback_C0(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		Q_EMIT getImage_0(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

void QNode::imgCallback_C1(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		Q_EMIT getImage_1(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

void QNode::imgCallback_C2(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		Q_EMIT getImage_2(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

void QNode::imgCallback_C3(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		Q_EMIT getImage_3(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

void QNode::imgCallback_C4(const sensor_msgs::CompressedImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

		Q_EMIT getImage_4(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}


/*********************** 非压缩图像回调函数 ***************************/
void QNode::imgCallback_0(const sensor_msgs::ImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage_0(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::imgCallback_1(const sensor_msgs::ImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage_1(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::imgCallback_2(const sensor_msgs::ImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage_2(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::imgCallback_3(const sensor_msgs::ImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage_3(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::imgCallback_4(const sensor_msgs::ImageConstPtr &msg) {
	cv::Mat img;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		Q_EMIT getImage_4(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


}  // namespace parallel_driving
