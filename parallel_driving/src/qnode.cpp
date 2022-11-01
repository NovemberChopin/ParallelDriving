
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


bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	// ros::init(remappings, "parallel_driving");
	ros::init(init_argc, init_argv, "parallel_driving");
	
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::NodeHandle node;
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	pub_ctrl_cmd = node.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5);
	pub_io_cmd = node.advertise<yhs_can_msgs::io_cmd>("io_cmd", 5);

	sub_ctrl_fb = node.subscribe("ctrl_fb", 5, &QNode::ctrlCallback, this);
	this->start();		// 启动线程
	return true;
}

void QNode::setConfigInfo(ConfigInfo *config) {
	qDebug() << "set config info";
	this->configInfo = config;
}



void QNode::sendCtrlCmd() {
	ros::Rate loop_rate(10);

    while (ros::ok())
    {	
		ROS_INFO("start send ctrl message: %d", ctrl_msg_.ctrl_cmd_gear);
        pub_ctrl_cmd.publish(this->ctrl_msg_);

        ros::spinOnce();

        loop_rate.sleep();
    }
}


void QNode::ctrlCallback(const yhs_can_msgs::ctrl_fb& msg) {
	// std::cout << "velocity fb is " << msg.ctrl_fb_velocity << " " << msg.ctrl_fb_steering << std::endl;
	this->velo_fb_ = msg.ctrl_fb_velocity;
	this->steer_fb_ = msg.ctrl_fb_steering;
	this->gear_fb_ = msg.ctrl_fb_gear;
	// 在这里更新速度信息
	int gear = msg.ctrl_fb_gear;
	Q_EMIT updateCtrlMsg(gear, this->velo_fb_, this->steer_fb_);
}

// void QNode::run() {
// 	ros::Rate loop_rate(10);

// 	while (ros::ok())
// 	{
// 		ROS_INFO("start send ctrl message: %d", ctrl_msg_.ctrl_cmd_gear);
// 		this->pub_ctrl_cmd.publish(this->ctrl_msg_);
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// 	Q_EMIT rosShutdown();
// }


void QNode::run() {
	
	// boost::thread ctrl_thread(boost::bind(&QNode::sendCtrlCmd, this));
	
	std::string topic;
	ros::NodeHandle n_a;
	ros::CallbackQueue callback_queue_a;
	n_a.setCallbackQueue(&callback_queue_a);
	image_transport::ImageTransport it(n_a);
	// 0 表示 Callback 的第二个参数 cam_index
	topic = this->configInfo->imageTopics.at(0).toStdString();
	image_sub1 = it.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 0));
	std::thread spinner_thread_a([&callback_queue_a](){
		ros::SingleThreadedSpinner spinner_a;
		spinner_a.spin(&callback_queue_a);
	});
	
	ros::NodeHandle n_b;
	ros::CallbackQueue callback_queue_b;
	n_b.setCallbackQueue(&callback_queue_b);
	image_transport::ImageTransport it_b(n_b);
	topic = this->configInfo->imageTopics.at(1).toStdString();
	image_sub2 = it_b.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 1));
	std::thread spinner_thread_b([&callback_queue_b](){
		ros::SingleThreadedSpinner spinner_b;
		spinner_b.spin(&callback_queue_b);
	});

	ros::NodeHandle n_c;
	ros::CallbackQueue callback_queue_c;
	n_c.setCallbackQueue(&callback_queue_c);
	image_transport::ImageTransport it_c(n_c);
	topic = this->configInfo->imageTopics.at(2).toStdString();
	image_sub3 = it_c.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 2));
	std::thread spinner_thread_c([&callback_queue_c](){
		ros::SingleThreadedSpinner spinner_c;
		spinner_c.spin(&callback_queue_c);
	});
	
	ros::NodeHandle n_d;
	ros::CallbackQueue callback_queue_d;
	n_d.setCallbackQueue(&callback_queue_d);
	image_transport::ImageTransport it_d(n_d);
	topic = this->configInfo->imageTopics.at(3).toStdString();
	image_sub4 = it_d.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 3));
	std::thread spinner_thread_d([&callback_queue_d](){
		ros::SingleThreadedSpinner spinner_d;
		spinner_d.spin(&callback_queue_d);
	});

	ros::NodeHandle n_e;
	ros::CallbackQueue callback_queue_e;
	n_e.setCallbackQueue(&callback_queue_e);
	image_transport::ImageTransport it_e(n_e);
	topic = this->configInfo->imageTopics.at(4).toStdString();
	image_sub5 = it_e.subscribe(topic, 1, boost::bind(&QNode::ImgCallback, this, _1, 4));
	std::thread spinner_thread_e([&callback_queue_e](){
		ros::SingleThreadedSpinner spinner_e;
		spinner_e.spin(&callback_queue_e);
	});
	ros::spin();
	spinner_thread_a.join();
	spinner_thread_b.join();
	spinner_thread_c.join();
	spinner_thread_d.join();
	spinner_thread_e.join();
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


}  // namespace parallel_driving
