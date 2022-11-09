#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"

#include "yhs_can_control/GetTopics.h"
#include "controlcan.h"


namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();

private:
	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher lr_wheel_fb_pub_;
	ros::Publisher rr_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher odo_fb_pub_;
	ros::Publisher bms_Infor_fb_pub_;
	ros::Publisher bms_flag_Infor_fb_pub_;
	ros::Publisher Drive_MCUEcoder_fb_pub_;
	ros::Publisher Veh_Diag_fb_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;

	ros::ServiceServer topic_service;

	boost::mutex cmd_mutex_io_;

	boost::mutex cmd_mutex_vel_;

	boost::mutex cmd_mutex_;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	unsigned int canID_io_ = 0;
	unsigned int canID_vel_ = 0;

	VCI_CAN_OBJ send[1];


	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);
	

	void recvData();
	void sendData();

	std::string prefix = "car0_";

};

}


#endif

