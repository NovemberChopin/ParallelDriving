#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"


namespace yhs_tool {


CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}


CanControl::~CanControl()
{

}

//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = msg.io_cmd_enable;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send[0].ID = 0x18C4D7D0;

	memcpy(send[0].Data,sendData_u_io_,8);

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
	}

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;
	short angular = msg.ctrl_cmd_steering * 100;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	sendData_u_vel_[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send[0].ID = 0x18C4D2D0;

	memcpy(send[0].Data,sendData_u_vel_,8);

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
	}

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{
	ros::Rate loop(100);

	int reclen=0;
	int i,j;
	int ind=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。

	while(ros::ok())
	{

		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				
				switch (rec[j].ID)
				{
					//速度控制反馈
					case 0x18C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & rec[j].Data[0];
						
						msg.ctrl_fb_velocity = (float)((unsigned short)((rec[j].Data[2] & 0x0f) << 12 | rec[j].Data[1] << 4 | (rec[j].Data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_steering = (float)((short)((rec[j].Data[4] & 0x0f) << 12 | rec[j].Data[3] << 4 | (rec[j].Data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_Brake = (rec[j].Data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = (rec[j].Data[4] & 0xc0) >> 6;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							ctrl_fb_pub_.publish(msg);
						}

						break;
					}

					//左轮反馈
					case 0x18C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//右轮反馈
					case 0x18C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x18C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & rec[j].Data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;
	
						if(0x02 & rec[j].Data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0x0c & rec[j].Data[1]) >> 2;

						if(0x10 & rec[j].Data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x01 & rec[j].Data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x02 & rec[j].Data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & rec[j].Data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x18C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(rec[j].Data[3] << 24 | rec[j].Data[2] << 16 | rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(rec[j].Data[7] << 24 | rec[j].Data[6] << 16 | rec[j].Data[5] << 8 | rec[j].Data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x18C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(rec[j].Data[3] << 8 | rec[j].Data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(rec[j].Data[5] << 8 | rec[j].Data[4])) / 100;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x18C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = rec[j].Data[0];

						if(0x01 & rec[j].Data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & rec[j].Data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & rec[j].Data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & rec[j].Data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & rec[j].Data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & rec[j].Data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & rec[j].Data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & rec[j].Data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & rec[j].Data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & rec[j].Data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & rec[j].Data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & rec[j].Data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & rec[j].Data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & rec[j].Data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(rec[j].Data[4] << 4 | rec[j].Data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((rec[j].Data[6] & 0x0f) << 8 | rec[j].Data[5])) / 10;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x18C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(rec[j].Data[3] << 24 | rec[j].Data[2] << 16 | rec[j].Data[1] << 8 | rec[j].Data[0]); 

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x18C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & rec[j].Data[0];

						if(0x10 & rec[j].Data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & rec[j].Data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & rec[j].Data[1]) msg.Veh_fb_STMCUDisOnline = true;	else msg.Veh_fb_STMCUDisOnline = false;

						if(0x02 & rec[j].Data[1]) msg.Veh_fb_STEncoderDisOnline = true;	else msg.Veh_fb_STEncoderDisOnline = false;

						if(0x04 & rec[j].Data[1]) msg.Veh_fb_STOverCurrent = true;	else msg.Veh_fb_STOverCurrent = false;

						if(0x10 & rec[j].Data[2]) msg.Veh_fb_BraMCUDisOnlie = true;	else msg.Veh_fb_BraMCUDisOnlie = false;

						if(0x20 & rec[j].Data[2]) msg.Veh_fb_BraOverCurrent = true;	else msg.Veh_fb_BraOverCurrent = false;

						if(0x01 & rec[j].Data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & rec[j].Data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & rec[j].Data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & rec[j].Data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & rec[j].Data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & rec[j].Data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & rec[j].Data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & rec[j].Data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;

						if(0x10 & rec[j].Data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & rec[j].Data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & rec[j].Data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & rec[j].Data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

						if(crc == rec[j].Data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					default:
						break;
				}

			}

					
		}

		loop.sleep();
	}
}

//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

	//复位CAN1通道。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);
	usleep(100000);

	//关闭设备。
	VCI_CloseDevice(VCI_USBCAN2,0);	
}


bool getTopicsCall(yhs_can_control::GetTopics::Request &req,
				   yhs_can_control::GetTopics::Response &res) {
	ros::V_string topics;
	res.nodeName = ros::this_node::getName();
	std::cout << "---- server ----" << std::endl;
	std::cout << "nodeName: " << res.nodeName << std::endl;
	// 获取当前发布的话题
	std::cout << "pub topic: " << std::endl;
	ros::this_node::getAdvertisedTopics(topics);
	for (int i=0; i<topics.size(); i++) {
		std::cout << topics[i] << std::endl;
		std_msgs::MultiArrayDimension dim;
		dim.label = topics[i];
		res.pub_topics.dim.push_back(dim);
	}
	// 获取当前订阅的话题
	topics.clear();
	std::cout << "sub topic: " << std::endl;
	ros::this_node::getSubscribedTopics(topics);
	for (auto item: topics) {
		std::cout << item << std::endl;
		std_msgs::MultiArrayDimension dim;
		dim.label = item;
		res.sub_topics.dim.push_back(dim);
	}
	return true;		
}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>(prefix+"ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>(prefix+"io_cmd", 5, &CanControl::io_cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>(prefix+"ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>(prefix+"lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>(prefix+"rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>(prefix+"io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>(prefix+"odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>(prefix+"bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>(prefix+"bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>(prefix+"Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>(prefix+"Veh_Diag_fb",5);

	// service
	topic_service = nh_.advertiseService(prefix+"get_topics", getTopicsCall);

	//打开设备
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
	{
		ROS_INFO(">>open can deivce success!");
	}
	else
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}


	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	//接收所有帧
	config.Filter=1;
	/*波特率500 Kbps  0x00  0x1C*/
	config.Timing0=0x00;
	config.Timing1=0x1C;
	//正常模式	
	config.Mode=0;	

	send[0].SendType=1;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=1;
	send[0].DataLen=8;
	
	bool error = false;
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		ROS_ERROR(">>Init CAN1 error!\n");
		error = true;
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		ROS_ERROR(">>Start CAN1 error!\n");
		error = true;
	}


	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		ROS_ERROR(">>Init can2 error!\n");
		error = true;
	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		ROS_ERROR(">>Start can2 error!\n");
		error = true;
	}

	if(error)
	{
		VCI_CloseDevice(VCI_USBCAN2,0);
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();
}

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "car0_yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
