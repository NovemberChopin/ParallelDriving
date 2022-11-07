
#include "../include/joy_to_car/joy_to_car.h"

JoyToCar::JoyToCar(/* args */) {
    
    this->velo_fb_ = 0.0;       // 初始化速度为0    
    this->gear_ = 1;            // 默认为驻车档
    this->trun_lamp_ = 0;       // 转向灯，默认关闭；
    this->upper_beam_ = false;  // 远光灯，默认关闭；
}

JoyToCar::~JoyToCar() {}

bool JoyToCar::init() {
    if (! ros::master::check()) {
        return false;
    }
    ros::start();
    sub_joy_ = node_.subscribe("joy", 10, &JoyToCar::joyCallback, this);
    sub_ctrl_fb_ = node_.subscribe("ctrl_fb", 5, &JoyToCar::ctrlFBCallback, this);

    pub_ctrl_ = node_.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5);
    pub_io_ = node_.advertise<yhs_can_msgs::io_cmd>("io_cmd", 5);

    boost::thread send_ctrl_thread(boost::bind(&JoyToCar::sendCtrlCmd, this));
    boost::thread send_io_thread(boost::bind(&JoyToCar::sendIOCmd, this));
    ros::spin();
    return true;
}


void JoyToCar::sendCtrlCmd() {
    ros::Rate loop_rate(100);
    
    ROS_INFO("start send ctrl message");
    while (ros::ok())
    {
        pub_ctrl_.publish(ctrl_msg_);

        ros::spinOnce();

        loop_rate.sleep();
    }
}


void JoyToCar::sendIOCmd() {
    ros::Rate loop_rate(40);

    ROS_INFO("start send io message");

    while (ros::ok()) {

        pub_io_.publish(io_msg_);

        ros::spinOnce();

        loop_rate.sleep();
    }
}

/**
 * @brief 把方向盘数据映射到小车
 * ctrl_cmd_gear： 档位
 *      01： 驻车P:车辆在完全停止时才可以使用
 *      02： 后退R
 *      03： 空挡N
 *      04： 前进D
 * ctrl_cmd_velocity： 速度范围0-5
 * ctrl_cmd_steering： 转向角范围（25:-25）度
 * 
 * @param msg 
 */
void JoyToCar::joyCallback(const sensor_msgs::JoyConstPtr& msg) {
    float velocity, steer;

    if (velo_fb_ == 0 && msg->buttons[14] == 1) {    // 当速度为0时候切换前进档位
        this->gear_ = 4;
        ROS_INFO("Change gear to D");
    }
    if (this->gear_ == 4 && msg->buttons[14] == 0) {
        this->gear_ = 3;    // 改为空档
        ROS_INFO("Change gear to N");
    }
    if (velo_fb_ == 0 && msg->buttons[15] == 1) {   // buttons[15] 后退档位
        this->gear_ = 2;
        ROS_INFO("Change gear to R");
    }
    if (this->gear_ == 2 && msg->buttons[15] == 0) {
        this->gear_ = 3;
        ROS_INFO("Change gear to N");
    }
    
    // 计算速度 
    float f_vel = 2.5 * msg->axes[2] + 2.5;
    float b_vel = -(2.5 * msg->axes[3] + 2.5);
    if (msg->axes[2] == 0 || msg->axes[2] == -1) {
        f_vel = 0;
    }
    // 第一次连接上方向盘，刹车的值 mas->axes[3] 为 0. 此时刹车速度也应该为 0
    if (msg->axes[3] == 0 || msg->axes[3] == -1) {
        b_vel = 0;
    }
    velocity = f_vel + b_vel;
    if (velocity > 5)   velocity = 5.0;
    if (velocity < 0)   velocity = 0.0;

    // 计算转向角
    steer = msg->axes[0] * 25.0;
    if (steer > 0) {            // 自动开启转向灯
        this->trun_lamp_ = 1;
    } else if (steer < 0) {
        this->trun_lamp_ = 2;
    } else {
        this->trun_lamp_ = 0;
    }

    ctrl_msg_.ctrl_cmd_gear = this->gear_;
    ctrl_msg_.ctrl_cmd_velocity = velocity;
    ctrl_msg_.ctrl_cmd_steering = steer;
    ctrl_msg_.ctrl_cmd_Brake = 0;


    /********** IO 数据 ***********/
    io_msg_.io_cmd_enable = false;                  // I/O 控制使能
    io_msg_.io_cmd_upper_beam_headlamp = false;     // 远光灯开关
    io_msg_.io_cmd_turn_lamp = this->trun_lamp_;    // 转向灯开关 0：全关，1：左转向。2：右转向
    io_msg_.io_cmd_speaker = false;                 // 扬声器开关
    
    if (msg->buttons[23] == 1) {                    // 扬声器
        io_msg_.io_cmd_enable = true;
        io_msg_.io_cmd_speaker = true;
    }
    
    if (msg->buttons[10] == 1) {                    // 远光灯 R3 键   
        this->upper_beam_ = !this->upper_beam_;
    }
    if (this->upper_beam_) {
        io_msg_.io_cmd_enable = true;
        io_msg_.io_cmd_upper_beam_headlamp = this->upper_beam_;
    }

}

void JoyToCar::ctrlFBCallback(const yhs_can_msgs::ctrl_fb& msg) {
    // std::cout << "velocity fb is " << msg.ctrl_fb_velocity << " " << msg.ctrl_fb_steering << std::endl;
    this->velo_fb_ = msg.ctrl_fb_velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_car");

    JoyToCar joy_to_car = JoyToCar();

    joy_to_car.init();

    return 0;
}