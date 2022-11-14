#include "ros/ros.h"

#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"

#include <sensor_msgs/Joy.h>    // 罗技话题数据
#include "joy_to_car/GetPrefix.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

class JoyToCar
{
private:
    
    std::string prefix = "";        // 当前的小车
    /* data */
    u_int8_t trun_lamp_;            // 转向灯
    bool upper_beam_;               // 远光灯
    u_int8_t gear_;                 // 车辆档位
    float velo_fb_;                 // 车辆当前的速度


    yhs_can_msgs::ctrl_cmd ctrl_msg_;    // ctrl数据
    yhs_can_msgs::io_cmd io_msg_;       // io数据

    ros::NodeHandle node_;
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_ctrl_fb_;
    ros::Publisher pub_ctrl_;
    ros::Publisher pub_io_;

    ros::ServiceServer service;
public:
    JoyToCar(/* args */);
    ~JoyToCar();

    bool init();
    void joyCallback(const sensor_msgs::JoyConstPtr& msg);
    void ctrlFBCallback(const yhs_can_msgs::ctrl_fb& msg);
    void sendCtrlCmd();
    void sendIOCmd();

    void shutdownTopics();
    void restoreTopics(std::string prefix);

    bool getPrefixCall(joy_to_car::GetPrefix::Request &req, 
                    joy_to_car::GetPrefix::Response &res);
};