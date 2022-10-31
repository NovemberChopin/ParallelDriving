
#ifndef parallel_driving_MAIN_WINDOW_H
#define parallel_driving_MAIN_WINDOW_H

#include <QWidget>
#include <QDebug>
#include <QLabel>
#include <QImage>
#include <QString>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QKeyEvent>
#include <QTimer>
#include <QMenu>
#include <QAction>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_cmd.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "ui_main_window.h"

#include "qnode.hpp"
#include "config_panel.hpp"
#include "page_left.hpp"
#include "page_center.hpp"


namespace parallel_driving {


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void initWindow();		// 初始化界面相关

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);		// 键盘事件


	void sendCtrlCmd();		// 播放ctrl_cmd话题的线程
	// void sendIOCmd();

	void paintEvent(QPaintEvent* event);

public Q_SLOTS:
	void openConfigPanel();				// 打开ROS配置页面
	void closeWindow();
	void connectByConfig(ConfigInfo* config);	// 处理配置弹窗

	void slot_updateCtrlMsg(int gear, float velo, float steer);

	void handleVelocity();				// 处理速度
	void handleSteer();					// 处理转向角

	void setImage_0(cv::Mat img);
	void setImage_1(cv::Mat img);
	void setImage_2(cv::Mat img);
	void setImage_3(cv::Mat img);

	/* 界面相关槽函数 */
	void menu_pop_load_config();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;				// ROS节点相关
	ConfigPanel *configP;		// ROS配置对话框
	PageLeft *pageL;			// 左边页面
	PageCenter *pageC;			// 中间页面


	/***** 右键弹出菜单变量 ******/
	QMenu *p_Menu;
	QAction *p_load_action;

	/***** 控制车辆运动的变量 ******/
	yhs_can_msgs::ctrl_cmd ctrl_msg_;

	QTimer *p_velo_timer;		// 速度按键计时器
	QTimer *p_steer_timer;		// 转向角按键计时器
	int velo_cmd_num = 0;
	int steer_cmd_num = 0;

	int click_num = 0;
	bool is_long_press = false;

	bool isBrake = true;		// 默认刹车
	bool gear_P = true;			// 默认为驻车
	bool key_up = false;
	bool key_down = false;
	bool key_left = false;
	bool key_right = false;

	
};

}

#endif
