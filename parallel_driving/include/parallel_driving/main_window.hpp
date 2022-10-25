
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

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_cmd.h"

#include "ui_main_window.h"

#include "qnode.hpp"
#include "config_panel.hpp"


namespace parallel_driving {


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	QWidget* init_main_page();

	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);		// 键盘事件


public Q_SLOTS:
	void openConfigPanel();				// 打开ROS配置页面
	void connectByConfig(ConfigInfo* config);	// 处理配置弹窗

	void handleCtrlMsg();

	void setImage_0(cv::Mat img);
	void setImage_1(cv::Mat img);
	void setImage_2(cv::Mat img);
	void setImage_3(cv::Mat img);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;				// ROS节点相关
	ConfigPanel *configP;		// ROS配置对话框
	/*** main window ***/
	QLabel *title_label;
	QLabel *main_label;
	QPushButton *btn_config;

	yhs_can_msgs::ctrl_cmd ctrl_msg_;

	QTimer *p_velo_timer;		// 速度按键计时器
	QTimer *p_steer_timer;		// 转向角按键计时器
	int velo_cmd_num = 0;
	int steer_cmd_num = 0;

	int click_num = 0;
	bool is_long_press = false;

	bool key_up = false;
	bool key_down = false;
	bool key_left = false;
	bool key_right = false;

	
};

}

#endif
