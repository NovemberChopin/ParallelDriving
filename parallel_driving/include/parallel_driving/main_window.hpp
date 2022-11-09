
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
#include <QFileDialog>
#include <QStringList>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "yhs_can_control/GetTopics.h"
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

	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);		// 键盘事件


	void sendCtrlCmd();		// 播放ctrl_cmd话题的线程
	// void sendIOCmd();

	void paintEvent(QPaintEvent* event);

	void loadCameraMatrix(int cam_index);	// 加载相机参数
	QPixmap fixImage(cv::Mat img, int cam_index);// 修复并转换图像

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
	void setImage_4(cv::Mat img);

	/* 界面相关槽函数 */
	void menu_pop_load_config();

	void getTopic_slot(const QString &node);
	void getSelectedImg_slot(QStringList *topics);
	void testNewFeatures();			// 测试槽函数

private:
	bool isShutdown = true;
	std::string prefix = "";

	Ui::MainWindowDesign ui;
	QNode qnode;				// ROS节点相关
	ConfigPanel *configP;		// ROS配置对话框
	PageLeft *pageL;			// 左边页面
	PageCenter *pageC;			// 中间页面

	int cam_num;				// 相机数量
	// 相机参数
	std::vector<bool> vec_hasLoadCameraMatrix;
	bool hasLoadCameraMatrix;						// 是否加载了相机相关参数
	std::vector<cv::Mat> vec_cameraMatrix;			// 相机内参矩阵
	std::vector<cv::Mat> vec_distCoeffs;			// 相机畸变参数
	std::vector<cv::Mat> vec_map1, vec_map2;		// 畸变修复映射矩阵
	cv::Size image_size;


	/***** 右键弹出菜单变量 ******/
	QMenu *p_Menu;
	QAction *p_load_action;
	// 当前右键菜单所在的相机
	// left_up left_down main right_up right_down
	// 1 2 0 3 4
	int cur_cam_index;			

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
