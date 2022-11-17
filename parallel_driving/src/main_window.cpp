
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/parallel_driving/main_window.hpp"

namespace parallel_driving {

using namespace Qt;
using namespace cv;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); 

	this->setAttribute(Qt::WA_QuitOnClose, true);

	configP = new ConfigPanel();
	pageL = new PageLeft();
	pageC = new PageCenter();

	p_velo_timer = new QTimer(this);
	p_steer_timer = new QTimer(this);
	p_check_timer = new QTimer(this);
	if (p_check_timer->isActive() == false) {
		p_check_timer->start(1000);		// 时间间隔为 1 秒
	}

	this->cam_num = 5;			// 相机数量
	
	/* 初始化相机参数变量 */
	image_size = cv::Size(1280, 720);
	cv::Mat m(3, 5, CV_32FC1, Scalar(1));
	for (int i=0; i < cam_num; i++) {       // 初始化 ROI
        // cur_frame.push_back(fill_img);
        this->vec_hasLoadCameraMatrix.push_back(false);
        this->vec_cameraMatrix.push_back(m);
        this->vec_distCoeffs.push_back(m);
        this->vec_map1.push_back(m);
        this->vec_map2.push_back(m);
    }

	this->initPubMsg();

	this->initWindow();		// 初始化界面
	// QT类获取键盘焦点时才可以获取事件，当页面布局复杂时，焦点可能不在该类中。
	this->setFocusPolicy(Qt::StrongFocus);
	// ros 节点信号
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(getImage_0(cv::Mat)), this, SLOT(setImage_0(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_1(cv::Mat)), this, SLOT(setImage_1(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_2(cv::Mat)), this, SLOT(setImage_2(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_3(cv::Mat)), this, SLOT(setImage_3(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_4(cv::Mat)), this, SLOT(setImage_4(cv::Mat)));
	// 控制信息反馈信号
	QObject::connect(&qnode, SIGNAL(updateCtrlMsg(int, float, float)), 
					 this, SLOT(slot_updateCtrlMsg(int, float, float)));
	
	QObject::connect(ui.btn_close, &QPushButton::clicked, this, &MainWindow::closeWindow);
	QObject::connect(ui.btn_config, &QPushButton::clicked, this, &MainWindow::openConfigPanel);
	// 登录页面的信号
	QObject::connect(configP, SIGNAL(getConfigInfo(ConfigInfo*)), this, SLOT(connectByConfig(ConfigInfo*)));
	QObject::connect(configP, SIGNAL(getSelectedImg_signal(QStringList*, std::string, bool)), 
						this, SLOT(getSelectedImg_slot(QStringList*, std::string, bool)));
	// 计时器超时信号
	QObject::connect(p_velo_timer, &QTimer::timeout, this, &MainWindow::handleVelocity);
	QObject::connect(p_steer_timer, &QTimer::timeout, this, &MainWindow::handleSteer);
	QObject::connect(p_check_timer, &QTimer::timeout, this, &MainWindow::checkROSStatus);
	// this->setCentralWidget(this->init_main_page());
}

MainWindow::~MainWindow() {}

/**
 * @brief 传递给 qnode 并启动相应的 发布订阅程序
 * 	
 * @param topics 
 */
void MainWindow::getSelectedImg_slot(QStringList *topics, std::string prefix, bool hasCompress) {
	if (topics->size() != 5) {
		QMessageBox::information(this, "错误", "话题不足五个!");
		return;
	}
	qDebug() << "--- getSelectedTopic_slot ---";
	this->prefix = prefix;
	this->qnode.setImageTopic(topics, hasCompress);			// 把选择的 topic 消息发给 qnode
	
	if (this->hasJoy) {		// 当前由方向盘控制
		// 通知方向盘，切换小车: 通过 get_prefix 服务传递给 joy_to_car 节点
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<joy_to_car::GetPrefix>("get_prefix");
		joy_to_car::GetPrefix srv;
		srv.request.prefix = prefix;
		if (client.call(srv)) {
			QString text = QString::fromStdString("方向盘已切换至" + prefix + "小车");
			QMessageBox::information(this, "提示", text);
		}
	} else {				// 软件控制
		std::cout << "关闭所有话题: " << prefix << std::endl;
		this->qnode.shutdownPubTopic();
		this->qnode.shutdownSubTopic();
		// this->stopThread();				// 停止线程（不清楚为啥不起作用）

		std::cout << "重新启动话题: " << prefix << std::endl;
		this->qnode.restorePubTopic(prefix+'/');
		this->qnode.restoreSubTopic(prefix+'/');
		// if (this->hasJoy == false) {
		// 	this->startThread();		// 启动发送话题线程
		// }
	}

	// 更新主界面小车话题展示
	ui.car_name->setText(QString::fromStdString(prefix));
	listwidget->clear();
	// const QStringList topicList = *topics;
	// listwidget->addItems(topicList);

	for (int i=0; i<topics->size(); i++) {
		QListWidgetItem *item = new QListWidgetItem;
		item->setSizeHint(QSize(0, 30));
		const QString topic = topics->at(i);
		item->setText(topic);
		listwidget->addItem(item);
	}
}


void MainWindow::initWindow() {
	this->setWindowTitle("平行驾驶-右");
	setWindowIcon(QIcon(":/images/logo.jpg"));

	this->p_Menu = new QMenu(this);
	this->p_load_action = new QAction(QStringLiteral("加载相机配置"), this);
	this->p_Menu->addAction(this->p_load_action);
	QObject::connect(this->p_load_action, &QAction::triggered, this, &MainWindow::menu_pop_load_config);
	
	this->pageC->ui->main_label->setContextMenuPolicy(Qt::CustomContextMenu);
	this->pageL->ui->left_img_up->setContextMenuPolicy(Qt::CustomContextMenu);
	this->pageL->ui->left_img_down->setContextMenuPolicy(Qt::CustomContextMenu);
	this->ui.right_img_up->setContextMenuPolicy(Qt::CustomContextMenu);
	this->ui.right_img_down->setContextMenuPolicy(Qt::CustomContextMenu);

	QObject::connect(this->pageC->ui->main_label, &QLabel::customContextMenuRequested, [=](){
		this->cur_cam_index = 0;
		this->p_Menu->exec(QCursor::pos());
	});
	QObject::connect(this->pageL->ui->left_img_up, &QLabel::customContextMenuRequested, [=](){
		this->cur_cam_index = 1;
		this->p_Menu->exec(QCursor::pos());
	});
	QObject::connect(this->pageL->ui->left_img_down, &QLabel::customContextMenuRequested, [=](){
		this->cur_cam_index = 2;
		this->p_Menu->exec(QCursor::pos());
	});
	QObject::connect(this->ui.right_img_up, &QLabel::customContextMenuRequested, [=](){
		this->cur_cam_index = 3;
		this->p_Menu->exec(QCursor::pos());
	});
	QObject::connect(this->ui.right_img_down, &QLabel::customContextMenuRequested, [=](){
		this->cur_cam_index = 4;
		this->p_Menu->exec(QCursor::pos());
	});

	ui.master_status->setText("未连接");
	ui.joy_status->setText("未连接");

	// 初始化信息展示区域小车节点信息
	ui.car_name->setText("未连接小车");
	listwidget = new QListWidget;
	listwidget->setStyleSheet("background-color: rgb(7, 28, 43);");
	QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(listwidget);
    ui.groupBox->setLayout(layout);

	// 显示页面
	this->pageL->show();
	this->pageC->show();
	this->show();
}


void MainWindow::initPubMsg() {
	this->gear_P = true;
	/* 初始化 ctrl_cmd 话题数据 */
	u_int8_t gear = 1;				// 初始化时候为驻车
	u_int8_t brake = 0;
	float velocity = 0.0;
	float steer = 0.0;
	this->ctrl_msg_.ctrl_cmd_gear = gear;
	this->ctrl_msg_.ctrl_cmd_velocity = velocity;
	this->ctrl_msg_.ctrl_cmd_steering = steer;
	this->ctrl_msg_.ctrl_cmd_Brake = brake;
}

/**
 * @brief 接收 qnode 接收到的小车 速度信息 信号，并将其展示
 * 
 * @param gear 档位信息
 * @param velo 速度
 * @param steer	转向角 
 */
void MainWindow::slot_updateCtrlMsg(int gear, float velo, float steer) {
	// qDebug() << gear << " " << velo << " " << steer;
	// 更新速度信息
	this->pageL->dash_1->setValue(velo);
	this->pageL->dash_1->update();

	this->pageL->dash_2->setValue(velo);
	this->pageL->dash_2->update();

	// 更新档位信息
	switch (gear)
	{
	case 1:	
		/* P */
		this->pageL->setGear_P();
		break;
	case 2:
		// R
		this->pageL->setGear_R();
		break;
	case 3:
		// N
		this->pageL->setGear_N();
		break;
	default:
		// D
		this->pageL->setGear_D();
		break;
	} 

	// 更新方向盘转向
	this->pageL->updateSteer(steer);

}

void MainWindow::handleSteer() {
	this->steer_cmd_num++;
	float steer = 0;
	if (key_left) {
		steer = qnode.steer_fb_ + 0.05 * steer_cmd_num;
		if (steer < -25.0)	steer = -25.0;
	} else {
		steer = qnode.steer_fb_ - 0.05 * steer_cmd_num;
		if (steer > 25.0)  steer = 25.0;
	}
	this->ctrl_msg_.ctrl_cmd_steering = steer;
}

// 处理速度
void MainWindow::handleVelocity() {
	if (gear_P) {	// 如果现在为驻车，直接返回默认值
		return;
	} 
	this->velo_cmd_num++;
	u_int8_t gear = 3;			// 默认空挡
	float velocity = 0.0;
	if (key_up) {				// 前进
		gear = 4;
		velocity = qnode.velo_fb_ + 0.01 * velo_cmd_num;
		if (velocity >= 5.0) velocity = 5.0;
	} 
	if (key_down) {				// 后退
		gear = 2;
		velocity = qnode.velo_fb_ + 0.01 * velo_cmd_num;
		if (velocity > 5.0) velocity = 5.0;
	}

	this->ctrl_msg_.ctrl_cmd_gear = gear;
	this->ctrl_msg_.ctrl_cmd_velocity = velocity;
}



void MainWindow::sendCtrlCmd() {
	if (!this->hasJoy) {
		ros::Rate loop_rate(100);		// 以100hz的频率发送话题
    
		ROS_INFO("start send ctrl message");
		while (ros::ok())
		{	
			this->qnode.pub_ctrl_cmd.publish(this->ctrl_msg_);

			ros::spinOnce();

			loop_rate.sleep();
		}
	}
}


void MainWindow::keyPressEvent(QKeyEvent *event) {
	if (this->hasJoy && (event->key() == Qt::Key_P || event->key() == Qt::Key_Space || event->key() == Qt::Key_Up ||
		event->key() == Qt::Key_Down || event->key() == Qt::Key_Left || event->key() == Qt::Key_Right)) {
		QMessageBox::information(this, "注意", "现已切换至方向盘控制!");
		return;
	}

	Q_UNUSED(event);
	if (event->key() == Qt::Key_P) {
		this->gear_P = !this->gear_P;
		qDebug() << "gear_P: " << this->gear_P;
		if (this->gear_P) {		// 如果改为驻车, 那么方向自动回正
			this->ctrl_msg_.ctrl_cmd_gear = u_int8_t(1);
			this->ctrl_msg_.ctrl_cmd_velocity = 0;
			this->ctrl_msg_.ctrl_cmd_steering = 0;
		}
	}

	if (event->key() == Qt::Key_Space) {		// 按下空格开始刹车
		if (event->isAutoRepeat())	return;

		this->ctrl_msg_.ctrl_cmd_gear = u_int8_t(3);
		this->ctrl_msg_.ctrl_cmd_Brake = u_int8_t(1);
		qDebug() << "brake: true";
	}

	if (event->key() == Qt::Key_Up) {			// 方向键 上
		if (event->isAutoRepeat()) return;
		
		if (p_velo_timer->isActive() == false) {
			key_up = true;
			qDebug() << "Key_Up press " << key_up;
			p_velo_timer->start(10);
		}
	} else if (event->key() == Qt::Key_Down) {	// 方向键 下
		if (event->isAutoRepeat()) return;
		
		if (p_velo_timer->isActive() == false) {
			key_down = true;
			qDebug() << "Key_down press " << key_up;
			p_velo_timer->start(10);
		}
	}

	if (event->key() == Qt::Key_Left) {			// 方向键 左
		if (event->isAutoRepeat()) return;

		if (p_steer_timer->isActive() == false) {
			key_left = true;
			qDebug() << "Key_left press " << key_left;
			p_steer_timer->start(10);
		}
	} else if (event->key() == Qt::Key_Right) {		// 方向键 右
		if (event->isAutoRepeat()) return;

		if (p_steer_timer->isActive() == false) {
			key_right = false;
			qDebug() << "Key_right press " << key_right;
			p_steer_timer->start(10);
		}
	}

}


void MainWindow::keyReleaseEvent(QKeyEvent *event) {
	Q_UNUSED(event);
	// 结束刹车，调为空挡
	if (event->key() == Qt::Key_Space) {	
		if (event->isAutoRepeat())	return;

		this->ctrl_msg_.ctrl_cmd_gear = u_int8_t(3);
		this->ctrl_msg_.ctrl_cmd_Brake = u_int8_t(0);
		qDebug() << "brake: false";
	}

	if (event->key() == Qt::Key_Up) {
		if (event->isAutoRepeat()) return;
		key_up = false;
		qDebug() << "Key_Up release" << key_up;
		// 改为空挡
		this->ctrl_msg_.ctrl_cmd_gear = u_int8_t(3);
		if (p_velo_timer->isActive() == true) {
			p_velo_timer->stop();
			this->velo_cmd_num = 0;
		}
	} else if (event->key() == Qt::Key_Down) {
		if (event->isAutoRepeat()) return;
		key_down = false;
		qDebug() << "Key_down release" << key_up;
		// 改为空挡
		this->ctrl_msg_.ctrl_cmd_gear = u_int8_t(3);
		if (p_velo_timer->isActive() == true) {
			p_velo_timer->stop();
			this->velo_cmd_num = 0;
		}
	}

	// 左右方向键：控制小车转向
	if (event->key() == Qt::Key_Left) {
		if (event->isAutoRepeat()) return;
		key_left = false;
		qDebug() << "Key_left release" << key_left;
		if (p_steer_timer->isActive() == true) {
			p_steer_timer->stop();
			this->steer_cmd_num = 0;
		}
	} else if (event->key() == Qt::Key_Right) {
		if (event->isAutoRepeat()) return;
		key_right = false;
		qDebug() << "Key_right release" << key_right;
		if (p_steer_timer->isActive() == true) {
			p_steer_timer->stop();
			this->steer_cmd_num = 0;
		}
	}
}


void MainWindow::menu_pop_load_config() {
	qDebug() << "cur_cam_index: " << cur_cam_index;
	this->loadCameraMatrix(this->cur_cam_index);
}


void MainWindow::loadCameraMatrix(int cam_index) {
	qDebug() << "menu_pop_load_config";
    QString filename = QFileDialog::getOpenFileName(this, "Open", "./", "(*.yml)");
    if (filename.isEmpty()) 
        return;
	cv::Mat cameraMatrix, distCoeffs;
    cv::Mat map1, map2;
	// 读取相机内参
	cv::FileStorage cameraPameras(filename.toStdString(), cv::FileStorage::READ);
    cameraPameras["camera_matrix"] >> cameraMatrix;
    cameraPameras["dist_coeffs"] >> distCoeffs;
    this->vec_cameraMatrix[cam_index] = cameraMatrix;
    this->vec_distCoeffs[cam_index] = distCoeffs;
	// 计算修复畸变的映射矩阵
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, image_size, CV_16SC2, map1, map2);
    this->vec_map1[cam_index] = map1;
    this->vec_map2[cam_index] = map2;
	this->vec_hasLoadCameraMatrix[cam_index] = true;
}


void MainWindow::openConfigPanel() {
	ROS_INFO("Open ROS config panel");
	configP->show();
}


/**
 * @brief 以固定的频率执行，系统的状态
 * 
 */
void MainWindow::checkROSStatus() {
    // 1. 如果主节点未启动，禁用除连接按钮之外的所有按钮
	Ui::ConfigPanel* config_ui = configP->getUIPointer();
    if (!ros::master::check()) {
        config_ui->btn_connect->setEnabled(true);
        config_ui->btn_refresh->setEnabled(false);
        config_ui->btn_confirm->setEnabled(false);
    } else {
        config_ui->btn_connect->setEnabled(false);
        config_ui->btn_refresh->setEnabled(true);
        config_ui->btn_confirm->setEnabled(true);
    }
	
	if (ros::master::check()) {
		// 2. 检测是否接入罗技方向盘，如果接入，自动切换成方向盘控制
		ros::V_string nodes;
		ros::master::getNodes(nodes);
		std::string joy_node = "joy_node";
		std::string joy_to_car = "joy_to_car_node";
		bool flag = false;		// 标志位，是否检测到 Joy
		for (auto n: nodes) {
			// 只要检测到 "joy_node" 和 "joy_to_car_node" 之一，就表示罗技方向盘接入
			if (n.find(joy_node) != std::string::npos || 
				n.find(joy_to_car) != std::string::npos) {
				flag = true;
				break;
			}
		}
		if (flag == true) {		// 检测到 Joy 节点
			if (this->hasJoy == false) {
				// 通知 joy_to_car 节点，传递 prefix 
				ros::NodeHandle n;
				ros::ServiceClient client = n.serviceClient<joy_to_car::GetPrefix>("get_prefix");
				joy_to_car::GetPrefix srv;
				srv.request.prefix = this->prefix;
				std::cout << "this->prefix: " << this->prefix << std::endl;
				if (client.call(srv)) {
					// 第一次接入，要弹窗
					this->hasJoy = true;
					this->qnode.shutdownPubTopic();		// 停止发布控制话题
					ui.joy_status->setText("已连接");
					QMessageBox::information(this, "提示", "检测到罗技方向盘，已切换至方向盘控制!");
				} else {
					QMessageBox::information(this, "提示", "请先关闭方向盘，连接小车之后再启动!");
				}
				// 关闭软件 话题发送线程
				// this->stopThread();
			} else {
				// Joy 正在运行，什么也不用做
			}
		} else {
			if (this->hasJoy == true) {
				// 没有检测到 Joy,但是 hasJoy == true。 -> Joy 节点退出
				this->hasJoy = false;
				this->initPubMsg();
				if (this->prefix != "") {	// 说明已经选择了话题播放，恢复话题发布
					qnode.restorePubTopic(this->prefix+'/');					
				}
				ui.joy_status->setText("未连接");
				QMessageBox::information(this, "提示", "方向盘移出，切换至软件控制!");
				// this->startThread();
			} else {
				// 没有接入 Joy，啥也不用做
			}
		}
	}
}


/**
 * @brief 接收车辆配置页面返回信号
 * 
 * @param config 
 */
void MainWindow::connectByConfig(ConfigInfo *config) {
	qDebug() << "--------- Config Info ---------";
    qDebug() << config->rosMasterUri;
    qDebug() << config->localhost;
	// qDebug() << config->imageTopics[0];
	this->qnode.setConfigInfo(config);
    if (!qnode.init()) {
        // 连接失败
		QMessageBox errBox;
		errBox.setText("主节点启动失败，请检查节点地址！");
		errBox.exec();
    } else {
        // 连接成功
        QMessageBox infoBox;
		infoBox.setText("主节点启动成功！");
		infoBox.exec();
		// 更新主页面信息
		ui.master_status->setText("已连接");
		ui.master_uri->setText(config->rosMasterUri);
		ui.localhost->setText(config->localhost);
		ui.nodename->setText(config->nodename);
		// 启动发送小车控制话题线程
		boost::thread send_ctrl_thread(boost::bind(&MainWindow::sendCtrlCmd, this));
		// boost::thread send_io_thread(boost::bind(&MainWindow::sendIOCmd, this));
    }
}


void MainWindow::startThread() {
	std::cout << "启动线程" << std::endl;
	this->pub_thread.reset(new boost::thread(boost::bind(&MainWindow::sendCtrlCmd, this)));
}


void MainWindow::stopThread() {
	if (this->pub_thread) {
		std::cout << "关闭线程" << std::endl;
		this->pub_thread->interrupt();
	}
}


// 重写绘图事件
void MainWindow::paintEvent(QPaintEvent* event) {

}


QPixmap MainWindow::fixImage(cv::Mat img, int cam_index) {
	cv::Mat imageCalib;     // 畸变修复后的图像
	if (this->vec_hasLoadCameraMatrix[cam_index]) {
		cv::remap(img, imageCalib, this->vec_map1[cam_index], this->vec_map2[cam_index], INTER_LINEAR);
	} else {
		imageCalib = img;
	}
	QImage qImg = QImage((const unsigned char*)(imageCalib.data), imageCalib.cols, 
                                imageCalib.rows, imageCalib.step, QImage::Format_RGB888);
	QPixmap pixmap = QPixmap::fromImage(qImg);
	return pixmap;
}


// 中间大屏显示的相机图像
void MainWindow::setImage_0(cv::Mat img) {
	QPixmap pixmap = this->fixImage(img, 0);
	// 设置中间屏幕展示
	QPixmap main_pixmap = pixmap.scaled(this->pageC->ui->main_label->size(), 
										Qt::KeepAspectRatio, Qt::SmoothTransformation);
	this->pageC->ui->main_label->setScaledContents(true);
	this->pageC->ui->main_label->setPixmap(main_pixmap);
}


// 左屏幕上方相机
void MainWindow::setImage_1(cv::Mat img) {
	QPixmap pixmap = this->fixImage(img, 1);

	// 左上视频
	QPixmap left_pixmap_up = pixmap.scaled(this->pageL->ui->left_img_up->size(), 
										Qt::KeepAspectRatio, Qt::SmoothTransformation);
	this->pageL->ui->left_img_up->setScaledContents(true);
	this->pageL->ui->left_img_up->setPixmap(left_pixmap_up);
}

// 左屏幕下方相机
void MainWindow::setImage_2(cv::Mat img) {
	QPixmap pixmap = this->fixImage(img, 2);
	// 左下视频
	QPixmap left_pixmap_down = pixmap.scaled(this->pageL->ui->left_img_down->size(), 
										Qt::KeepAspectRatio, Qt::SmoothTransformation);
	this->pageL->ui->left_img_down->setScaledContents(true);
	this->pageL->ui->left_img_down->setPixmap(left_pixmap_down);
}

// 右上视频
void MainWindow::setImage_3(cv::Mat img) {
	QPixmap pixmap = this->fixImage(img, 3);
	// 右上视频
	QPixmap right_pixmap_up = pixmap.scaled(ui.right_img_up->size(), 
										Qt::KeepAspectRatio, Qt::SmoothTransformation);
	this->ui.right_img_up->setScaledContents(true);
	this->ui.right_img_up->setPixmap(right_pixmap_up);
}


// 右下视频
void MainWindow::setImage_4(cv::Mat img) {
	QPixmap pixmap = this->fixImage(img, 4);
	// 右下视频
	QPixmap right_pixmap_down = pixmap.scaled(ui.right_img_down->size(), 
										Qt::KeepAspectRatio, Qt::SmoothTransformation);
	this->ui.right_img_down->setScaledContents(true);
	this->ui.right_img_down->setPixmap(right_pixmap_down);
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	// WriteSettings();
	QMainWindow::closeEvent(event);
}

// 关闭软件
void MainWindow::closeWindow() {
	if (p_check_timer->isActive() == true) {
		p_check_timer->stop();
	}
	this->close();
}

}  // namespace parallel_driving

