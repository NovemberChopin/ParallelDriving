
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/parallel_driving/main_window.hpp"


namespace parallel_driving {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); 
	configP = new ConfigPanel();

	p_velo_timer = new QTimer(this);

	u_int8_t gear = 3;
	u_int8_t brake = 0;
	float velocity = 0.0;
	float steer = 0.0;
	this->ctrl_msg_.ctrl_cmd_gear = gear;
	this->ctrl_msg_.ctrl_cmd_velocity = velocity;
	this->ctrl_msg_.ctrl_cmd_steering = steer;
	this->ctrl_msg_.ctrl_cmd_Brake = brake;

	setWindowIcon(QIcon(":/images/icon.png"));
	// ros 节点信号
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(getImage_0(cv::Mat)), this, SLOT(setImage_0(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_1(cv::Mat)), this, SLOT(setImage_1(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_2(cv::Mat)), this, SLOT(setImage_2(cv::Mat)));
	QObject::connect(&qnode, SIGNAL(getImage_3(cv::Mat)), this, SLOT(setImage_3(cv::Mat)));
	// 登录页面的信号
	QObject::connect(configP, SIGNAL(getConfigInfo(ConfigInfo*)), this, SLOT(connectByConfig(ConfigInfo*)));

	QObject::connect(p_velo_timer, &QTimer::timeout, this, &MainWindow::handleCtrlMsg);

	this->setCentralWidget(this->init_main_page());
}

MainWindow::~MainWindow() {}

// 处理
void MainWindow::handleCtrlMsg() {
	this->velo_cmd_num++;
	u_int8_t gear = 3;		// 默认驻车档
	u_int8_t brake = 0;
	float velocity = 0.0;
	float steer = 0.0;
	// 在此处构造消息
	if (key_up) {		// 前进
		gear = 4;
		velocity = 0.01 * velo_cmd_num;
		if (velocity >= 5.0) velocity = 5.0;
	} else {			// 后退（刹车）
		if (qnode.velo_fb_ > 0) {
			/* 刹车 */
			velocity = velo_cmd_num - 0.01 * velo_cmd_num;
			if (velocity < 0)	velocity = 0;
		} else if (qnode.velo_fb_ = 0) {
			/* 后退 */
			gear = 2;
			velocity = 0.01 * velo_cmd_num;
		}
	}

	this->ctrl_msg_.ctrl_cmd_gear = gear;
	this->ctrl_msg_.ctrl_cmd_velocity = velocity;
	this->ctrl_msg_.ctrl_cmd_steering = steer;
	this->ctrl_msg_.ctrl_cmd_Brake = brake;
	this->qnode.pub_ctrl_cmd.publish(this->ctrl_msg_);
}


void MainWindow::keyPressEvent(QKeyEvent *event) {
	Q_UNUSED(event);

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
			qDebug() << "Key_Up press " << key_up;
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
			key_right = true;
			qDebug() << "Key_right press " << key_right;
			p_steer_timer->start(10);
		}
	}

}


void MainWindow::keyReleaseEvent(QKeyEvent *event) {
	Q_UNUSED(event);

	if (event->key() == Qt::Key_Up) {
		if (event->isAutoRepeat()) return;
		key_up = false;
		qDebug() << "Key_down release" << key_up;
		if (p_velo_timer->isActive() == true) {
			p_velo_timer->stop();
			this->velo_cmd_num = 0;
		}
	} else if (event->key() == Qt::Key_Down) {
		if (event->isAutoRepeat()) return;
		key_down = false;
		qDebug() << "Key_down release" << key_up;
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


QWidget* MainWindow::init_main_page() {
	qDebug() << "init main page";
	QWidget *main_page = new QWidget(this);
	QVBoxLayout *layout = new QVBoxLayout;
	QHBoxLayout *sub_layout = new QHBoxLayout;
	
	this->title_label = new QLabel("远程平行驾驶系统", this);
	this->title_label->setAlignment(Qt::AlignCenter);
	this->title_label->setMinimumHeight(60);
	QSizePolicy policy = title_label->sizePolicy();
	policy.setVerticalPolicy(QSizePolicy::Fixed);
	policy.setHorizontalStretch(3);
	this->title_label->setSizePolicy(policy);
	QFont font;
	font.setPointSize(20);
	QPalette pa;
	pa.setColor(QPalette::WindowText, Qt::blue);
	this->title_label->setFont(font);
	this->title_label->setPalette(pa);
	this->btn_config = new QPushButton("配置ROS", this);
	QObject::connect(this->btn_config, &QPushButton::clicked, this, &MainWindow::openConfigPanel);

	this->main_label = new QLabel("Camera", this);
	this->main_label->setAlignment(Qt::AlignCenter);

	sub_layout->addStretch();
	sub_layout->addWidget(this->title_label);
	sub_layout->addStretch();
	sub_layout->addWidget(this->btn_config);
	layout->addLayout(sub_layout);
	layout->addWidget(main_label);
	main_page->setLayout(layout);
	return main_page;
}


void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
	this->close();
}


void MainWindow::openConfigPanel() {
	ROS_INFO("Open ROS config panel");
	configP->show();
}


void MainWindow::connectByConfig(ConfigInfo *config) {
	qDebug() << "--------- Config Info ---------";
    qDebug() << config->rosMasterUri;
    qDebug() << config->localhost;
	qDebug() << config->imageTopics[0];
	this->qnode.setConfigInfo(config);
    if (!qnode.init(config->rosMasterUri.toStdString(), 
					config->localhost.toStdString())) {
        // 连接失败
        this->showNoMasterMessage();
    } else {
        // 连接成功
        this->btn_config->setEnabled(false);
    }
}


void MainWindow::setImage_0(cv::Mat img) {
	QImage qImg = QImage((const unsigned char*)(img.data), img.cols, 
                                img.rows, img.step, QImage::Format_RGB888);
	// QImage scaleImg = qImg.scaled(800, 600);
	this->main_label->setPixmap(QPixmap::fromImage(qImg));
}

void MainWindow::setImage_1(cv::Mat img) {
	ROS_INFO("img index: %d, size: %d | %d", 1, img.rows, img.cols);
}


void MainWindow::setImage_2(cv::Mat img) {
	ROS_INFO("img index: %d, size: %d | %d", 2, img.rows, img.cols);
}


void MainWindow::setImage_3(cv::Mat img) {
	ROS_INFO("img index: %d, size: %d | %d", 3, img.rows, img.cols);
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	// WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace parallel_driving

