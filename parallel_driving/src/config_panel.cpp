
#include <QDebug>
#include <QString>
#include <QPushButton>
#include <QDesktopWidget>
#include "../include/parallel_driving/config_panel.hpp"

using namespace std;

namespace parallel_driving {

ConfigPanel::ConfigPanel(QWidget *parent) :
    QWidget(parent)
    // ui(new ConfigPanel)
{
    ui = new Ui::ConfigPanel();
    ui->setupUi(this);

    QObject::connect(ui->btn_connect, &QPushButton::clicked, this, &ConfigPanel::ros_connect_clicked);
    // QObject::connect(ui->btn_refresh, &QPushButton::clicked, this, &ConfigPanel::refresh_node);
    // QObject::connect(ui->btn_return, &QPushButton::clicked, this, &ConfigPanel::closeConfigPanel);

    ui->rosMasterUri->setText("http://192.168.50.23:11311");
    ui->localhost->setText("http://192.168.50.23");
    // ui->ros_topic_1->setText("/hik_cam_node/hik_camera");
    // ui->ros_topic_2->setText("/hik_cam_node/hik_camera2");
    // ui->ros_topic_3->setText("/hik_cam_node/hik_camera3");
    // ui->ros_topic_4->setText("/hik_cam_node/hik_camera4");
    // ui->ros_topic_5->setText("/hik_cam_node/hik_camera5");
    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {

    // 设置窗体居中显示，并且不能更改大小
    this->setFixedSize(600, 400);
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);

    //设置窗体阻塞所有窗体
    setWindowModality(Qt::WindowModality::ApplicationModal);
    //设置窗体标题栏为没有按钮
    // setWindowButtonType(QBasePara::TypeDialog::None_Dialog);
    //设置窗体关闭后是否释放内存
	setAttribute(Qt::WA_DeleteOnClose, false);
    // ui->ros_connect->setStyleSheet(" border: 2px solid #000000");

    // 刷新
    // this->refresh_node();
    // 初始化 ComboBox
    

}

void ConfigPanel::ros_connect_clicked() {
    ConfigInfo *configInfo = new ConfigInfo();
    configInfo->rosMasterUri = ui->rosMasterUri->text();
    configInfo->localhost = ui->localhost->text();

    // QString topic_1 = ui->ros_topic_1->text();
    // if (!topic_1.isEmpty())
    //     configInfo->imageTopics.push_back(topic_1);
    // QString topic_2 = ui->ros_topic_2->text();
    // if (!topic_2.isEmpty())
    //     configInfo->imageTopics.push_back(topic_2);
    // QString topic_3 = ui->ros_topic_3->text();
    // if (!topic_3.isEmpty())
    //     configInfo->imageTopics.push_back(topic_3);    
    // QString topic_4 = ui->ros_topic_4->text();
    // if (!topic_4.isEmpty())
    //     configInfo->imageTopics.push_back(topic_4);
    // QString topic_5 = ui->ros_topic_5->text();
    // if (!topic_5.isEmpty())
    //     configInfo->imageTopics.push_back(topic_5);

    // 手动填入话题数据
    for (int i=0; i<5; i++) {
        configInfo->imageTopics.push_back(QString::fromStdString("/hik_cam_node/hik_camera"));
    }
    Q_EMIT getConfigInfo(configInfo);

    this->close();
}


}       // namespace
