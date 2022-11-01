
#include "../include/parallel_driving/page_left.hpp"


using namespace std;

namespace parallel_driving {

PageLeft::PageLeft(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageLeft();
    ui->setupUi(this);
    this->setAttribute(Qt::WA_QuitOnClose, false);
    this->setWindowTitle("平行驾驶-左");

    timer = new QTimer();
    timer->start(1000);
    QObject::connect(timer, &QTimer::timeout, this, &PageLeft::timeout_slot);

    // 尽量把组件放在布局里面，这样可以自适应页面
    QVBoxLayout *layout_1 = new QVBoxLayout();
    dash_1 = new Dashboard();
    layout_1->addWidget(dash_1);
    ui->dash_1->setLayout(layout_1);

    // 第二个 dashboard
    QVBoxLayout *layout_2 = new QVBoxLayout();
    dash_2 = new Dashboard();
    layout_2->addWidget(dash_2);
    ui->dash_2->setLayout(layout_2);

    // 添加方向盘
    my_label = new MyLabel();
    QVBoxLayout *layout_3 = new QVBoxLayout();
    layout_3->addWidget(my_label);
    ui->show_steer->setLayout(layout_3);


    // 测试
    // ui->gear_P->setStyleSheet("border-image: url(:/images/left/13.png);");
}


PageLeft::~PageLeft() {
    delete ui;
}


void PageLeft::timeout_slot() {
    QTime timeNow = QTime::currentTime();
    ui->cur_time->setText(tr("%1").arg(timeNow.toString()));
}


/**
 * @brief 更新界面绘制事件
 * 
 * @param angle 小车返回的角度
 * 注意：小车返回的角度和实际旋转的角度存在一个映射关系, 
 * scale 表示小车实际转向与方向盘旋转的对应关系 [-25, 25] -> [-100, 100]
 */
void PageLeft::updateSteer(double angle) {
    if (angle > 0) {
        ui->steer_left->setStyleSheet(this->steer_dark);
        ui->steer_right->setStyleSheet(this->steer_green);
    } else if (angle < 0) {
        ui->steer_left->setStyleSheet(this->steer_green);
        ui->steer_right->setStyleSheet(this->steer_dark);
    } else {
        ui->steer_left->setStyleSheet(this->steer_dark);
        ui->steer_right->setStyleSheet(this->steer_dark);
    }
    int scale = 4;
    double base_angle = 115;    // 图片初始时的偏的角度
    my_label->setAngle(angle * 4 + base_angle);
    my_label->update();

    QString str = QString("%1").arg(angle);
    ui->steer_val->setText(str);
}


void PageLeft::setGear_P() {
    ui->gear_P->setStyleSheet(this->gear_P_s);
    ui->gear_R->setStyleSheet(this->gear_R);
    ui->gear_N->setStyleSheet(this->gear_N);
    ui->gear_D->setStyleSheet(this->gear_D);
}

void PageLeft::setGear_R() {
    ui->gear_P->setStyleSheet(this->gear_P);
    ui->gear_R->setStyleSheet(this->gear_R_s);
    ui->gear_N->setStyleSheet(this->gear_N);
    ui->gear_D->setStyleSheet(this->gear_D);
}

void PageLeft::setGear_N() {
    ui->gear_P->setStyleSheet(this->gear_P);
    ui->gear_R->setStyleSheet(this->gear_R);
    ui->gear_N->setStyleSheet(this->gear_N_s);
    ui->gear_D->setStyleSheet(this->gear_D);
}

void PageLeft::setGear_D() {
    ui->gear_P->setStyleSheet(this->gear_P);
    ui->gear_R->setStyleSheet(this->gear_R);
    ui->gear_N->setStyleSheet(this->gear_N);
    ui->gear_D->setStyleSheet(this->gear_D_s);
}


}
