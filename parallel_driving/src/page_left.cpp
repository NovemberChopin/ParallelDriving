
#include "../include/parallel_driving/page_left.hpp"


using namespace std;

namespace parallel_driving {

PageLeft::PageLeft(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageLeft();
    ui->setupUi(this);
    this->setAttribute(Qt::WA_QuitOnClose, false);

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


/**
 * @brief 更新界面绘制事件
 * 
 * @param angle 小车返回的角度
 * 注意：小车返回的角度和实际旋转的角度存在一个映射关系
 */
void PageLeft::updateSteer(double angle) {
    double base_angle = 120;
    my_label->setAngle(angle + base_angle);
    my_label->update();
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
