
#include "../include/parallel_driving/page_center.hpp"


using namespace std;

namespace parallel_driving {

PageCenter::PageCenter(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageCenter();
    ui->setupUi(this);
    this->setAttribute(Qt::WA_QuitOnClose, false);
    this->setWindowTitle("平行驾驶-中");

    // decorateWidget();
    // this->setWindowFlags(Qt::Window);
    // this->showFullScreen();
}

PageCenter::~PageCenter() {
    delete ui;
}


void PageCenter::keyPressEvent(QKeyEvent *event) {
    // if (event->key() == Qt::Key_Enter) {    // 回车进入全屏
    //     this->setWindowFlags(Qt::Window);
    //     this->showFullScreen();
    // }

    // if (event->key() == Qt::Key_Escape) {   // esc 退出全屏
    //     this->setWindowFlags(Qt::SubWindow);
    //     this->showNormal();
    // }
}


void PageCenter::setCenterImage(cv::Mat img) {
    QImage qImg = QImage((const unsigned char*)(img.data), img.cols, 
                                img.rows, img.step, QImage::Format_RGB888);
	// QImage scaleImg = qImg.scaled(800, 600);
	ui->main_label->setPixmap(QPixmap::fromImage(qImg));
}



void PageCenter::decorateWidget() {
    // 设置边框样式
    ui->main_label->setFrameShape(QFrame::Box);
    ui->main_label->setStyleSheet("border-width: 3px; border-style: solid; border-color: rgb(0, 205, 214);");
    // ui->center_label->setStyleSheet("border: 3px solid red;");
}   


}       // namespace parallel_driving
