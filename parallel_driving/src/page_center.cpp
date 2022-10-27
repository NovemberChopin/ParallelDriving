
#include "../include/parallel_driving/page_center.hpp"


using namespace std;

namespace parallel_driving {

PageCenter::PageCenter(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageCenter();
    ui->setupUi(this);
    this->setAttribute(Qt::WA_QuitOnClose, false);

    // decorateWidget();
}

PageCenter::~PageCenter() {
    delete ui;
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
