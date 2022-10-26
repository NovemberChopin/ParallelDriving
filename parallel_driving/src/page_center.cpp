
#include "../include/parallel_driving/page_center.hpp"


using namespace std;

namespace parallel_driving {

PageCenter::PageCenter(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageCenter();
    ui->setupUi(this);
}

PageCenter::~PageCenter() {
    delete ui;
}


}       // namespace parallel_driving
