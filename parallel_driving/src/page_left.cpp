
#include "../include/parallel_driving/page_left.hpp"


using namespace std;

namespace parallel_driving {

PageLeft::PageLeft(QWidget *parent) : QWidget(parent) {

    ui = new Ui::PageLeft();
    ui->setupUi(this);
}

PageLeft::~PageLeft() {
    delete ui;
}


}
