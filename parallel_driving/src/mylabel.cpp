

#include "../include/parallel_driving/mylabel.hpp"

namespace parallel_driving {

MyLabel::MyLabel(QWidget *parent) : QLabel(parent)
{
    disc = QPixmap(":/images/left/6.png");
    angle = 0;
}

MyLabel::~MyLabel() {}



void MyLabel::setAngle(double angle) {
    this->angle = angle;
}


/**
 * @brief 在界面绘制事件中写图片旋转绘制代码，绕Widget中心转动
 * 
 */
void MyLabel::paintEvent(QPaintEvent *) {
      /* 这里旋转中心点就是图片中心，所以在缩放图片时候要忽略等比例缩放 */
//    QPainter painter(this);
//    /* 把图片缩放到和组件同样大小 */
//    QPixmap fit_pixmap = disc.scaled(this->size(), 
//                             Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
//    /* 设定旋转中心点 */
//    painter.translate(fit_pixmap.width()/2,fit_pixmap.height()/2);
//    /* 旋转的角度 */
//    painter.rotate(angle);
//    /* 恢复中心点 */
//    painter.translate(-fit_pixmap.width()/2,-fit_pixmap.height()/2);
//    /* 画图操作 */
//    painter.drawPixmap(0,0,fit_pixmap.width(),fit_pixmap.height(), fit_pixmap);


    /* 这里默认旋转位置在组件正中心 */
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing
                           | QPainter::SmoothPixmapTransform);
    // 首先把图片缩放到组件大小尺寸
    QPixmap fit_pixmap = disc.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    //设定旋转中心点，QRectF 即，继承 QRect（Qt 的矩形类）， F 代表精确到浮点类型
    QRect rect((this->width() - fit_pixmap.width()) / 2,
               (this->height() - fit_pixmap.height()) / 2,
               fit_pixmap.width(), fit_pixmap.height());
    // 默认参考点为左上角原点（0,0），因为旋转需要以图形的中心为参考点
    // 我们使用 translate 把参考点设置为 CD 图形的中心点坐标
    painter.translate(0 + rect.x() + rect.width() / 2,
                      0 + rect.y() + rect.height() / 2);
    // 旋转的角度, 旋转时候是以左上角为旋转点（上文设置的中心点）
    painter.rotate(angle);

    // 现在参考点为 CD 图形的中心，我们需要把它设置回原点的位置，
    // 所以需要减去上面加上的数 即将绘图的起点设置回起点
    painter.translate(0 - (rect.x() + rect.width() / 2),
                      0 - (rect.y() + rect.height() / 2));
    //画图操作,在矩形 rect 中画图 disc
    painter.drawPixmap(rect,disc);
}


}


