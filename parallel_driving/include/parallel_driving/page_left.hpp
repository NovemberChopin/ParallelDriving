#ifndef parallel_driving_PAGE_LEFT_HPP_
#define parallel_driving_PAGE_LEFT_HPP_

#include <QWidget>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>
#include <QtMath>

#include "ui_page_left.h"
#include "dashboard.hpp"
#include "mylabel.hpp"

namespace parallel_driving {


class PageLeft : public QWidget
{
    Q_OBJECT

public:
    explicit PageLeft(QWidget *parent = 0);
    ~PageLeft();

    void updateSteer(double angle);

    void setGear_P();
    void setGear_R();
    void setGear_N();
    void setGear_D();

    Ui::PageLeft* ui;
    Dashboard *dash_1;
    Dashboard *dash_2;

    QTimer *timer;

    // 计时器相关
    // QTimer *timer;
    // int minute;
    // int second;

private:
    QString gear_P = "image: url(:/images/left/14.png);";
    QString gear_P_s = "border-image: url(:/images/left/13.png);";
    QString gear_R = "image: url(:/images/left/12.png);";
    QString gear_R_s = "border-image: url(:/images/left/11.png);";
    QString gear_N = "image: url(:/images/left/16.png);";
    QString gear_N_s = "border-image: url(:/images/left/15.png);";
    QString gear_D = "image: url(:/images/left/19.png);";
    QString gear_D_s = "border-image: url(:/images/left/17.png);";

    QString steer_dark = "image: url(:/images/left/27.png);";
    QString steer_green = "image: url(:/images/left/28.png);";

    MyLabel *my_label;

public Q_SLOTS:
    void timeout_slot();

};

}

#endif