#ifndef parallel_driving_MYLABEL_HPP_
#define parallel_driving_MYLABEL_HPP_

#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <QPainter>
#include <QKeyEvent>

namespace parallel_driving {


class MyLabel : public QLabel
{
    Q_OBJECT

public:
    MyLabel(QWidget *parent = nullptr);
    ~MyLabel();

    void paintEvent(QPaintEvent *);
    void setAngle(double angle);
    

public slots:


private:
    double angle;
    QPixmap disc;
};

}   // namespace

#endif