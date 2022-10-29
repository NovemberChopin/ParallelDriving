#ifndef parallel_driving_DASHBOARD_HPP_
#define parallel_driving_DASHBOARD_HPP_

#include <QWidget>
#include <QPainter>
#include <QtMath>

#include <iostream>
#include <iomanip>
#include <sstream>

namespace parallel_driving {

/**
 * @brief The dash panel example
 * https://blog.csdn.net/weixin_36302584/article/details/109890336
 */
class Dashboard : public QWidget
{
    Q_OBJECT

public:
    explicit Dashboard(QWidget *parent = 0);
    ~Dashboard();

    void setValue(double val);

protected:
    void paintEvent(QPaintEvent *);

    void drawCrown(QPainter *painter);
    void drawBackground(QPainter *painter);
    void drawScale(QPainter *painter);
    void drawScaleNum(QPainter *painter);
    void drawTitle(QPainter *painter);
    void drawIndicator(QPainter *painter);
    void drawNumericValue(QPainter *painter);

private:

    QColor m_background;
    QColor m_foreground;

    int m_maxValue;
    int m_minValue;
    int m_startAngle;
    int m_endAngle;

    int m_scaleMajor;
    int m_scaleMinor;
    double m_value;
    int m_precision;
    QTimer *m_updateTimer;
    QString m_units;
    QString m_title;

public Q_SLOTS:
    void UpdateAngle();

};


}   // namespace parallel_driving

#endif