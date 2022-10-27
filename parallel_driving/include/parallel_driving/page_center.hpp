#ifndef parallel_driving_PAGE_CENTER_HPP_
#define parallel_driving_PAGE_CENTER_HPP_

#include <QWidget>
#include <QString>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "ui_page_center.h"

namespace parallel_driving {


class PageCenter : public QWidget
{
    Q_OBJECT

public:
    explicit PageCenter(QWidget *parent = 0);
    ~PageCenter();

    void setCenterImage(cv::Mat img); 

    void decorateWidget();      // 修饰样式

Q_SIGNALS:


private:


public:
    Ui::PageCenter* ui;
};

}

#endif