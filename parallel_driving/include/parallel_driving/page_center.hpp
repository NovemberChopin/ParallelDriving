#ifndef parallel_driving_PAGE_CENTER_HPP_
#define parallel_driving_PAGE_CENTER_HPP_

#include <QWidget>
#include <QString>

#include "ui_page_center.h"

namespace parallel_driving {


class PageCenter : public QWidget
{
    Q_OBJECT

public:
    explicit PageCenter(QWidget *parent = 0);
    ~PageCenter();


Q_SIGNALS:


private:

    Ui::PageCenter* ui;
};

}

#endif